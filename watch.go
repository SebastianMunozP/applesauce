package applesauce

import (
	"context"
	"fmt"
	"os"
	"time"

	"github.com/golang/geo/r3"

	applepose "github.com/biotinker/applesauce/apple_pose"
	"go.viam.com/rdk/pointcloud"
)

// Watch moves both arms to their viewing positions and polls the primary camera
// for a bowl of apples. Returns when a bowl is detected or the context is cancelled.
func Watch(ctx context.Context, r *Robot) error {
	// Move primary arm to viewing position.
	r.logger.Info("Moving primary arm to viewing position")
	if err := r.moveArmDirectToJoints(ctx, r.primaryArm, PrimaryViewingJoints); err != nil {
		return err
	}

	// Move secondary arm to viewing position (stub — position not yet recorded).
	if SecondaryViewingJoints != nil && r.secondaryArm != nil {
		r.logger.Info("Moving secondary arm to viewing position")
		if err := r.moveArmDirectToJoints(ctx, r.secondaryArm, SecondaryViewingJoints); err != nil {
			return err
		}
	} else {
		r.logger.Warn("SecondaryViewingJoints not recorded; skipping secondary arm positioning")
	}

	// Check that we have a camera to poll.
	if r.primaryCam == nil {
		r.logger.Warn("Primary camera not available; skipping bowl detection (stub)")
		return nil
	}

	// Poll camera every 3 seconds until a bowl is detected.
	r.logger.Info("Watching for bowl of apples...")
	ticker := time.NewTicker(3 * time.Second)
	defer ticker.Stop()

	for {
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-ticker.C:
		}

		cloud, err := r.primaryCam.NextPointCloud(ctx, nil)
		if err != nil {
			r.logger.Warnf("Camera error: %v", err)
			continue
		}

		r.logger.Infof("Point cloud has %d points, downsampling...", cloud.Size())

		// Downsample to ~30K points for faster processing
		downsampled := pointcloud.NewBasicEmpty()
		step := cloud.Size() / 30000
		if step < 1 {
			step = 1
		}
		i := 0
		cloud.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
			if i%step == 0 {
				err := downsampled.Set(p, d)
				if err != nil {
					r.logger.Warnf("Failed to add point: %v", err)
				}
			}
			i++
			return true
		})

		r.logger.Infof("Downsampled to %d points, processing...", downsampled.Size())
		result, err := r.detector.Detect(ctx, downsampled)
		r.logger.Info("Detection complete")
		if err != nil {
			r.logger.Warnf("Detection failed: %v", err)
			continue
		}

		// Log what we detected, even if bowl not confirmed
		if len(result.Bowl.Apples) > 0 {
			r.logger.Infof("Detected %d apple(s) but BowlDetected=%v", len(result.Bowl.Apples), result.BowlDetected)
			for i, apple := range result.Bowl.Apples {
				pos := apple.Pose.Point()
				r.logger.Debugf("  Apple %d: center=(%.1f, %.1f, %.1f) radius=%.1fmm visible=%.0f%%",
					i, pos.X, pos.Y, pos.Z, apple.Radius, apple.VisibleFraction*100)
			}
		} else {
			r.logger.Warn("No apples detected in point cloud")
		}

		if result.BowlDetected {
			r.logger.Infof("Bowl detected with %d apples!", len(result.Bowl.Apples))
			r.state.LastDetection = result

			// Save camera-frame point clouds BEFORE transformation.
			if err := savePointClouds(r, downsampled, result, "camera"); err != nil {
				r.logger.Warnf("Failed to save camera-frame point clouds: %v", err)
			}

			// Transform point clouds to world frame.
			if err := transformPointCloudsToWorldFrame(ctx, r, downsampled, result); err != nil {
				r.logger.Warnf("Failed to transform point clouds: %v", err)
			} else {
				// Save world-frame point clouds AFTER transformation.
				if err := savePointClouds(r, downsampled, result, "world"); err != nil {
					r.logger.Warnf("Failed to save world-frame point clouds: %v", err)
				}
			}

			return nil
		}
	}
}

// savePointClouds saves the camera point cloud and individual apple point clouds to PCD files.
// frameType should be "camera" or "world" to distinguish between reference frames.
func savePointClouds(r *Robot, cameraCloud pointcloud.PointCloud, result *applepose.DetectionResult, frameType string) error {
	outputDir := "pointclouds"
	if err := os.MkdirAll(outputDir, 0o755); err != nil {
		return fmt.Errorf("create output dir: %w", err)
	}

	// Save full camera point cloud (only for camera frame; world frame is handled separately)
	if frameType == "camera" {
		cameraPath := fmt.Sprintf("%s/camera_full_%s.pcd", outputDir, frameType)
		if err := savePointCloudToPCD(cameraCloud, cameraPath); err != nil {
			return fmt.Errorf("save camera cloud: %w", err)
		}
		r.logger.Infof("Saved %s-frame camera point cloud to %s (%d points)", frameType, cameraPath, cameraCloud.Size())
	}

	// Save individual apple point clouds.
	for i, apple := range result.Bowl.Apples {
		if apple.Points != nil && apple.Points.Size() > 0 {
			applePath := fmt.Sprintf("%s/apple_%s_%s.pcd", outputDir, apple.ID, frameType)
			if err := savePointCloudToPCD(apple.Points, applePath); err != nil {
				r.logger.Warnf("Failed to save apple %s cloud: %v", apple.ID, err)
				continue
			}
			r.logger.Infof("Saved %s-frame apple %d point cloud to %s (%d points)", frameType, i, applePath, apple.Points.Size())
		}
	}

	return nil
}

// savePointCloudToPCD writes a point cloud to a PCD file in binary format.
func savePointCloudToPCD(cloud pointcloud.PointCloud, path string) error {
	file, err := os.Create(path)
	if err != nil {
		return fmt.Errorf("create file: %w", err)
	}
	defer file.Close()

	if err := pointcloud.ToPCD(cloud, file, pointcloud.PCDBinary); err != nil {
		return fmt.Errorf("write PCD: %w", err)
	}

	return nil
}

// transformPointCloudsToWorldFrame transforms the camera point cloud and apple point clouds
// from camera frame to world frame. This modifies the point clouds in place.
func transformPointCloudsToWorldFrame(ctx context.Context, r *Robot, cameraCloud pointcloud.PointCloud, result *applepose.DetectionResult) error {
	if r.primaryCam == nil {
		return fmt.Errorf("no primary camera available")
	}

	// Get the camera's pose in world frame via the robot's frame system.
	// frameSystem, err := r.machine.FrameSystemConfig(ctx)
	// if err != nil {
	// 	return fmt.Errorf("get frame system: %w", err)
	// }

	// Find the camera's transform in the frame system.
	// var cameraPoseInWorld spatialmath.Pose
	// found := false
	// for _, part := range frameSystem.Parts {
	// 	if part.FrameConfig.Name() == "primary-cam" {
	// 		cameraPoseInWorld = part.FrameConfig.Pose()
	// 		found = true
	// 		r.logger.Debugf("Camera pose in world: %v", cameraPoseInWorld)
	// 		break
	// 	}
	// }

	// if !found {
	// 	return fmt.Errorf("camera frame 'primary-cam' not found in frame system")
	// }

	// Get camera pose in world frame
	cameraPoseInWorld, err := r.fsSvc.GetPose(ctx, r.primaryCam.Name().Name, "", nil, nil)
	if err != nil {
		return err
	}

	// Transform the full camera point cloud to world frame and save it directly
	transformedCameraCloud := pointcloud.NewBasicPointCloud(cameraCloud.Size())
	err = pointcloud.ApplyOffset(cameraCloud, cameraPoseInWorld.Pose(), transformedCameraCloud)
	if err != nil {
		return fmt.Errorf("failed to transform camera point cloud: %w", err)
	}

	// Save the transformed camera cloud (world frame)
	outputDir := "pointclouds"
	cameraWorldPath := fmt.Sprintf("%s/camera_full_world.pcd", outputDir)
	if err := savePointCloudToPCD(transformedCameraCloud, cameraWorldPath); err != nil {
		r.logger.Warnf("Failed to save world-frame camera cloud: %v", err)
	} else {
		r.logger.Infof("Saved world-frame camera point cloud to %s (%d points)", cameraWorldPath, transformedCameraCloud.Size())
	}

	// Transform each apple's point cloud to world frame.
	for i := range result.Bowl.Apples {
		pc := result.Bowl.Apples[i].Points
		pcInWorld := pointcloud.NewBasicPointCloud(pc.Size())
		err = pointcloud.ApplyOffset(pc, cameraPoseInWorld.Pose(), pcInWorld)
		if err != nil {
			return fmt.Errorf("failed to transform apple point cloud: %w", err)
		}
		result.Bowl.Apples[i].Points = pcInWorld

		// // Transform apple point cloud.
		// if result.Bowl.Apples[i].Points != nil {
		// 	transformedCloud := pointcloud.NewBasicEmpty()

		// 	result.Bowl.Apples[i].Points.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
		// 		// Transform point: create pose from point, compose, extract point
		// 		pointPose := spatialmath.NewPoseFromPoint(p)
		// 		worldPointPose := spatialmath.Compose(cameraPoseInWorld, pointPose)
		// 		worldPt := worldPointPose.Point()
		// 		if err := transformedCloud.Set(worldPt, d); err != nil {
		// 			r.logger.Warnf("Failed to add point to transformed cloud: %v", err)
		// 		}
		// 		return true
		// 	})
		// 	result.Bowl.Apples[i].Points = transformedCloud
		// }
	}

	r.logger.Infof("Transformed %d apple point clouds from camera frame to world frame", len(result.Bowl.Apples))
	return nil
}
