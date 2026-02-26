package applesauce

import (
	"context"
	"fmt"
	"math"
	"os"
	"time"

	"github.com/golang/geo/r3"

	applepose "github.com/biotinker/applesauce/apple_pose"
	viz "github.com/viam-labs/motion-tools/client/client"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/spatialmath"
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
		if err := r.confirmSecondaryArmAt(ctx, SecondaryViewingJoints); err != nil {
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

		result, err := r.detectApples(ctx)
		if err != nil {
			r.logger.Warnf("Detection error: %v", err)
			continue
		}

		// Log what we detected, even if bowl not confirmed.
		if len(result.Bowl.Apples) > 0 {
			r.logger.Infof("Detected %d apple(s), BowlDetected=%v", len(result.Bowl.Apples), result.BowlDetected)
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
			// Detection ran on world-frame data, so results are already in world frame.
			r.state.LastDetection = result
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

	// Save full point cloud.
	fullPath := fmt.Sprintf("%s/camera_full_%s.pcd", outputDir, frameType)
	if err := savePointCloudToPCD(cameraCloud, fullPath); err != nil {
		return fmt.Errorf("save full cloud: %w", err)
	}
	r.logger.Infof("Saved %s-frame point cloud to %s (%d points)", frameType, fullPath, cameraCloud.Size())

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

const vizDelay = 300 * time.Millisecond

// visualizeWatch draws the world-frame point cloud, bowl region box, and detection results
// in the motion-tools visualizer.
func visualizeWatch(r *Robot, worldCloud pointcloud.PointCloud, result *applepose.DetectionResult) {
	if err := viz.RemoveAllSpatialObjects(); err != nil {
		r.logger.Warnf("viz: could not clear scene (is motion-tools running?): %v", err)
		return
	}
	time.Sleep(vizDelay)

	// Draw the merged, filtered world-frame point cloud.
	if err := viz.DrawPointCloud("world_cloud", worldCloud, nil); err != nil {
		r.logger.Warnf("viz: could not draw world cloud: %v", err)
		return
	}
	time.Sleep(vizDelay)
	r.logger.Infof("viz: drew world-frame point cloud (%d points)", worldCloud.Size())

	// Draw bowl region box.
	if BowlRegionBox != nil {
		if err := viz.DrawGeometry(BowlRegionBox, "green"); err != nil {
			r.logger.Warnf("viz: could not draw bowl region box: %v", err)
		}
		time.Sleep(vizDelay)
	}

	// Draw support plane as a flat box.
	if result.Bowl.SupportPlane != nil {
		plane := result.Bowl.SupportPlane
		center := plane.Center()
		normal := plane.Normal()
		norm := normal.Norm()
		if norm > 1e-9 {
			normal = normal.Mul(1.0 / norm)
		}

		var t1 r3.Vector
		if math.Abs(normal.X) < 0.9 {
			t1 = normal.Cross(r3.Vector{X: 1, Y: 0, Z: 0})
		} else {
			t1 = normal.Cross(r3.Vector{X: 0, Y: 1, Z: 0})
		}
		t1 = t1.Mul(1.0 / t1.Norm())
		t2 := normal.Cross(t1)

		planeCloud, err := plane.PointCloud()
		if err == nil && planeCloud != nil && planeCloud.Size() > 0 {
			var minT1, maxT1, minT2, maxT2 float64
			first := true
			planeCloud.Iterate(0, 0, func(pt r3.Vector, d pointcloud.Data) bool {
				rel := pt.Sub(center)
				p1 := rel.Dot(t1)
				p2 := rel.Dot(t2)
				if first {
					minT1, maxT1 = p1, p1
					minT2, maxT2 = p2, p2
					first = false
				} else {
					if p1 < minT1 {
						minT1 = p1
					}
					if p1 > maxT1 {
						maxT1 = p1
					}
					if p2 < minT2 {
						minT2 = p2
					}
					if p2 > maxT2 {
						maxT2 = p2
					}
				}
				return true
			})

			width := maxT1 - minT1
			height := maxT2 - minT2
			thickness := 1.0

			ov := &spatialmath.OrientationVector{OX: normal.X, OY: normal.Y, OZ: normal.Z}
			planePose := spatialmath.NewPose(center, ov)
			planeBox, err := spatialmath.NewBox(planePose, r3.Vector{X: width, Y: height, Z: thickness}, "support_plane")
			if err == nil {
				if err := viz.DrawGeometry(planeBox, "gray"); err != nil {
					r.logger.Warnf("viz: could not draw support plane: %v", err)
				} else {
					r.logger.Infof("viz: drew support plane (%.0f x %.0f mm) at (%.1f, %.1f, %.1f)",
						width, height, center.X, center.Y, center.Z)
				}
				time.Sleep(vizDelay)
			}
		}
	}

	// Draw apple spheres and features.
	for i, apple := range result.Bowl.Apples {
		center := apple.Pose.Point()

		sphere, err := spatialmath.NewSphere(
			spatialmath.NewPoseFromPoint(center),
			apple.Radius,
			fmt.Sprintf("apple_%d", i),
		)
		if err != nil {
			r.logger.Warnf("viz: failed to create sphere %d: %v", i, err)
			continue
		}
		if err := viz.DrawGeometry(sphere, "red"); err != nil {
			r.logger.Warnf("viz: could not draw sphere %d: %v", i, err)
			continue
		}
		time.Sleep(vizDelay)
		r.logger.Infof("viz: drew apple %d (radius=%.1fmm) at (%.1f, %.1f, %.1f)",
			i, apple.Radius, center.X, center.Y, center.Z)

		for j, f := range apple.Features {
			fPos := f.Pose.Point()
			color := "white"
			switch f.Feature {
			case applepose.FeatureStem:
				color = "black"
			case applepose.FeatureCalyx:
				color = "green"
			}
			fSphere, err := spatialmath.NewSphere(
				spatialmath.NewPoseFromPoint(fPos),
				5.0,
				fmt.Sprintf("apple_%d_feature_%d", i, j),
			)
			if err != nil {
				continue
			}
			if err := viz.DrawGeometry(fSphere, color); err != nil {
				continue
			}
			time.Sleep(vizDelay)
		}
	}

	r.logger.Info("viz: visualization complete")
}
