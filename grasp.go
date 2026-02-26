package applesauce

import (
	"context"
	"fmt"
	"math"
	"sort"

	"github.com/golang/geo/r3"

	applepose "github.com/biotinker/applesauce/apple_pose"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

const (
	maxGraspAttempts  = 3
	graspApproachMm   = 200.0
	graspFinalOffsetMm = 20.0
	obstacleSafetyMm  = 5.0
)

// Grasp performs multi-angle scanning, selects the best apple, and attempts
// to grasp it with the primary arm. On success, updates the apple pose in state.
func Grasp(ctx context.Context, r *Robot) error {
	// Multi-angle scan to build a merged detection.
	detection, err := multiAngleScan(ctx, r)
	if err != nil {
		return fmt.Errorf("multi-angle scan: %w", err)
	}

	if len(detection.Bowl.Apples) == 0 {
		return fmt.Errorf("no apples found in scan")
	}

	// Transform apple poses from camera frame to world frame.
	if err := transformApplesToWorldFrame(ctx, r, detection.Bowl.Apples); err != nil {
		r.logger.Warnf("Failed to transform apple poses to world frame: %v", err)
		r.logger.Warn("Proceeding with camera-frame positions (may cause unreachable errors)")
	}

	// Select the most accessible apple.
	target := selectApple(detection.Bowl.Apples)
	r.state.TargetApple = &target
	r.state.AppleRadius = target.Radius

	pos := target.Pose.Point()
	r.logger.Infof("Selected apple %s (radius=%.1fmm, visible=%.0f%%, world pos=(%.1f, %.1f, %.1f))",
		target.ID, target.Radius, target.VisibleFraction*100, pos.X, pos.Y, pos.Z)

	// Build obstacle world state from other apples.
	worldState := buildObstacleWorldState(detection.Bowl.Apples, target.ID)

	// Grasp approach pose: pointing straight down.
	appleCenter := target.Pose.Point()
	downOrientation := &spatialmath.OrientationVectorDegrees{OZ: -1, Theta: 0}

	for attempt := 1; attempt <= maxGraspAttempts; attempt++ {
		r.state.GraspAttempts = attempt
		r.logger.Infof("Grasp attempt %d/%d", attempt, maxGraspAttempts)

		// Move to approach position above the apple.
		approachPose := spatialmath.NewPose(
			r3.Vector{X: appleCenter.X, Y: appleCenter.Y, Z: appleCenter.Z + graspApproachMm},
			downOrientation,
		)
		if err := r.moveFree(ctx, "xarm6", approachPose, worldState); err != nil {
			r.logger.Warnf("Failed to move to approach: %v", err)
			continue
		}

		// Open gripper.
		if err := r.appleGripper.Open(ctx, nil); err != nil {
			return fmt.Errorf("open gripper: %w", err)
		}

		// Linear descent to just above the apple.
		graspPose := spatialmath.NewPose(
			r3.Vector{X: appleCenter.X, Y: appleCenter.Y, Z: appleCenter.Z + graspFinalOffsetMm},
			downOrientation,
		)
		if err := r.moveLinear(ctx, "xarm6", graspPose, worldState); err != nil {
			r.logger.Warnf("Failed linear descent: %v", err)
			continue
		}

		// Grab.
		grabbed, err := r.appleGripper.Grab(ctx, nil)
		if err != nil {
			r.logger.Warnf("Grab failed: %v", err)
			continue
		}

		// Retreat upward.
		if err := r.moveLinear(ctx, "xarm6", approachPose, nil); err != nil {
			r.logger.Warnf("Failed to retreat: %v", err)
		}

		if !grabbed {
			r.logger.Warn("Gripper did not detect grasp, retrying")
			continue
		}

		// Verify grasp.
		holding, err := r.appleGripper.IsHoldingSomething(ctx, nil)
		if err != nil {
			r.logger.Warnf("Hold check failed: %v", err)
			continue
		}
		if !holding.IsHoldingSomething {
			r.logger.Warn("Gripper reports not holding anything, retrying")
			continue
		}

		// STUB: visual verification with secondary camera would go here.

		// Update apple pose to current gripper position.
		endPos, err := r.primaryArm.EndPosition(ctx, nil)
		if err != nil {
			r.logger.Warnf("Could not read end position: %v", err)
		} else {
			r.state.ApplePose = endPos
		}

		r.logger.Info("Apple grasped successfully")
		return nil
	}

	return fmt.Errorf("failed to grasp apple after %d attempts", maxGraspAttempts)
}

// multiAngleScan moves the primary arm through scan angles and merges detections.
func multiAngleScan(ctx context.Context, r *Robot) (*applepose.DetectionResult, error) {
	// If no scan angles are configured, use the last detection from Watch.
	if len(PrimaryBowlScanAngles) == 0 {
		r.logger.Warn("No scan angles configured; using last detection from Watch")
		if r.state.LastDetection != nil {
			return r.state.LastDetection, nil
		}

		// If we have a camera, do a single detection from the current position.
		if r.primaryCam == nil {
			return nil, fmt.Errorf("no camera available and no prior detection")
		}
		cloud, err := r.primaryCam.NextPointCloud(ctx, nil)
		if err != nil {
			return nil, fmt.Errorf("camera: %w", err)
		}
		return r.detector.Detect(ctx, cloud)
	}

	// Move through each scan angle and merge detections.
	var merged *applepose.DetectionResult
	for i, joints := range PrimaryBowlScanAngles {
		r.logger.Infof("Scanning angle %d/%d", i+1, len(PrimaryBowlScanAngles))
		if err := r.moveToJoints(ctx, "xarm6", joints); err != nil {
			r.logger.Warnf("Failed to move to scan angle %d: %v", i+1, err)
			continue
		}

		if r.primaryCam == nil {
			continue
		}

		cloud, err := r.primaryCam.NextPointCloud(ctx, nil)
		if err != nil {
			r.logger.Warnf("Camera error at angle %d: %v", i+1, err)
			continue
		}

		result, err := r.detector.DetectWithHistory(ctx, cloud, merged)
		if err != nil {
			r.logger.Warnf("Detection error at angle %d: %v", i+1, err)
			continue
		}
		merged = result
	}

	if merged == nil {
		// Fallback to last detection from Watch.
		if r.state.LastDetection != nil {
			return r.state.LastDetection, nil
		}
		return nil, fmt.Errorf("no detections from any scan angle")
	}

	r.state.LastDetection = merged
	return merged, nil
}

// selectApple chooses the most accessible apple: highest Z (most accessible from above)
// and highest visible fraction.
func selectApple(apples []applepose.Apple) applepose.Apple {
	type scored struct {
		apple applepose.Apple
		score float64
	}

	var candidates []scored
	for _, a := range apples {
		// Score = normalized Z height + visible fraction.
		// Higher Z = more accessible from above.
		score := a.Pose.Point().Z + a.VisibleFraction*100
		candidates = append(candidates, scored{apple: a, score: score})
	}

	sort.Slice(candidates, func(i, j int) bool {
		return candidates[i].score > candidates[j].score
	})

	return candidates[0].apple
}

// buildObstacleWorldState creates a WorldState with other apples as sphere obstacles.
func buildObstacleWorldState(apples []applepose.Apple, excludeID string) *referenceframe.WorldState {
	var geometries []spatialmath.Geometry

	for _, a := range apples {
		if a.ID == excludeID {
			continue
		}
		// Add safety margin to radius.
		radius := a.Radius + obstacleSafetyMm
		sphere, err := spatialmath.NewSphere(
			spatialmath.NewPoseFromPoint(a.Pose.Point()),
			radius,
			fmt.Sprintf("obstacle_%s", a.ID),
		)
		if err != nil {
			continue
		}
		geometries = append(geometries, sphere)
	}

	if len(geometries) == 0 {
		return nil
	}

	geoInFrame := referenceframe.NewGeometriesInFrame("world", geometries)
	ws, _ := referenceframe.NewWorldState([]*referenceframe.GeometriesInFrame{geoInFrame}, nil)
	return ws
}

// pointingDownOrientation returns an orientation vector pointing straight down (-Z).
func pointingDownOrientation() spatialmath.Orientation {
	return &spatialmath.OrientationVectorDegrees{OZ: -1, Theta: 0}
}

// distBetween computes the Euclidean distance between two 3D points.
func distBetween(a, b r3.Vector) float64 {
	return math.Sqrt(
		(a.X-b.X)*(a.X-b.X) +
			(a.Y-b.Y)*(a.Y-b.Y) +
			(a.Z-b.Z)*(a.Z-b.Z),
	)
}

// transformApplesToWorldFrame transforms apple poses from camera frame to world frame.
// It modifies the apple poses in place.
func transformApplesToWorldFrame(ctx context.Context, r *Robot, apples []applepose.Apple) error {
	if r.primaryCam == nil {
		return fmt.Errorf("no primary camera available")
	}

	// Get the camera's pose in world frame via the robot's frame system.
	// The frame system knows the geometric relationship between components.
	frameSystem, err := r.machine.FrameSystemConfig(ctx)
	if err != nil {
		return fmt.Errorf("get frame system: %w", err)
	}

	// Find the camera's transform in the frame system.
	var cameraPoseInWorld spatialmath.Pose
	found := false
	for _, part := range frameSystem.Parts {
		if part.FrameConfig.Name() == "primary-cam" {
			cameraPoseInWorld = part.FrameConfig.Pose()
			found = true
			r.logger.Debugf("Camera pose in world: %v", cameraPoseInWorld)
			break
		}
	}

	if !found {
		return fmt.Errorf("camera frame 'primary-cam' not found in frame system")
	}

	// Transform each apple's pose from camera frame to world frame.
	for i := range apples {
		// Compose: world_T_camera * camera_T_apple = world_T_apple
		apples[i].Pose = spatialmath.Compose(cameraPoseInWorld, apples[i].Pose)

		// Transform apple point cloud to world frame.
		if apples[i].Points != nil {
			transformedCloud := pointcloud.NewBasicEmpty()
			apples[i].Points.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
				// Transform point: create pose from point, compose, extract point
				pointPose := spatialmath.NewPoseFromPoint(p)
				worldPointPose := spatialmath.Compose(cameraPoseInWorld, pointPose)
				worldPt := worldPointPose.Point()
				if err := transformedCloud.Set(worldPt, d); err != nil {
					r.logger.Warnf("Failed to add point to transformed cloud: %v", err)
				}
				return true
			})
			apples[i].Points = transformedCloud
		}

		// Also transform feature poses and point clouds.
		for j := range apples[i].Features {
			apples[i].Features[j].Pose = spatialmath.Compose(cameraPoseInWorld, apples[i].Features[j].Pose)

			// Transform feature point cloud.
			if apples[i].Features[j].Points != nil {
				transformedFeatureCloud := pointcloud.NewBasicEmpty()
				apples[i].Features[j].Points.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
					// Transform point: create pose from point, compose, extract point
					pointPose := spatialmath.NewPoseFromPoint(p)
					worldPointPose := spatialmath.Compose(cameraPoseInWorld, pointPose)
					worldPt := worldPointPose.Point()
					if err := transformedFeatureCloud.Set(worldPt, d); err != nil {
						r.logger.Warnf("Failed to add point to transformed feature cloud: %v", err)
					}
					return true
				})
				apples[i].Features[j].Points = transformedFeatureCloud
			}
		}
	}

	r.logger.Infof("Transformed %d apples (poses and point clouds) from camera frame to world frame", len(apples))
	return nil
}
