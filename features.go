package applesauce

import (
	"context"
	"fmt"
	"math"

	"github.com/golang/geo/r3"

	applepose "github.com/biotinker/applesauce/apple_pose"
	"go.viam.com/rdk/spatialmath"
)

const (
	featureConfidenceThreshold = 0.6
	maxRegraspAttempts         = 4
	regraspRetreatMm           = 50.0
	regraspRotationDeg         = 45.0
)

// IdentifyFeatures extracts stem and calyx positions from the detected apple.
// If features are missing, it uses the secondary camera for additional analysis
// and may trigger regrasps.
func IdentifyFeatures(ctx context.Context, r *Robot) error {
	if r.state.TargetApple == nil {
		return fmt.Errorf("no target apple in state")
	}

	// First, check features already found during detection.
	extractKnownFeatures(r)

	if r.state.StemFound && r.state.CalyxFound {
		r.logger.Info("Both stem and calyx found from detection")
		return nil
	}

	// Try secondary camera analysis.
	if err := analyzeWithSecondaryCamera(ctx, r); err != nil {
		r.logger.Warnf("Secondary camera analysis failed: %v", err)
	}

	if r.state.StemFound && r.state.CalyxFound {
		r.logger.Info("Both features found after secondary camera analysis")
		return nil
	}

	// Regrasp to find missing features.
	for attempt := 1; attempt <= maxRegraspAttempts; attempt++ {
		if r.state.StemFound && r.state.CalyxFound {
			break
		}

		r.logger.Infof("Regrasp attempt %d/%d to find missing features", attempt, maxRegraspAttempts)
		if err := Regrasp(ctx, r); err != nil {
			r.logger.Warnf("Regrasp %d failed: %v", attempt, err)
			continue
		}

		// Re-analyze after regrasp.
		extractKnownFeatures(r)
		if err := analyzeWithSecondaryCamera(ctx, r); err != nil {
			r.logger.Warnf("Post-regrasp analysis failed: %v", err)
		}
	}

	if !r.state.StemFound {
		r.logger.Warn("Stem not found after all attempts; proceeding with best guess")
	}
	if !r.state.CalyxFound {
		r.logger.Warn("Calyx not found after all attempts; proceeding with best guess")
	}

	return nil
}

// extractKnownFeatures populates stem/calyx state from the target apple's feature list.
func extractKnownFeatures(r *Robot) {
	if r.state.TargetApple == nil {
		return
	}

	for _, f := range r.state.TargetApple.Features {
		if f.Confidence < featureConfidenceThreshold {
			continue
		}

		switch f.Feature {
		case applepose.FeatureStem:
			r.state.StemPose = f.Pose
			r.state.StemFound = true
			r.logger.Infof("Stem found (confidence=%.2f)", f.Confidence)
		case applepose.FeatureCalyx:
			r.state.CalyxPose = f.Pose
			r.state.CalyxFound = true
			r.logger.Infof("Calyx found (confidence=%.2f)", f.Confidence)
		}
	}

	// Compute feature vector if both are found.
	if r.state.StemFound && r.state.CalyxFound {
		computeFeatureVector(r)
	}
}

// analyzeWithSecondaryCamera points the secondary camera at the gripper
// and runs single-apple analysis.
func analyzeWithSecondaryCamera(ctx context.Context, r *Robot) error {
	if r.secondaryCam == nil {
		r.logger.Warn("Secondary camera not available; skipping additional feature analysis")
		return nil
	}

	// STUB: move secondary arm to point camera at gripper position.
	// For now, just capture from wherever the secondary camera is.

	cloud, err := r.secondaryCam.NextPointCloud(ctx, nil)
	if err != nil {
		return fmt.Errorf("secondary camera: %w", err)
	}

	center := r.state.TargetApple.Pose.Point()
	apple, err := r.detector.AnalyzeSingleApple(ctx, cloud, center, r.state.AppleRadius)
	if err != nil {
		return fmt.Errorf("single apple analysis: %w", err)
	}

	// Merge features: keep higher confidence.
	for _, f := range apple.Features {
		if f.Confidence < featureConfidenceThreshold {
			continue
		}

		switch f.Feature {
		case applepose.FeatureStem:
			if !r.state.StemFound || f.Confidence > featureConfidenceThreshold {
				r.state.StemPose = f.Pose
				r.state.StemFound = true
				r.logger.Infof("Stem found via secondary camera (confidence=%.2f)", f.Confidence)
			}
		case applepose.FeatureCalyx:
			if !r.state.CalyxFound || f.Confidence > featureConfidenceThreshold {
				r.state.CalyxPose = f.Pose
				r.state.CalyxFound = true
				r.logger.Infof("Calyx found via secondary camera (confidence=%.2f)", f.Confidence)
			}
		}
	}

	// Update target apple with new analysis.
	r.state.TargetApple = apple

	if r.state.StemFound && r.state.CalyxFound {
		computeFeatureVector(r)
	}

	return nil
}

// computeFeatureVector calculates the unit vector from calyx to stem.
func computeFeatureVector(r *Robot) {
	stemPt := r.state.StemPose.Point()
	calyxPt := r.state.CalyxPose.Point()
	vec := stemPt.Sub(calyxPt)
	length := vec.Norm()
	if length < 1e-6 {
		r.logger.Warn("Stem and calyx too close to compute feature vector")
		return
	}
	unit := vec.Mul(1.0 / length)
	r.state.FeatureVector = &unit
	r.logger.Infof("Feature vector (calyx→stem): [%.3f, %.3f, %.3f]", unit.X, unit.Y, unit.Z)
}

// Regrasp releases the apple, adjusts the gripper orientation, and re-grasps.
// Strategy 1 (default): rotate end effector 45 degrees around its approach axis.
func Regrasp(ctx context.Context, r *Robot) error {
	r.state.RegraspAttempts++

	// Get current gripper position.
	currentPose, err := r.primaryArm.EndPosition(ctx, nil)
	if err != nil {
		return fmt.Errorf("read end position: %w", err)
	}

	currentPoint := currentPose.Point()

	// Open gripper to release apple.
	if err := r.appleGripper.Open(ctx, nil); err != nil {
		return fmt.Errorf("open gripper: %w", err)
	}

	// Retreat 50mm upward linearly.
	retreatPose := poseAbove(currentPose, regraspRetreatMm)
	if err := r.moveLinear(ctx, "apple-arm", retreatPose, nil, 1); err != nil {
		return fmt.Errorf("retreat: %w", err)
	}

	// Rotate orientation 45 degrees around the Z axis for a different view.
	rotAngle := regraspRotationDeg * math.Pi / 180.0 * float64(r.state.RegraspAttempts)
	rotatedOrientation := &spatialmath.OrientationVectorDegrees{
		OZ:    -1,
		Theta: rotAngle * 180.0 / math.Pi,
	}

	// Approach from the new angle.
	approachPose := spatialmath.NewPose(
		r3.Vector{X: currentPoint.X, Y: currentPoint.Y, Z: currentPoint.Z + regraspRetreatMm},
		rotatedOrientation,
	)
	if err := r.moveFree(ctx, "apple-arm", approachPose, nil); err != nil {
		return fmt.Errorf("move to regrasp approach: %w", err)
	}

	// Descend to grasp position.
	graspPose := spatialmath.NewPose(
		r3.Vector{X: currentPoint.X, Y: currentPoint.Y, Z: currentPoint.Z + graspFinalOffsetMm},
		rotatedOrientation,
	)
	if err := r.moveLinear(ctx, "apple-arm", graspPose, nil, 1); err != nil {
		return fmt.Errorf("regrasp descent: %w", err)
	}

	// Grab.
	grabbed, err := r.appleGripper.Grab(ctx, nil)
	if err != nil {
		return fmt.Errorf("regrasp grab: %w", err)
	}
	if !grabbed {
		return fmt.Errorf("regrasp did not detect grasp")
	}

	// Retreat upward.
	if err := r.moveLinear(ctx, "apple-arm", approachPose, nil, 1); err != nil {
		r.logger.Warnf("Failed to retreat after regrasp: %v", err)
	}

	// Update apple pose.
	endPos, err := r.primaryArm.EndPosition(ctx, nil)
	if err != nil {
		r.logger.Warnf("Could not read end position after regrasp: %v", err)
	} else {
		r.state.ApplePose = endPos
	}

	r.logger.Info("Regrasp successful")
	return nil
}
