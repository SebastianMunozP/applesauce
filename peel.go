package applesauce

import (
	"context"
	"fmt"
	"math"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/spatialmath"
)

// Peel orients the grasped apple so the stab vector aligns with the peeler spikes,
// then lowers it onto the peeling machine.
func Peel(ctx context.Context, r *Robot) error {
	// Verify we have positions needed for peeling.
	if PeelerDevicePose == nil {
		r.logger.Warn("PeelerDevicePose not configured (stub); skipping peel")
		return nil
	}
	if PeelerJawsPose == nil {
		r.logger.Warn("PeelerJawsPose not configured (stub); skipping peel")
		return nil
	}

	// Compute the orientation to align apple onto spikes.
	// The stab vector should be the apple's axis (calyx→stem = feature vector).
	// If feature vector is not available, use a default orientation (point down).
	var stabOrientation spatialmath.Orientation
	if r.state.FeatureVector != nil {
		stabOrientation = featureVectorToOrientation(*r.state.FeatureVector)
		r.logger.Info("Orienting apple using feature vector for spike alignment")
	} else {
		r.logger.Warn("No feature vector; using default orientation for peeling")
		stabOrientation = &spatialmath.OrientationVectorDegrees{OZ: -1, Theta: 0}
	}

	peelerPoint := PeelerDevicePose.Point()
	jawsPoint := PeelerJawsPose.Point()

	// Move above the peeler device.
	abovePeeler := spatialmath.NewPose(
		r3.Vector{X: peelerPoint.X, Y: peelerPoint.Y, Z: peelerPoint.Z + 150},
		stabOrientation,
	)
	r.logger.Info("Moving to above peeler")
	if err := r.moveFree(ctx, "xarm7", abovePeeler, nil); err != nil {
		return fmt.Errorf("move above peeler: %w", err)
	}

	// Linear descent into the jaws.
	jawsPose := spatialmath.NewPose(jawsPoint, stabOrientation)
	r.logger.Info("Lowering into peeler jaws")
	if err := r.moveLinear(ctx, "xarm7", jawsPose, nil, 1); err != nil {
		return fmt.Errorf("lower into jaws: %w", err)
	}

	// Push onto spikes.
	spikePose := spatialmath.NewPose(peelerPoint, stabOrientation)
	r.logger.Info("Pushing apple onto spikes")
	if err := r.moveLinear(ctx, "xarm7", spikePose, nil, 1); err != nil {
		return fmt.Errorf("push onto spikes: %w", err)
	}

	// Open gripper to release apple onto spikes.
	if err := r.appleGripper.Open(ctx, nil); err != nil {
		return fmt.Errorf("release apple: %w", err)
	}

	// Retreat upward.
	r.logger.Info("Retreating from peeler")
	if err := r.moveLinear(ctx, "xarm7", abovePeeler, nil, 1); err != nil {
		return fmt.Errorf("retreat from peeler: %w", err)
	}

	r.logger.Info("Apple placed on peeler successfully")
	return nil
}

// featureVectorToOrientation converts a feature vector (calyx→stem direction)
// into an arm orientation that will align the apple's axis with the peeler spike axis.
// The spike axis is assumed to point in +X (horizontal into the machine).
func featureVectorToOrientation(featureVec r3.Vector) spatialmath.Orientation {
	// Target: align the feature vector with +X (spike direction).
	// Compute the rotation from the feature vector to +X.
	target := r3.Vector{X: 1, Y: 0, Z: 0}

	// Cross product gives rotation axis.
	cross := r3.Vector{
		X: featureVec.Y*target.Z - featureVec.Z*target.Y,
		Y: featureVec.Z*target.X - featureVec.X*target.Z,
		Z: featureVec.X*target.Y - featureVec.Y*target.X,
	}
	crossNorm := cross.Norm()

	// Dot product gives cos(angle).
	dot := featureVec.X*target.X + featureVec.Y*target.Y + featureVec.Z*target.Z

	if crossNorm < 1e-6 {
		// Vectors are parallel or anti-parallel.
		if dot > 0 {
			// Already aligned.
			return &spatialmath.OrientationVectorDegrees{OX: 1, Theta: 0}
		}
		// Anti-parallel: rotate 180 degrees around Z.
		return &spatialmath.OrientationVectorDegrees{OZ: 1, Theta: 180}
	}

	angle := math.Atan2(crossNorm, dot)
	axis := cross.Mul(1.0 / crossNorm)

	return &spatialmath.R4AA{
		Theta: angle,
		RX:    axis.X,
		RY:    axis.Y,
		RZ:    axis.Z,
	}
}
