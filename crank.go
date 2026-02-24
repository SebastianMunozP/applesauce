package applesauce

import (
	"context"
	"fmt"
	"math"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/spatialmath"
)

const (
	crankRadiusMm    = 59.5
	crankPitchMm     = 6.0 // Linear advance per revolution in -X direction.
	crankRevolutions = 23  // 23
	crankStepMm      = 1.0
)

// Crank drives the peeling arm through a spiral path to peel the apple.
// The spiral has an 80mm radius, 14mm pitch per revolution, for 25 revolutions.
// Each step is approximately 1mm of arc length for smooth motion.
func Crank(ctx context.Context, r *Robot) error {
	if r.peelingArm == nil {
		r.logger.Warn("Peeling arm not available (stub); skipping crank")
		return nil
	}

	// Move peeling arm to crank grasp position.
	if err := r.moveToJoints(ctx, "peeling-arm", PeelingCrankGraspJoints); err != nil {
		return fmt.Errorf("move to crank grasp: %w", err)
	}

	// STUB: grasp the crank handle.
	if r.peelingGripper != nil {
		if _, err := r.peelingGripper.Grab(ctx, nil); err != nil {
			return fmt.Errorf("grasp crank handle: %w", err)
		}
	} else {
		r.logger.Warn("Peeling gripper not available; assuming crank is pre-grasped")
	}

	// Compute spiral parameters.
	circumference := 2 * math.Pi * crankRadiusMm
	stepsPerRev := int(math.Ceil(circumference / crankStepMm))
	totalSteps := stepsPerRev * crankRevolutions

	r.logger.Infof("Cranking: R=%.0fmm, pitch=%.0fmm/rev, %d revs, %d steps/rev, %d total steps",
		crankRadiusMm, crankPitchMm, crankRevolutions, stepsPerRev, totalSteps)

	// Starting position: the crank center in world frame.
	center := CrankCenter

	// Record the starting X position for pitch calculation.
	startX := center.X

	// Hold gripper orientation constant throughout cranking.
	// STUB: orientation should match the crank handle alignment.
	crankOrientation := &spatialmath.OrientationVectorDegrees{OX: 1, Theta: 0}

	for step := 0; step < totalSteps; step++ {
		select {
		case <-ctx.Done():
			return ctx.Err()
		default:
		}

		// Parametric angle.
		angle := 2 * math.Pi * float64(step) / float64(stepsPerRev)

		// Revolutions completed (fractional).
		revsCompleted := float64(step) / float64(stepsPerRev)

		// X advances linearly with each revolution (moves in -X direction).
		x := startX - revsCompleted*crankPitchMm

		// Circle in YZ plane.
		y := center.Y + crankRadiusMm*math.Cos(angle)
		z := center.Z + crankRadiusMm*math.Sin(angle)

		stepPose := spatialmath.NewPose(
			r3.Vector{X: x, Y: y, Z: z},
			crankOrientation,
		)

		// Use arm.MoveToPosition for each step — avoids motion planning overhead
		// for predetermined 1mm incremental steps.
		if err := r.peelingArm.MoveToPosition(ctx, stepPose, nil); err != nil {
			return fmt.Errorf("crank step %d: %w", step, err)
		}

		// Log progress each revolution.
		if step > 0 && step%stepsPerRev == 0 {
			rev := step / stepsPerRev
			r.logger.Infof("Crank revolution %d/%d complete", rev, crankRevolutions)
		}
	}

	// Release crank handle.
	if r.peelingGripper != nil {
		if err := r.peelingGripper.Open(ctx, nil); err != nil {
			r.logger.Warnf("Failed to release crank handle: %v", err)
		}
	}

	r.logger.Info("Cranking complete")
	return nil
}
