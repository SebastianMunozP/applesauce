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
	crankRevolutions = 2  // 23
	crankStepMm      = 1.0
)

// Crank drives the peeling arm through a spiral path to peel the apple.
// The spiral has a 59.5mm radius, 6mm pitch per revolution, for 23 revolutions.
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

	if _, err := r.peelingGripper.Grab(ctx, nil); err != nil {
		return fmt.Errorf("grasp crank handle: %w", err)
	}

	// Query the gripper's current world-frame orientation to hold constant
	// throughout the spiral.
	gripperPoseInFrame, err := r.motion.GetPose(ctx, "peeling-gripper", "world", nil, nil)
	if err != nil {
		return fmt.Errorf("get gripper orientation: %w", err)
	}
	gripperPose := gripperPoseInFrame.Pose()
	gp := gripperPose.Point()
	gov := gripperPose.Orientation().OrientationVectorDegrees()
	r.logger.Infof("Gripper start pose: X=%.3f Y=%.3f Z=%.3f OX=%.4f OY=%.4f OZ=%.4f Theta=%.4f",
		gp.X, gp.Y, gp.Z, gov.OX, gov.OY, gov.OZ, gov.Theta)
	crankOrientation := gripperPose.Orientation()

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

		sp := stepPose.Point()
		sov := stepPose.Orientation().OrientationVectorDegrees()
		r.logger.Infof("Crank step %d goal: X=%.3f Y=%.3f Z=%.3f OX=%.4f OY=%.4f OZ=%.4f Theta=%.4f",
			step, sp.X, sp.Y, sp.Z, sov.OX, sov.OY, sov.OZ, sov.Theta)

		if err := r.moveLinear(ctx, "peeling-gripper", stepPose, nil); err != nil {
			return fmt.Errorf("crank step %d: %w", step, err)
		}

		// Log progress each revolution.
		if step > 0 && step%stepsPerRev == 0 {
			rev := step / stepsPerRev
			r.logger.Infof("Crank revolution %d/%d complete", rev, crankRevolutions)
		}
	}

	// Release crank handle.
	if err := r.peelingGripper.Open(ctx, nil); err != nil {
		r.logger.Warnf("Failed to release crank handle: %v", err)
	}

	r.logger.Info("Cranking complete")
	return nil
}
