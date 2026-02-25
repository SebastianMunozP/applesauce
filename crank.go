package applesauce

import (
	"context"
	"fmt"
	"math"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
)

const (
	crankRadiusMm    = 59.5
	crankPitchMm     = 6.0 // Linear advance per revolution along CrankAxis.
	crankRevolutions = 15  // 23
	crankStepMm      = 1.0
)

// Crank drives the peeling arm through a spiral path to peel the apple.
// The spiral has a 59.5mm radius, 6mm pitch per revolution, for 23 revolutions.
// All waypoints are batched into a single motion.Move call for smooth execution.
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

	// Build an orthonormal basis for the spiral from CrankAxis.
	axis := CrankAxis.Normalize()
	ref := r3.Vector{X: 0, Y: 1, Z: 0}
	if math.Abs(axis.Dot(ref)) > 0.9 {
		ref = r3.Vector{X: 0, Y: 0, Z: 1}
	}
	u := ref.Sub(axis.Mul(axis.Dot(ref))).Normalize() // Gram-Schmidt
	w := axis.Cross(u)

	center := gp.Add(r3.Vector{0, 0, 59.5})
	offset := gp.Sub(center)
	startAngle := math.Atan2(offset.Dot(w), offset.Dot(u))

	// Compute the world-frame pose for a given spiral step.
	poseAtStep := func(step int) spatialmath.Pose {
		angle := startAngle + 2*math.Pi*float64(step)/float64(stepsPerRev)
		revsCompleted := float64(step) / float64(stepsPerRev)
		along := axis.Mul(revsCompleted * crankPitchMm)
		circle := u.Mul(crankRadiusMm * math.Cos(angle)).Add(w.Mul(crankRadiusMm * math.Sin(angle)))
		return spatialmath.NewPose(center.Add(along).Add(circle), crankOrientation)
	}

	// Build the waypoints list: steps 1..totalSteps-2 as intermediate waypoints,
	// step totalSteps-1 as the Destination. All are batched into one Move call so
	// the motion service plans and executes the spiral as a single smooth motion.
	wpList := make([]interface{}, 0, totalSteps-2)
	for step := 1; step < totalSteps-1; step++ {
		pif := referenceframe.NewPoseInFrame("world", poseAtStep(step))
		wpState := armplanning.NewPlanState(
			referenceframe.FrameSystemPoses{"peeling-gripper": pif},
			nil,
		)
		wpList = append(wpList, wpState.Serialize())
	}

	finalPIF := referenceframe.NewPoseInFrame("world", poseAtStep(totalSteps-1))

	constraints := motionplan.NewConstraints(
		[]motionplan.LinearConstraint{{
			LineToleranceMm:          1.0,
			OrientationToleranceDegs: 2.0,
		}},
		nil, nil, nil,
	)

	r.logger.Infof("Executing crank spiral: %d waypoints + destination in single motion call", len(wpList))
	if _, err := r.motion.Move(ctx, motion.MoveReq{
		ComponentName: "peeling-gripper",
		Destination:   finalPIF,
		Constraints:   constraints,
		Extra: map[string]interface{}{
			"waypoints": wpList,
		},
	}); err != nil {
		return fmt.Errorf("crank spiral: %w", err)
	}

	r.logger.Info("Cranking complete")
	return nil
}
