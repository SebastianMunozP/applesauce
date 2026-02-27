package applesauce

import (
	"context"
	"fmt"
	"math"
	"time"

	"github.com/golang/geo/r3"
	viz "github.com/viam-labs/motion-tools/client/client"

	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
)

const (
	crankRadiusMm    = 59.5
	crankPitchMm     = 6.0 // Linear advance per revolution along CrankAxis.
	crankRevolutions = 22  // 22
	crankStepMm      = 1.0
)

// Crank drives the peeling arm through a spiral path to peel the apple.
// The spiral trajectory is planned once via DoPlan and cached in memory
// (and optionally on disk when CrankPlansDir is set). Subsequent calls
// replay the cached trajectory via DoExecute.
func Crank(ctx context.Context, r *Robot) error {
	if r.peelingArm == nil {
		r.logger.Warn("Peeling arm not available (stub); skipping crank")
		return nil
	}
	// Open gripper.
	if err := r.appleGripper.Open(ctx, nil); err != nil {
		return fmt.Errorf("open gripper: %w", err)
	}

	time.Sleep(3 * time.Second) // wait for gripper to finish opening

	// Move peeling arm to crank start pose.
	if err := r.moveFree(ctx, r.peelingArm.Name().Name, CrankStartPose, nil); err != nil {
		return fmt.Errorf("move to crank start pose: %w", err)
	}

	_, err := r.peelingArm.DoCommand(ctx, map[string]interface{}{"move_gripper": 100})
	if err != nil {
		r.logger.Warnf("Grab failed: %v", err)
		return err
	}

	// Query the gripper's current world-frame pose — held constant throughout
	// the spiral and used as the retract destination.
	gripperPoseInFrame, err := r.motion.GetPose(ctx, r.peelingGripper.Name().Name, "world", nil, nil)
	if err != nil {
		return fmt.Errorf("get gripper orientation: %w", err)
	}
	gripperPose := gripperPoseInFrame.Pose()
	gp := gripperPose.Point()
	gov := gripperPose.Orientation().OrientationVectorDegrees()
	r.logger.Infof("Gripper start pose: X=%.3f Y=%.3f Z=%.3f OX=%.4f OY=%.4f OZ=%.4f Theta=%.4f",
		gp.X, gp.Y, gp.Z, gov.OX, gov.OY, gov.OZ, gov.Theta)

	// ── Spiral ──────────────────────────────────────────────────────────
	if r.crankSpiralTrajectory == nil {
		r.crankSpiralTrajectory = r.loadCachedTrajectory("crank_spiral.json")
	}
	if r.crankSpiralTrajectory == nil {
		r.logger.Info("Planning crank spiral trajectory (first run; will be cached)")
		spiralPlan := buildSpiralReq(gp, gripperPose.Orientation(), r.peelingGripper.Name().Name, r)
		traj, err := r.doPlan(ctx, spiralPlan)
		if err != nil {
			return fmt.Errorf("plan crank spiral: %w", err)
		}
		r.crankSpiralTrajectory = traj
		r.saveCachedTrajectory("crank_spiral.json", traj)
		r.logger.Infof("Crank spiral planned: %d trajectory steps", len(traj))
	} else {
		r.logger.Info("Executing cached crank spiral trajectory")
	}

	if err := r.doExecute(ctx, r.crankSpiralTrajectory); err != nil {
		return fmt.Errorf("execute crank spiral: %w", err)
	}
	r.logger.Info("Cranking complete")
	return nil
}

// buildSpiralReq constructs the MoveReq for the full crank spiral, with all
// intermediate 1 mm arc steps batched as waypoints.
func buildSpiralReq(gp r3.Vector, crankOrientation spatialmath.Orientation, peelingGripperName string, r *Robot) motion.MoveReq {
	r.logger.Info("Building spiral trajectory")
	circumference := 2 * math.Pi * crankRadiusMm
	stepsPerRev := int(math.Ceil(circumference / crankStepMm))
	totalSteps := stepsPerRev * crankRevolutions

	r.logger.Infof("Circumference: %.2f mm", circumference)
	r.logger.Infof("Steps per revolution: %d", stepsPerRev)
	r.logger.Infof("Total steps: %d", totalSteps)

	axis := CrankAxis.Normalize()
	ref := r3.Vector{X: 0, Y: 1, Z: 0}
	if math.Abs(axis.Dot(ref)) > 0.9 {
		ref = r3.Vector{X: 0, Y: 0, Z: 1}
	}
	u := ref.Sub(axis.Mul(axis.Dot(ref))).Normalize()
	w := axis.Cross(u)

	center := gp.Add(r3.Vector{X: 0, Y: 0, Z: 59.5})
	offset := gp.Sub(center)
	startAngle := math.Atan2(offset.Dot(w), offset.Dot(u))

	poseAtStep := func(step int) spatialmath.Pose {
		angle := startAngle + 2*math.Pi*float64(step)/float64(stepsPerRev)
		revsCompleted := float64(step) / float64(stepsPerRev)
		along := axis.Mul(revsCompleted * crankPitchMm)
		circle := u.Mul(crankRadiusMm * math.Cos(angle)).Add(w.Mul(crankRadiusMm * math.Sin(angle)))
		return spatialmath.NewPose(center.Add(along).Add(circle), crankOrientation)
	}

	wpList := make([]interface{}, 0, totalSteps-2)
	r.logger.Infof("Center: X=%.3f Y=%.3f Z=%.3f", center.X, center.Y, center.Z)
	r.logger.Infof("Offset: X=%.3f Y=%.3f Z=%.3f", offset.X, offset.Y, offset.Z)
	r.logger.Infof("Start angle: %.4f radians", startAngle)
	for step := 1; step < totalSteps-1; step++ {
		pif := referenceframe.NewPoseInFrame("world", poseAtStep(step))
		// Pring and plot 1 every 1000 steps
		if step%100 == 0 {
			r.logger.Infof("Waypoint %d: X=%.3f Y=%.3f Z=%.3f", step, pif.Pose().Point().X, pif.Pose().Point().Y, pif.Pose().Point().Z)
			if err := viz.DrawPoses([]spatialmath.Pose{pif.Pose()}, []string{"blue"}, true); err != nil {
				r.logger.Warnf("Failed to draw waypoint %d: %v", step, err)
			}
		}

		wpState := armplanning.NewPlanState(
			referenceframe.FrameSystemPoses{peelingGripperName: pif},
			nil,
		)
		wpList = append(wpList, wpState.Serialize())
	}

	finalPIF := referenceframe.NewPoseInFrame("world", poseAtStep(totalSteps-1))
	if err := viz.DrawPoses([]spatialmath.Pose{finalPIF.Pose()}, []string{"blue"}, true); err != nil {
		r.logger.Warnf("Failed to draw final waypoint: %v", err)
	}

	constraints := motionplan.NewConstraints(
		[]motionplan.LinearConstraint{{
			LineToleranceMm:          1.0,
			OrientationToleranceDegs: 2.0,
		}},
		nil, nil, nil,
	)

	return motion.MoveReq{
		ComponentName: peelingGripperName,
		Destination:   finalPIF,
		Constraints:   constraints,
		Extra: map[string]interface{}{
			"waypoints": wpList,
		},
	}
}
