package applesauce

import (
	"context"
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils"
)

// Retract presses the release lever, retracts spikes, and returns the secondary arm
// to its viewing position. Unlike ResetMachine, it does not handle the core.
func Retract(ctx context.Context, r *Robot) error {
	if err := pressReleaseLever(ctx, r); err != nil {
		r.logger.Warnf("Release lever: %v", err)
	}

	if SecondaryReleaseLeverApproach != nil {
		if err := returnSecondaryArm(ctx, r); err != nil {
			r.logger.Warnf("Secondary arm return: %v", err)
		}
	}

	r.logger.Info("Retract complete")
	return nil
}

// RemoveApple grasps the peeled apple from the peeler and deposits it in the peeled apple bowl.
func RemoveApple(ctx context.Context, r *Robot) error {
	if PeelerCorePose == nil {
		r.logger.Warn("PeelerCorePose not configured (stub); skipping apple removal")
		return nil
	}
	if PeeledAppleBowlPose == nil {
		r.logger.Warn("PeeledAppleBowlPose not configured (stub); skipping apple removal")
		return nil
	}

	corePoint := PeelerCorePose.Point()

	// Approach the peeled apple with gripper pointing in +X (toward the peeler).
	approachOrientation := &spatialmath.OrientationVectorDegrees{OX: 1, Theta: 0}
	approachPose := spatialmath.NewPose(
		r3.Vector{X: corePoint.X - 100, Y: corePoint.Y, Z: corePoint.Z},
		approachOrientation,
	)

	r.logger.Info("Approaching peeled apple")
	if err := r.moveFree(ctx, "xarm6", approachPose, nil); err != nil {
		return fmt.Errorf("move to apple approach: %w", err)
	}

	// Open gripper.
	if err := r.appleGripper.Open(ctx, nil); err != nil {
		return fmt.Errorf("open gripper for apple removal: %w", err)
	}

	// Linear approach to the apple on the core.
	corePose := spatialmath.NewPose(corePoint, approachOrientation)
	if err := r.moveLinear(ctx, "xarm6", corePose, nil); err != nil {
		return fmt.Errorf("approach peeled apple: %w", err)
	}

	// Grab the apple.
	if _, err := r.appleGripper.Grab(ctx, nil); err != nil {
		return fmt.Errorf("grab peeled apple: %w", err)
	}

	// Pull apple off the core by moving linearly in -X.
	pullPose := spatialmath.NewPose(
		r3.Vector{X: corePoint.X - 150, Y: corePoint.Y, Z: corePoint.Z},
		approachOrientation,
	)
	r.logger.Info("Pulling apple off core")
	if err := r.moveLinear(ctx, "xarm6", pullPose, nil); err != nil {
		return fmt.Errorf("pull apple off core: %w", err)
	}

	// Move to peeled apple bowl and release.
	r.logger.Info("Depositing peeled apple")
	aboveBowl := poseAbove(PeeledAppleBowlPose, 100)
	if err := r.moveFree(ctx, "xarm6", aboveBowl, nil); err != nil {
		return fmt.Errorf("move to peeled apple bowl: %w", err)
	}

	if err := r.appleGripper.Open(ctx, nil); err != nil {
		return fmt.Errorf("release peeled apple: %w", err)
	}

	r.logger.Info("Peeled apple deposited")
	return nil
}

// ResetMachine resets the peeling machine by removing the core, pressing the release lever,
// retracting the spikes, and returning all arms to their starting positions.
func ResetMachine(ctx context.Context, r *Robot) error {
	// Step 1: Primary arm grabs core from peeler.
	if err := grabCore(ctx, r); err != nil {
		r.logger.Warnf("Core removal: %v", err)
	}

	// Step 2: Press release lever, retract spikes, return secondary arm.
	if err := Retract(ctx, r); err != nil {
		r.logger.Warnf("Retract: %v", err)
	}

	// Step 3: Primary arm deposits core and returns to viewing position.
	if err := depositCoreAndReturn(ctx, r); err != nil {
		r.logger.Warnf("Core deposit: %v", err)
	}

	// Reset state for next cycle.
	r.resetState()
	r.logger.Info("Machine reset complete")
	return nil
}

// grabCore uses the primary arm to approach and grab the core from the peeler.
func grabCore(ctx context.Context, r *Robot) error {
	if PeelerCorePose == nil {
		r.logger.Warn("PeelerCorePose not configured (stub); skipping core grab")
		return nil
	}

	corePoint := PeelerCorePose.Point()
	approachOrientation := &spatialmath.OrientationVectorDegrees{OX: 1, Theta: 0}

	// Approach from -X direction.
	approachPose := spatialmath.NewPose(
		r3.Vector{X: corePoint.X - 100, Y: corePoint.Y, Z: corePoint.Z},
		approachOrientation,
	)
	if err := r.moveFree(ctx, "xarm6", approachPose, nil); err != nil {
		return fmt.Errorf("approach core: %w", err)
	}

	if err := r.appleGripper.Open(ctx, nil); err != nil {
		return fmt.Errorf("open for core: %w", err)
	}

	corePose := spatialmath.NewPose(corePoint, approachOrientation)
	if err := r.moveLinear(ctx, "xarm6", corePose, nil); err != nil {
		return fmt.Errorf("reach core: %w", err)
	}

	if _, err := r.appleGripper.Grab(ctx, nil); err != nil {
		return fmt.Errorf("grab core: %w", err)
	}

	r.logger.Info("Core grabbed")
	return nil
}

// pressReleaseLever uses the secondary arm to press the release lever.
// The four linear moves (descent, press, release, retract) are each planned
// once via DoPlan and cached for immediate replay on subsequent calls.
func pressReleaseLever(ctx context.Context, r *Robot) error {
	if SecondaryReleaseLeverApproach == nil {
		r.logger.Warn("SecondaryReleaseLeverApproach not configured; skipping lever press")
		return nil
	}

	aboveApproach := poseAbove(SecondaryReleaseLeverApproach, 200)

	r.logger.Info("Moving above lever approach")
	if err := r.moveFree(ctx, "secondary-arm", aboveApproach, nil); err != nil {
		return fmt.Errorf("move above lever approach: %w", err)
	}

	if err := r.cachedLinearMove(ctx, "secondary-arm", SecondaryReleaseLeverApproach,
		&r.leverDescentTrajectory, "lever_descent.json"); err != nil {
		return fmt.Errorf("descend to lever: %w", err)
	}

	if SecondaryReleaseLeverPressPose == nil {
		r.logger.Warn("SecondaryReleaseLeverPressPose not configured (stub); lever press simulated")
		if err := r.cachedLinearMove(ctx, "secondary-arm", aboveApproach,
			&r.leverRetractTrajectory, "lever_retract.json"); err != nil {
			r.logger.Warnf("retract from lever approach: %v", err)
		}
		return nil
	}

	r.logger.Info("Pressing release lever")
	if err := r.cachedLinearMove(ctx, "secondary-arm", SecondaryReleaseLeverPressPose,
		&r.leverPressTrajectory, "lever_press.json"); err != nil {
		return fmt.Errorf("press lever: %w", err)
	}

	if err := retractSpikes(ctx, r); err != nil {
		r.logger.Warnf("Spike retraction: %v", err)
	}

	if err := r.cachedLinearMove(ctx, "secondary-arm", SecondaryReleaseLeverApproach,
		&r.leverReleaseTrajectory, "lever_release.json"); err != nil {
		return fmt.Errorf("release lever: %w", err)
	}

	if err := r.cachedLinearMove(ctx, "secondary-arm", aboveApproach,
		&r.leverRetractTrajectory, "lever_retract.json"); err != nil {
		return fmt.Errorf("retract from lever: %w", err)
	}

	r.logger.Info("Release lever pressed")
	return nil
}

// spikeRetractMaxVelDegsPerSec is the maximum joint velocity for the spike
// retraction move. The spikes are embedded in an apple core, so we pull slowly.
const spikeRetractMaxVelDegsPerSec = 15.0

// retractSpikes retracts the peeling arm back to PeelingCrankGraspJoints using
// a cached trajectory loaded from crank_retract.json. The trajectory is sent
// directly to the arm via MoveThroughJointPositions with a velocity limit,
// bypassing the motion service's DoExecute (which doesn't support MoveOptions).
func retractSpikes(ctx context.Context, r *Robot) error {
	if r.peelingArm == nil {
		r.logger.Warn("Peeling arm not available (stub); skipping spike retraction")
		return nil
	}
	if PeelingCrankGraspJoints == nil {
		r.logger.Warn("PeelingCrankGraspJoints not recorded; skipping spike retraction")
		return nil
	}

	steps, err := loadRetractSteps(r)
	if err != nil {
		return fmt.Errorf("load retract trajectory: %w", err)
	}

	vel := utils.DegToRad(spikeRetractMaxVelDegsPerSec)
	opts := &arm.MoveOptions{MaxVelRads: vel}

	r.logger.Infof("Retracting spikes (%d steps, max %.1f deg/s)", len(steps), spikeRetractMaxVelDegsPerSec)
	if err := r.peelingArm.MoveThroughJointPositions(ctx, steps, opts, nil); err != nil {
		return fmt.Errorf("execute spike retract: %w", err)
	}
	r.logger.Info("Spikes retracted")
	return nil
}

// loadRetractSteps loads the crank_retract.json trajectory and extracts the
// peeling-arm joint positions as a slice of input steps.
func loadRetractSteps(r *Robot) ([][]referenceframe.Input, error) {
	if r.PlansDir == "" {
		return nil, fmt.Errorf("PlansDir not set; cannot load crank_retract.json")
	}
	path := filepath.Join(r.PlansDir, "crank_retract.json")
	data, err := os.ReadFile(path)
	if err != nil {
		return nil, fmt.Errorf("read %s: %w", path, err)
	}
	var traj motionplan.Trajectory
	if err := json.Unmarshal(data, &traj); err != nil {
		return nil, fmt.Errorf("parse %s: %w", path, err)
	}

	steps := make([][]referenceframe.Input, 0, len(traj))
	for i, step := range traj {
		joints, ok := step["peeling-arm"]
		if !ok || len(joints) == 0 {
			return nil, fmt.Errorf("step %d missing peeling-arm joints", i)
		}
		steps = append(steps, joints)
	}
	r.logger.Infof("Loaded %d retract steps from %s", len(steps), path)
	return steps, nil
}

// returnSecondaryArm returns the secondary arm to its viewing position.
func returnSecondaryArm(ctx context.Context, r *Robot) error {
	if SecondaryViewingJoints == nil {
		r.logger.Warn("SecondaryViewingJoints not recorded; secondary arm stays in place")
		return nil
	}

	r.logger.Info("Returning secondary arm to viewing position")
	return r.moveToJoints(ctx, "secondary-arm", SecondaryViewingJoints)
}

// depositCoreAndReturn moves the primary arm to the waste bin, drops the core,
// and returns to the viewing position.
func depositCoreAndReturn(ctx context.Context, r *Robot) error {
	if WasteBinPose != nil {
		r.logger.Info("Depositing core in waste bin")
		aboveBin := poseAbove(WasteBinPose, 100)
		if err := r.moveFree(ctx, "xarm6", aboveBin, nil); err != nil {
			return fmt.Errorf("move to waste bin: %w", err)
		}

		if err := r.appleGripper.Open(ctx, nil); err != nil {
			return fmt.Errorf("release core: %w", err)
		}
	} else {
		r.logger.Warn("WasteBinPose not configured (stub); opening gripper in place")
		if err := r.appleGripper.Open(ctx, nil); err != nil {
			r.logger.Warnf("Open gripper: %v", err)
		}
	}

	// Return primary arm to viewing position.
	r.logger.Info("Returning primary arm to viewing position")
	return r.moveToJoints(ctx, "xarm6", PrimaryViewingJoints)
}
