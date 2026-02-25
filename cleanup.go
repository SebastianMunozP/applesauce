package applesauce

import (
	"context"
	"fmt"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/spatialmath"
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
	if err := r.moveFree(ctx, "xarm7", approachPose, nil); err != nil {
		return fmt.Errorf("move to apple approach: %w", err)
	}

	// Open gripper.
	if err := r.appleGripper.Open(ctx, nil); err != nil {
		return fmt.Errorf("open gripper for apple removal: %w", err)
	}

	// Linear approach to the apple on the core.
	corePose := spatialmath.NewPose(corePoint, approachOrientation)
	if err := r.moveLinear(ctx, "xarm7", corePose, nil); err != nil {
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
	if err := r.moveLinear(ctx, "xarm7", pullPose, nil); err != nil {
		return fmt.Errorf("pull apple off core: %w", err)
	}

	// Move to peeled apple bowl and release.
	r.logger.Info("Depositing peeled apple")
	aboveBowl := poseAbove(PeeledAppleBowlPose, 100)
	if err := r.moveFree(ctx, "xarm7", aboveBowl, nil); err != nil {
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

	// Step 2: Secondary arm presses release lever.
	if err := pressReleaseLever(ctx, r); err != nil {
		r.logger.Warnf("Release lever: %v", err)
	}

	// Step 3: Secondary arm returns to viewing position.
	if SecondaryReleaseLeverApproach != nil {
		// Move above approach first.
		if err := returnSecondaryArm(ctx, r); err != nil {
			r.logger.Warnf("Secondary arm return: %v", err)
		}
	}

	// Step 4: Primary arm deposits core and returns to viewing position.
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
	if err := r.moveFree(ctx, "xarm7", approachPose, nil); err != nil {
		return fmt.Errorf("approach core: %w", err)
	}

	if err := r.appleGripper.Open(ctx, nil); err != nil {
		return fmt.Errorf("open for core: %w", err)
	}

	corePose := spatialmath.NewPose(corePoint, approachOrientation)
	if err := r.moveLinear(ctx, "xarm7", corePose, nil); err != nil {
		return fmt.Errorf("reach core: %w", err)
	}

	if _, err := r.appleGripper.Grab(ctx, nil); err != nil {
		return fmt.Errorf("grab core: %w", err)
	}

	r.logger.Info("Core grabbed")
	return nil
}

// pressReleaseLever uses the secondary arm to press the release lever.
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

	if err := r.moveLinear(ctx, "secondary-arm", SecondaryReleaseLeverApproach, nil); err != nil {
		return fmt.Errorf("move to lever approach: %w", err)
	}

	if SecondaryReleaseLeverPressPose == nil {
		r.logger.Warn("SecondaryReleaseLeverPressPose not configured (stub); lever press simulated")
		if err := r.moveLinear(ctx, "secondary-arm", aboveApproach, nil); err != nil {
			r.logger.Warnf("retract from lever approach: %v", err)
		}
		return nil
	}

	r.logger.Info("Pressing release lever")
	if err := r.moveLinear(ctx, "secondary-arm", SecondaryReleaseLeverPressPose, nil); err != nil {
		return fmt.Errorf("press lever: %w", err)
	}
	
	err := retractSpikes(ctx, r)
	if err != nil {
		return err
	}
	
	if err := r.moveLinear(ctx, "secondary-arm", SecondaryReleaseLeverApproach, nil); err != nil {
		return fmt.Errorf("move to lever approach: %w", err)
	}

	// Return to 200mm above approach.
	if err := r.moveLinear(ctx, "secondary-arm", aboveApproach, nil); err != nil {
		return fmt.Errorf("retract from lever: %w", err)
	}

	r.logger.Info("Release lever pressed")
	return nil
}

// retractSpikes uses the peeling arm to pull the spikes out of the core.
func retractSpikes(ctx context.Context, r *Robot) error {
	if r.peelingArm == nil {
		r.logger.Warn("Peeling arm not available (stub); skipping spike retraction")
		return nil
	}

	if PeelerSpikeRetractPose == nil {
		r.logger.Warn("PeelerSpikeRetractPose not configured (stub); skipping spike retraction")
		return nil
	}

	r.logger.Info("Retracting spikes from core")
	if err := r.moveFree(ctx, "peeling-arm", PeelerSpikeRetractPose, nil); err != nil {
		return fmt.Errorf("retract spikes: %w", err)
	}

	r.logger.Info("Spikes retracted")
	return nil
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
		if err := r.moveFree(ctx, "xarm7", aboveBin, nil); err != nil {
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
	return r.moveToJoints(ctx, "xarm7", PrimaryViewingJoints)
}
