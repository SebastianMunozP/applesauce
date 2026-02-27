package applesauce

import (
	"context"
	"fmt"
	"time"

	"go.viam.com/rdk/motionplan"
)

// PeelHardcoded moves the grasped apple through hardcoded poses to place it on the peeler.
// Assumes the apple is already grasped and lifted.
//
// Moves the applegripper through three poses defined in the frame system:
// 1. pose-above: Position above the peeler
// 2. pose-align: Aligned with the peeler spikes
// 3. pose-pierce: Pushed onto the peeler spikes
func PeelHardcoded(ctx context.Context, r *Robot) error {
	r.logger.Info("Starting hardcoded peel sequence")

	// Move to pose-above.
	r.logger.Infof("Moving to pose-above: %v", PeelAbovePose)
	if err := r.moveFree(ctx, r.primaryArm.Name().Name, PeelAbovePose, nil); err != nil {
		return fmt.Errorf("move to pose-above: %w", err)
	}

	// Sleep
	r.logger.Info("Sleeping for 1 second")
	time.Sleep(1 * time.Second)

	// Move to pose-align.
	r.logger.Infof("Moving to pose-align: %v", PeelAlignPose)
	if err := r.moveLinear(ctx, r.primaryArm.Name().Name, PeelAlignPose, nil, 1); err != nil {
		return fmt.Errorf("move to pose-align: %w", err)
	}

	// Sleep
	r.logger.Info("Sleeping for 1 second")
	time.Sleep(1 * time.Second)

	armPeelerCrankCollision := motionplan.CollisionSpecification{
		Allows: []motionplan.CollisionSpecificationAllowedFrameCollisions{
			{Frame1: "applegripper", Frame2: "peeler-crank"},
		},
	}

	// Increase arm speed before pierce
	r.logger.Infof("Increasing arm speed before pierce")

	if _, err := r.primaryArm.DoCommand(ctx, map[string]interface{}{"set_speed": 30}); err != nil {
		return fmt.Errorf("failed to increase gripper and acceleration speed: %w", err)
	}

	// Move to pose-pierce (push apple onto spikes).
	r.logger.Infof("Moving to pose-pierce (pushing onto spikes): %v", PeelPiercePose)
	if err := r.moveLinear(ctx, r.primaryArm.Name().Name, PeelPiercePose, nil, 1, armPeelerCrankCollision); err != nil {
		return fmt.Errorf("move to pose-pierce: %w", err)
	}

	// Decrease arm speed after pierce
	r.logger.Infof("Decreasing arm speed after pierce")
	if _, err := r.primaryArm.DoCommand(ctx, map[string]interface{}{"set_speed": 20}); err != nil {
		return fmt.Errorf("failed to decrease gripper and acceleration speed: %w", err)
	}

	// Sleep
	r.logger.Info("Sleeping for 1 second")
	time.Sleep(1 * time.Second)

	// Open gripper to release apple onto spikes.
	r.logger.Info("Opening gripper to release apple")
	if err := r.appleGripper.Open(ctx, nil); err != nil {
		return fmt.Errorf("open gripper: %w", err)
	}

	time.Sleep(1 * time.Second) // wait for gripper to open

	// Retreat back to pose-above.
	r.logger.Infof("Retreating to pose-above: %v", PeelAbovePose)
	if err := r.moveLinear(ctx, r.primaryArm.Name().Name, PeelAbovePose, nil, 1); err != nil {
		return fmt.Errorf("retreat to pose-above: %w", err)
	}

	r.logger.Info("Hardcoded peel sequence complete")
	return nil
}
