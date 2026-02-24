package applesauce

import (
	"context"
	"fmt"
)

// Run executes the main peeling loop: watch → grasp → identify → peel → crank → remove → reset.
func Run(ctx context.Context, r *Robot) error {
	r.logger.Info("Starting peeling loop")

	for {
		select {
		case <-ctx.Done():
			r.logger.Info("Shutting down")
			return nil
		default:
		}

		if err := runCycle(ctx, r); err != nil {
			r.logger.Errorf("Cycle failed: %v", err)
			r.logger.Info("Retrying full cycle...")
			continue
		}

		r.state.ApplesProcessed++
		r.logger.Infof("Apple %d processed successfully", r.state.ApplesProcessed)
	}
}

// runCycle executes a single watch-to-reset peeling cycle.
func runCycle(ctx context.Context, r *Robot) error {
	r.resetState()

	steps := []struct {
		name string
		fn   func(context.Context, *Robot) error
	}{
		{"Watch", Watch},
		{"Grasp", Grasp},
		{"IdentifyFeatures", IdentifyFeatures},
		{"Peel", Peel},
		{"Crank", Crank},
		{"RemoveApple", RemoveApple},
		{"ResetMachine", ResetMachine},
	}

	for _, step := range steps {
		select {
		case <-ctx.Done():
			return ctx.Err()
		default:
		}

		r.logger.Infof("=== %s ===", step.name)
		if err := step.fn(ctx, r); err != nil {
			return fmt.Errorf("%s: %w", step.name, err)
		}
	}

	return nil
}
