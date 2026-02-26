package main

import (
	"context"
	"flag"
	"fmt"
	"os"
	"os/signal"
	"syscall"

	"github.com/biotinker/applesauce"
	"github.com/biotinker/applesauce/internal/creds"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/utils/rpc"
)

var steps = map[string]func(context.Context, *applesauce.Robot) error{
	"grasp":    applesauce.Grasp,
	"identify": applesauce.IdentifyFeatures,
	"peel":     applesauce.Peel,
	"crank":    applesauce.Crank,
	"remove":   applesauce.RemoveApple,
	"reset":    applesauce.ResetMachine,
	"retract":  applesauce.Retract,
}

const validSteps = "watch, grasp, identify, peel, crank, remove, reset, retract"

func main() {
	credsPath := flag.String("creds", "", "path to robot credentials JSON file")
	step := flag.String("step", "", "step to run: "+validSteps)
	plansDir := flag.String("plans-dir", "", "directory for cached crank trajectory plans (optional)")
	flag.Parse()

	logger := logging.NewLogger("applesauce-cli")

	if *credsPath == "" {
		logger.Fatal("-creds flag is required")
	}
	if *step == "" {
		logger.Fatal("-step flag is required; valid steps: " + validSteps)
	}

	// Validate step name.
	if *step != "watch" {
		if _, ok := steps[*step]; !ok {
			logger.Fatalf("unknown step %q; valid steps: %s", *step, validSteps)
		}
	}

	robotCreds, err := creds.Load(*credsPath)
	if err != nil {
		logger.Fatal(err)
	}

	ctx, cancel := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer cancel()

	machine, err := client.New(
		ctx,
		robotCreds.Address,
		logger,
		client.WithDialOptions(rpc.WithEntityCredentials(
			robotCreds.EntityID,
			rpc.Credentials{
				Type:    rpc.CredentialsTypeAPIKey,
				Payload: robotCreds.APIKey,
			})),
	)
	if err != nil {
		logger.Fatal(err)
	}
	defer machine.Close(context.Background())

	logger.Info("Connected to robot")

	r, err := applesauce.NewRobot(ctx, machine, logger)
	if err != nil {
		logger.Fatal(err)
	}
	if *plansDir != "" {
		r.PlansDir = *plansDir
	}

	logger.Infof("=== Running step: %s ===", *step)

	if *step == "watch" {
		if err := runWatch(ctx, r, logger); err != nil {
			logger.Fatal(err)
		}
		return
	}

	if err := steps[*step](ctx, r); err != nil {
		logger.Fatal(err)
	}
	logger.Infof("Step %s completed successfully", *step)
}

func runWatch(ctx context.Context, r *applesauce.Robot, logger logging.Logger) error {
	if err := applesauce.Watch(ctx, r); err != nil {
		return fmt.Errorf("watch: %w", err)
	}

	result := r.LastDetection()
	if result == nil {
		logger.Info("No detection result available")
		return nil
	}

	// Print detection summary.
	logger.Infof("Bowl detected: %v", result.BowlDetected)
	logger.Infof("Apples found: %d", len(result.Bowl.Apples))

	for i, apple := range result.Bowl.Apples {
		pos := apple.Pose.Point()
		logger.Infof("  Apple %d (%s): center=(%.1f, %.1f, %.1f) radius=%.1fmm visible=%.0f%%",
			i, apple.ID, pos.X, pos.Y, pos.Z, apple.Radius, apple.VisibleFraction*100)
		for _, f := range apple.Features {
			fPos := f.Pose.Point()
			logger.Infof("    %v: confidence=%.2f pos=(%.1f, %.1f, %.1f)",
				f.Feature, f.Confidence, fPos.X, fPos.Y, fPos.Z)
		}
	}

	return nil
}
