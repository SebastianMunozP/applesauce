package main

import (
	"context"
	"flag"
	"fmt"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/biotinker/applesauce"
	applepose "github.com/biotinker/applesauce/apple_pose"
	"github.com/biotinker/applesauce/internal/creds"

	vizClient "github.com/viam-labs/motion-tools/client/client"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/rdk/spatialmath"
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
	flag.Parse()

	logger := logging.NewDebugLogger("applesauce-cli")

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

	// Visualize with motion-tools.
	cam := r.PrimaryCam()
	if cam == nil {
		logger.Warn("No primary camera; skipping visualization")
		return nil
	}

	cloud, err := cam.NextPointCloud(ctx, nil)
	if err != nil {
		logger.Warnf("Could not get point cloud for visualization: %v", err)
		return nil
	}

	visualize(logger, cloud, result)
	return nil
}

const vizDelay = 300 * time.Millisecond

func visualize(logger logging.Logger, cloud pointcloud.PointCloud, result *applepose.DetectionResult) {
	if err := vizClient.RemoveAllSpatialObjects(); err != nil {
		logger.Warnf("viz: could not clear scene (is motion-tools running?): %v", err)
		return
	}
	time.Sleep(vizDelay)

	if err := vizClient.DrawPointCloud("applecam", cloud, nil); err != nil {
		logger.Warnf("viz: could not draw pointcloud: %v", err)
		return
	}
	time.Sleep(vizDelay)
	logger.Infof("viz: drew pointcloud (%d points)", cloud.Size())

	for i, apple := range result.Bowl.Apples {
		center := apple.Pose.Point()

		sphere, err := spatialmath.NewSphere(
			spatialmath.NewPoseFromPoint(center),
			apple.Radius,
			fmt.Sprintf("apple_%d", i),
		)
		if err != nil {
			logger.Warnf("viz: failed to create sphere %d: %v", i, err)
			continue
		}
		if err := vizClient.DrawGeometry(sphere, "red"); err != nil {
			logger.Warnf("viz: could not draw sphere %d: %v", i, err)
			continue
		}
		time.Sleep(vizDelay)
		logger.Infof("viz: drew apple %d (radius=%.1fmm) at (%.1f, %.1f, %.1f)",
			i, apple.Radius, center.X, center.Y, center.Z)

		for j, f := range apple.Features {
			fPos := f.Pose.Point()
			color := "white"
			switch f.Feature {
			case applepose.FeatureStem:
				color = "black"
			case applepose.FeatureCalyx:
				color = "green"
			}
			fSphere, err := spatialmath.NewSphere(
				spatialmath.NewPoseFromPoint(fPos),
				5.0,
				fmt.Sprintf("apple_%d_feature_%d", i, j),
			)
			if err != nil {
				continue
			}
			if err := vizClient.DrawGeometry(fSphere, color); err != nil {
				continue
			}
			time.Sleep(vizDelay)
		}
	}

	logger.Info("viz: visualization complete")
}
