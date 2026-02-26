package applesauce

import (
	"context"
	"time"
)

// Watch moves both arms to their viewing positions and polls the primary camera
// for a bowl of apples. Returns when a bowl is detected or the context is cancelled.
func Watch(ctx context.Context, r *Robot) error {
	// Move primary arm to viewing position.
	r.logger.Info("Moving primary arm to viewing position")
	if err := r.moveToJoints(ctx, "apple-arm", PrimaryViewingJoints); err != nil {
		return err
	}

	// Move secondary arm to viewing position (stub — position not yet recorded).
	if SecondaryViewingJoints != nil {
		r.logger.Info("Moving secondary arm to viewing position")
		if err := r.moveToJoints(ctx, "secondary-arm", SecondaryViewingJoints); err != nil {
			return err
		}
		if err := r.confirmSecondaryArmAt(ctx, SecondaryViewingJoints); err != nil {
			return err
		}
	} else {
		r.logger.Warn("SecondaryViewingJoints not recorded; skipping secondary arm positioning")
	}

	// Check that we have a camera to poll.
	if r.primaryCam == nil {
		r.logger.Warn("Primary camera not available; skipping bowl detection (stub)")
		return nil
	}

	// Poll camera every 3 seconds until a bowl is detected.
	r.logger.Info("Watching for bowl of apples...")
	ticker := time.NewTicker(3 * time.Second)
	defer ticker.Stop()

	for {
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-ticker.C:
		}

		cloud, err := r.primaryCam.NextPointCloud(ctx, nil)
		if err != nil {
			r.logger.Warnf("Camera error: %v", err)
			continue
		}

		result, err := r.detector.Detect(ctx, cloud)
		if err != nil {
			r.logger.Debugf("Detection: %v", err)
			continue
		}

		if result.BowlDetected {
			r.logger.Infof("Bowl detected with %d apples", len(result.Bowl.Apples))
			r.state.LastDetection = result
			return nil
		}

		r.logger.Debug("No bowl detected, retrying...")
	}
}
