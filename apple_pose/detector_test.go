package applepose

import (
	"context"
	"fmt"
	"os"
	"testing"
	"time"

	vizClient "github.com/viam-labs/motion-tools/client/client"

	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/spatialmath"
)

const testPCDPath = "testdata/applecam.pcd"

func loadTestCloud(t *testing.T) pointcloud.PointCloud {
	t.Helper()
	if _, err := os.Stat(testPCDPath); os.IsNotExist(err) {
		t.Skipf("test PCD not found at %s (run cmd/test_apples to capture)", testPCDPath)
	}
	cloud, err := pointcloud.NewFromFile(testPCDPath, "")
	if err != nil {
		t.Fatalf("failed to load PCD: %v", err)
	}
	return cloud
}

// TestDetect_RealData runs the full pipeline on a captured PCD from the applecam.
// Ground truth for testdata/applecam.pcd (5 apples in a blue bowl):
//
//	Apple 0: front-most apple, center ≈ (-74, 42, 431), radius ≈ 42mm
//	Apple 1: left apple, center ≈ (-139, -25, 475), radius ≈ 39mm
//	Apple 2: center apple, center ≈ (-69, -11, 482), radius ≈ 42mm  ← best fit, nearly exact
//	Apple 3: right apple, center ≈ (-13, 45, 479), radius ≈ 40mm
//	Apple 4: rear apple, center ≈ (-96, -72, 518), radius ≈ 42mm
//
// Expected radii are ~41mm (diameter ~82mm). True radii should be similar
// across all 5. Visibility ranges from ~20% (rear) to ~43% (front).
func TestDetect_RealData(t *testing.T) {
	cloud := loadTestCloud(t)
	t.Logf("loaded %d points, hasColor=%v", cloud.Size(), cloud.MetaData().HasColor)

	meta := cloud.MetaData()
	t.Logf("bounds: X[%.1f, %.1f] Y[%.1f, %.1f] Z[%.1f, %.1f]",
		meta.MinX, meta.MaxX, meta.MinY, meta.MaxY, meta.MinZ, meta.MaxZ)

	cfg := DefaultConfig()
	detector := NewDetector(&cfg)

	result, err := detector.Detect(context.Background(), cloud)
	if err != nil {
		t.Fatalf("Detect failed: %v", err)
	}

	t.Logf("bowl detected: %v", result.BowlDetected)
	t.Logf("apples found: %d", len(result.Bowl.Apples))

	if len(result.Bowl.Apples) == 0 {
		t.Fatal("expected at least one apple")
	}

	for i, apple := range result.Bowl.Apples {
		pos := apple.Pose.Point()
		t.Logf("apple %d (%s): center=(%.1f, %.1f, %.1f) radius=%.1f inliers=%.2f visible=%.2f features=%d points=%d",
			i, apple.ID, pos.X, pos.Y, pos.Z,
			apple.Radius, apple.InlierFraction, apple.VisibleFraction,
			len(apple.Features), apple.Points.Size())

		for _, f := range apple.Features {
			fPos := f.Pose.Point()
			t.Logf("  %s: confidence=%.2f pos=(%.1f, %.1f, %.1f)",
				f.Feature, f.Confidence, fPos.X, fPos.Y, fPos.Z)
		}
	}

	t.Logf("suggested views: %d", len(result.SuggestedViews))
	t.Logf("suggested actions: %d", len(result.SuggestedActions))
	if result.Bowl.UnclassifiedPoints != nil {
		t.Logf("unclassified points: %d", result.Bowl.UnclassifiedPoints.Size())
	}

	// Visualize results in motion-tools.
	visualizeResults(t, cloud, result)
}

func TestDetectWithHistory_RealData(t *testing.T) {
	cloud := loadTestCloud(t)

	cfg := DefaultConfig()
	detector := NewDetector(&cfg)

	// First detection.
	result1, err := detector.Detect(context.Background(), cloud)
	if err != nil {
		t.Fatalf("first Detect failed: %v", err)
	}
	t.Logf("first pass: %d apples", len(result1.Bowl.Apples))

	// Second detection with history (same cloud simulates a second view).
	result2, err := detector.DetectWithHistory(context.Background(), cloud, result1)
	if err != nil {
		t.Fatalf("DetectWithHistory failed: %v", err)
	}
	t.Logf("second pass: %d apples", len(result2.Bowl.Apples))

	// Matched apples should preserve IDs from the first pass.
	for _, apple := range result2.Bowl.Apples {
		t.Logf("apple %s: visible=%.2f features=%d", apple.ID, apple.VisibleFraction, len(apple.Features))
	}
}

func TestAnalyzeSingleApple_RealData(t *testing.T) {
	cloud := loadTestCloud(t)

	cfg := DefaultConfig()
	detector := NewDetector(&cfg)

	// Run full detection first to find apple centers.
	result, err := detector.Detect(context.Background(), cloud)
	if err != nil {
		t.Fatalf("Detect failed: %v", err)
	}
	if len(result.Bowl.Apples) == 0 {
		t.Skip("no apples found to analyze individually")
	}

	// Analyze the first apple individually.
	apple := result.Bowl.Apples[0]
	single, err := detector.AnalyzeSingleApple(
		context.Background(),
		apple.Points,
		apple.Pose.Point(),
		apple.Radius,
	)
	if err != nil {
		t.Fatalf("AnalyzeSingleApple failed: %v", err)
	}

	pos := single.Pose.Point()
	t.Logf("single apple: center=(%.1f, %.1f, %.1f) radius=%.1f features=%d",
		pos.X, pos.Y, pos.Z, single.Radius, len(single.Features))

	fmt.Printf("Single apple analysis: radius=%.1fmm, %d features, visible=%.0f%%\n",
		single.Radius, len(single.Features), single.VisibleFraction*100)
}

const vizDelay = 300 * time.Millisecond

// visualizeResults draws the detection results in the motion-tools visualizer.
func visualizeResults(t *testing.T, cloud pointcloud.PointCloud, result *DetectionResult) {
	t.Helper()

	// Clear the scene.
	if err := vizClient.RemoveAllSpatialObjects(); err != nil {
		t.Logf("viz: could not clear scene (is motion-tools running?): %v", err)
		return
	}
	time.Sleep(vizDelay)
	t.Log("viz: scene cleared")

	// Draw the full pointcloud.
	if err := vizClient.DrawPointCloud("applecam", cloud, nil); err != nil {
		t.Logf("viz: could not draw pointcloud: %v", err)
		return
	}
	time.Sleep(vizDelay)
	t.Logf("viz: drew pointcloud (%d points)", cloud.Size())

	// Draw each apple.
	for i, apple := range result.Bowl.Apples {
		center := apple.Pose.Point()

		// Red sphere at detected radius for the apple body.
		appleSphere, err := spatialmath.NewSphere(
			spatialmath.NewPoseFromPoint(center),
			apple.Radius,
			fmt.Sprintf("apple_%d", i),
		)
		if err != nil {
			t.Logf("viz: failed to create apple sphere %d: %v", i, err)
			continue
		}
		if err := vizClient.DrawGeometry(appleSphere, "red"); err != nil {
			t.Logf("viz: could not draw apple sphere %d: %v", i, err)
			continue
		}
		time.Sleep(vizDelay)
		t.Logf("viz: drew apple %d sphere (radius=%.1fmm) at (%.1f, %.1f, %.1f)",
			i, apple.Radius, center.X, center.Y, center.Z)

		// Draw features.
		for _, f := range apple.Features {
			fPos := f.Pose.Point()
			var color string
			switch f.Feature {
			case FeatureStem:
				color = "black"
			case FeatureCalyx:
				color = "green"
			}

			featureSphere, err := spatialmath.NewSphere(
				spatialmath.NewPoseFromPoint(fPos),
				5.0, // 5mm radius for feature markers
				fmt.Sprintf("apple_%d_%s", i, f.Feature),
			)
			if err != nil {
				t.Logf("viz: failed to create %s sphere: %v", f.Feature, err)
				continue
			}
			if err := vizClient.DrawGeometry(featureSphere, color); err != nil {
				t.Logf("viz: could not draw %s sphere: %v", f.Feature, err)
				continue
			}
			time.Sleep(vizDelay)
			t.Logf("viz: drew %s (%s, confidence=%.2f) at (%.1f, %.1f, %.1f)",
				f.Feature, color, f.Confidence, fPos.X, fPos.Y, fPos.Z)
		}
	}

	t.Log("viz: visualization complete")
}
