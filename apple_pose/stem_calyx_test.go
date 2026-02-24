package applepose

import (
	"image/color"
	"math"
	"math/rand"
	"testing"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/spatialmath"
)

func TestDetectStemCalyx_SyntheticApple(t *testing.T) {
	// Create a synthetic apple: sphere with a small indentation at the top (stem)
	// and a small indentation at the bottom (calyx).
	center := r3.Vector{X: 0, Y: 0, Z: 50}
	radius := 35.0
	supportNormal := r3.Vector{X: 0, Y: 0, Z: 1}

	cloud := pointcloud.NewBasicEmpty()
	//nolint:gosec
	rng := rand.New(rand.NewSource(42))
	appleSkin := color.NRGBA{R: 200, G: 30, B: 30, A: 255}
	stemColor := color.NRGBA{R: 100, G: 80, B: 40, A: 255} // Brownish

	for i := 0; i < 800; i++ {
		theta := rng.Float64() * 2 * math.Pi
		phi := math.Acos(2*rng.Float64() - 1)

		dir := r3.Vector{
			X: math.Sin(phi) * math.Cos(theta),
			Y: math.Sin(phi) * math.Sin(theta),
			Z: math.Cos(phi),
		}

		r := radius
		clr := appleSkin

		// Top pole (stem): indent and color it stem-like.
		if dir.Z > 0.9 {
			r = radius - 4.0 // Indentation
			clr = stemColor
		}

		// Bottom pole (calyx): indent slightly.
		if dir.Z < -0.9 {
			r = radius - 3.0
			clr = stemColor
		}

		pt := center.Add(dir.Mul(r))
		//nolint:errcheck
		cloud.Set(pt, pointcloud.NewColoredData(clr))
	}

	cfg := FeatureConfig{
		CurvatureNeighbors:    10,
		DeviationThresholdMm:  2.0,
		AnomalyClusterRadius:  8.0,
		MinAnomalyPoints:      3,
		StemHueMin:            20.0,
		StemHueMax:            90.0,
		StemMaxSaturation:     0.8,
	}

	features := detectStemCalyx(cloud, center, radius, supportNormal, cfg)

	t.Logf("detected %d features", len(features))
	for _, f := range features {
		t.Logf("  %s: confidence=%.2f, pos=%v", f.Feature, f.Confidence, f.Pose.Point())
	}

	if len(features) == 0 {
		t.Error("expected at least one feature detected")
	}

	// Check that if we got a stem, it's above center.
	for _, f := range features {
		if f.Feature == FeatureStem {
			if f.Pose.Point().Z < center.Z {
				t.Error("stem should be above apple center")
			}
		}
	}
}

func TestDetectStemCalyx_NoFeatures(t *testing.T) {
	// A perfect sphere should have no features.
	center := r3.Vector{X: 0, Y: 0, Z: 50}
	radius := 35.0
	supportNormal := r3.Vector{X: 0, Y: 0, Z: 1}

	cloud := pointcloud.NewBasicEmpty()
	//nolint:gosec
	rng := rand.New(rand.NewSource(99))
	appleSkin := color.NRGBA{R: 200, G: 30, B: 30, A: 255}

	for i := 0; i < 500; i++ {
		theta := rng.Float64() * 2 * math.Pi
		phi := math.Acos(2*rng.Float64() - 1)
		pt := r3.Vector{
			X: center.X + radius*math.Sin(phi)*math.Cos(theta),
			Y: center.Y + radius*math.Sin(phi)*math.Sin(theta),
			Z: center.Z + radius*math.Cos(phi),
		}
		//nolint:errcheck
		cloud.Set(pt, pointcloud.NewColoredData(appleSkin))
	}

	cfg := FeatureConfig{
		CurvatureNeighbors:    10,
		DeviationThresholdMm:  2.0,
		AnomalyClusterRadius:  5.0,
		MinAnomalyPoints:      5,
		StemHueMin:            20.0,
		StemHueMax:            90.0,
		StemMaxSaturation:     0.8,
	}

	features := detectStemCalyx(cloud, center, radius, supportNormal, cfg)

	// A perfect sphere might detect a small number of spurious features due to PCA noise,
	// but they should have low confidence.
	for _, f := range features {
		if f.Confidence > 0.5 {
			t.Errorf("unexpected high-confidence feature on perfect sphere: %s (%.2f)", f.Feature, f.Confidence)
		}
	}

	t.Logf("detected %d features on perfect sphere", len(features))
}

func TestComputeAppleOrientation(t *testing.T) {
	center := r3.Vector{X: 0, Y: 0, Z: 50}
	stemPos := r3.Vector{X: 0, Y: 0, Z: 85}
	calyxPos := r3.Vector{X: 0, Y: 0, Z: 15}

	stemCloud := pointcloud.NewBasicEmpty()
	stemCloud.Set(stemPos, nil) //nolint:errcheck
	calyxCloud := pointcloud.NewBasicEmpty()
	calyxCloud.Set(calyxPos, nil) //nolint:errcheck

	features := []FeatureDetection{
		{
			Feature:    FeatureStem,
			Pose:       spatialmath.NewPoseFromPoint(stemPos),
			Confidence: 0.9,
			Points:     stemCloud,
		},
		{
			Feature:    FeatureCalyx,
			Pose:       spatialmath.NewPoseFromPoint(calyxPos),
			Confidence: 0.8,
			Points:     calyxCloud,
		},
	}

	pose := computeAppleOrientation(center, features)

	// The orientation should point roughly along Z (calyx-to-stem = +Z).
	ov := pose.Orientation().OrientationVectorRadians()
	if ov.OZ < 0.9 {
		t.Errorf("expected orientation along +Z, got OV=(%f, %f, %f)", ov.OX, ov.OY, ov.OZ)
	}

	t.Logf("apple orientation: OV=(%f, %f, %f)", ov.OX, ov.OY, ov.OZ)
}

func TestRGBToHSV(t *testing.T) {
	tests := []struct {
		r, g, b    uint8
		wantH      float64
		wantS      float64
		wantV      float64
		tolerance  float64
	}{
		{255, 0, 0, 0, 1.0, 1.0, 1.0},     // Pure red
		{0, 255, 0, 120, 1.0, 1.0, 1.0},   // Pure green
		{0, 0, 255, 240, 1.0, 1.0, 1.0},   // Pure blue
		{255, 255, 0, 60, 1.0, 1.0, 1.0},  // Yellow
		{128, 128, 128, 0, 0.0, 0.502, 2.0}, // Gray (any hue ok)
	}

	for _, tt := range tests {
		hsv := rgbToHSV(tt.r, tt.g, tt.b)
		if tt.tolerance < 2.0 {
			if math.Abs(hsv.H-tt.wantH) > tt.tolerance && math.Abs(hsv.H-tt.wantH-360) > tt.tolerance {
				t.Errorf("rgbToHSV(%d,%d,%d) H=%.1f, want %.1f", tt.r, tt.g, tt.b, hsv.H, tt.wantH)
			}
			if math.Abs(hsv.S-tt.wantS) > 0.01 {
				t.Errorf("rgbToHSV(%d,%d,%d) S=%.3f, want %.3f", tt.r, tt.g, tt.b, hsv.S, tt.wantS)
			}
		}
		if math.Abs(hsv.V-tt.wantV) > 0.01 && tt.tolerance < 2.0 {
			t.Errorf("rgbToHSV(%d,%d,%d) V=%.3f, want %.3f", tt.r, tt.g, tt.b, hsv.V, tt.wantV)
		}
	}
}
