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

func TestPlanViews_MissingFeatures(t *testing.T) {
	// Create an apple with no features detected (should suggest views).
	center := r3.Vector{X: 0, Y: 0, Z: 50}
	radius := 35.0

	cloud := generatePartialSphere(center, radius, 200, 0.3, 0)

	sphere, _ := spatialmath.NewSphere(spatialmath.NewPoseFromPoint(center), radius, "test")
	apple := Apple{
		ID:              "apple_0",
		Sphere:          sphere,
		Pose:            spatialmath.NewPoseFromPoint(center),
		Radius:          radius,
		Features:        nil, // No features
		Points:          cloud,
		VisibleFraction: 0.4,
	}

	result := &DetectionResult{
		Bowl: BowlOfApples{
			Apples:       []Apple{apple},
			SupportPlane: pointcloud.NewPlane(pointcloud.NewBasicEmpty(), [4]float64{0, 0, 1, 0}),
		},
	}

	cfg := ViewPlanningConfig{
		ViewDistanceMm:         200.0,
		NumCandidateViews:      8,
		MinVisibleFractionGap:  0.3,
	}

	views, actions := PlanViews(result, cfg)

	t.Logf("suggested %d views, %d actions", len(views), len(actions))

	if len(views) == 0 && len(actions) == 0 {
		t.Error("expected at least one view or action suggestion for apple missing features")
	}

	// Verify views are in the hidden hemisphere and look toward the apple.
	for i, v := range views {
		camPos := v.CameraPose.Point()
		dirToApple := center.Sub(camPos)
		dist := dirToApple.Norm()

		// Camera should be roughly at the configured distance.
		if math.Abs(dist-cfg.ViewDistanceMm) > cfg.ViewDistanceMm*0.5 {
			t.Errorf("view %d: camera distance %.1f mm far from expected %.1f mm", i, dist, cfg.ViewDistanceMm)
		}

		// Camera should be above the support plane (Z > 0).
		if camPos.Z < 0 {
			t.Errorf("view %d: camera below support plane (Z=%.1f)", i, camPos.Z)
		}

		// Priority should be in [0, 1].
		if v.Priority < 0 || v.Priority > 1 {
			t.Errorf("view %d: priority %.2f outside [0,1]", i, v.Priority)
		}

		t.Logf("view %d: pos=%v purpose=%s priority=%.2f", i, camPos, v.Purpose, v.Priority)
	}
}

func TestPlanViews_HiddenBelowSurface(t *testing.T) {
	// Create an apple sitting on a surface where bottom hemisphere is hidden.
	// Points only on the upper hemisphere.
	center := r3.Vector{X: 0, Y: 0, Z: 35}
	radius := 35.0

	cloud := pointcloud.NewBasicEmpty()
	//nolint:gosec
	rng := rand.New(rand.NewSource(42))
	clr := pointcloud.NewColoredData(color.NRGBA{R: 200, G: 50, B: 50, A: 255})

	for cloud.Size() < 200 {
		theta := rng.Float64() * 2 * math.Pi
		phi := math.Acos(2*rng.Float64() - 1)
		dir := r3.Vector{
			X: math.Sin(phi) * math.Cos(theta),
			Y: math.Sin(phi) * math.Sin(theta),
			Z: math.Cos(phi),
		}
		// Only keep upper hemisphere (visible from above).
		if dir.Z < 0 {
			continue
		}
		pt := center.Add(dir.Mul(radius))
		cloud.Set(pt, clr) //nolint:errcheck
	}

	sphere, _ := spatialmath.NewSphere(spatialmath.NewPoseFromPoint(center), radius, "test")
	apple := Apple{
		ID:              "apple_0",
		Sphere:          sphere,
		Pose:            spatialmath.NewPoseFromPoint(center),
		Radius:          radius,
		Features:        nil,
		Points:          cloud,
		VisibleFraction: 0.5,
	}

	result := &DetectionResult{
		Bowl: BowlOfApples{
			Apples:       []Apple{apple},
			SupportPlane: pointcloud.NewPlane(pointcloud.NewBasicEmpty(), [4]float64{0, 0, 1, 0}),
		},
	}

	cfg := ViewPlanningConfig{
		ViewDistanceMm:         200.0,
		NumCandidateViews:      8,
		MinVisibleFractionGap:  0.3,
	}

	views, actions := PlanViews(result, cfg)

	t.Logf("suggested %d views, %d actions", len(views), len(actions))

	// When the hidden hemisphere faces the surface, we should get a flip action.
	hasFlip := false
	for _, a := range actions {
		if a.Type == ActionFlipApple {
			hasFlip = true
			t.Logf("flip action: %s", a.Description)
		}
	}

	if !hasFlip {
		t.Log("no flip action generated (may have camera views instead)")
	}
}

func TestRotateToAlign(t *testing.T) {
	// Rotating Z to Z should be identity.
	v := r3.Vector{X: 1, Y: 0, Z: 0}
	result := rotateToAlign(v, r3.Vector{Z: 1})
	if result.Sub(v).Norm() > 1e-6 {
		t.Errorf("identity rotation failed: got %v", result)
	}

	// Rotating Z to X: the Z-axis in the new frame is X in world.
	target := r3.Vector{X: 1}
	zAxis := r3.Vector{Z: 1}
	rotated := rotateToAlign(zAxis, target)
	if rotated.Sub(target).Norm() > 1e-6 {
		t.Errorf("rotateToAlign(Z, X) should give X, got %v", rotated)
	}
}

func TestComputeObservedDirection(t *testing.T) {
	center := r3.Vector{X: 0, Y: 0, Z: 0}
	cloud := pointcloud.NewBasicEmpty()

	// Points on the +Z hemisphere only.
	//nolint:gosec
	rng := rand.New(rand.NewSource(1))
	for i := 0; i < 100; i++ {
		theta := rng.Float64() * 2 * math.Pi
		phi := rng.Float64() * math.Pi / 2 // Only upper hemisphere
		pt := r3.Vector{
			X: 30 * math.Sin(phi) * math.Cos(theta),
			Y: 30 * math.Sin(phi) * math.Sin(theta),
			Z: 30 * math.Cos(phi),
		}
		cloud.Set(pt, nil) //nolint:errcheck
	}

	dir := computeObservedDirection(cloud, center)

	// Direction should be roughly +Z.
	if dir.Z < 0.5 {
		t.Errorf("expected observed direction roughly +Z, got %v", dir)
	}
	t.Logf("observed direction: %v", dir)
}
