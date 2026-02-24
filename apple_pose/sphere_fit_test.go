package applepose

import (
	"image/color"
	"math"
	"math/rand"
	"testing"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/pointcloud"
)

func TestFitSphere_PerfectSphere(t *testing.T) {
	// Generate a complete sphere with known center and radius.
	center := r3.Vector{X: 100, Y: 50, Z: 30}
	radius := 40.0
	cloud := generateSyntheticSphere(center, radius, 500, 0)

	cfg := SphereFitConfig{
		RANSACIterations:    500,
		InlierThresholdMm:   2.0,
		ExpectedRadiusMinMm: 25,
		ExpectedRadiusMaxMm: 60,
		MinInlierFraction:    0.5,
	}

	result, err := FitSphere(cloud, cfg)
	if err != nil {
		t.Fatalf("FitSphere failed: %v", err)
	}

	// Check center within 1mm.
	centerErr := result.Center.Sub(center).Norm()
	if centerErr > 1.0 {
		t.Errorf("center error %.2f mm > 1.0 mm (got %v, want %v)", centerErr, result.Center, center)
	}

	// Check radius within 1mm.
	radiusErr := math.Abs(result.Radius - radius)
	if radiusErr > 1.0 {
		t.Errorf("radius error %.2f mm > 1.0 mm (got %.2f, want %.2f)", radiusErr, result.Radius, radius)
	}

	// Check inlier fraction is high.
	if result.InlierFraction < 0.9 {
		t.Errorf("inlier fraction %.2f < 0.9", result.InlierFraction)
	}

	t.Logf("center error: %.3f mm, radius error: %.3f mm, inlier fraction: %.2f, visible: %.2f",
		centerErr, radiusErr, result.InlierFraction, result.VisibleFraction)
}

func TestFitSphere_NoisySphere(t *testing.T) {
	center := r3.Vector{X: 0, Y: 0, Z: 50}
	radius := 35.0
	cloud := generateSyntheticSphere(center, radius, 400, 0.5)

	cfg := SphereFitConfig{
		RANSACIterations:    1000,
		InlierThresholdMm:   2.0,
		ExpectedRadiusMinMm: 25,
		ExpectedRadiusMaxMm: 60,
		MinInlierFraction:    0.5,
	}

	result, err := FitSphere(cloud, cfg)
	if err != nil {
		t.Fatalf("FitSphere failed: %v", err)
	}

	centerErr := result.Center.Sub(center).Norm()
	if centerErr > 2.0 {
		t.Errorf("center error %.2f mm > 2.0 mm", centerErr)
	}

	radiusErr := math.Abs(result.Radius - radius)
	if radiusErr > 2.0 {
		t.Errorf("radius error %.2f mm > 2.0 mm", radiusErr)
	}

	t.Logf("center error: %.3f mm, radius error: %.3f mm, RMS: %.3f mm",
		centerErr, radiusErr, result.RMSResidual)
}

func TestFitSphere_PartialSphere(t *testing.T) {
	// Only generate the top hemisphere (z > center.z).
	center := r3.Vector{X: 0, Y: 0, Z: 0}
	radius := 40.0
	cloud := generatePartialSphere(center, radius, 300, 0.3, 0)

	cfg := SphereFitConfig{
		RANSACIterations:    1000,
		InlierThresholdMm:   2.0,
		ExpectedRadiusMinMm: 25,
		ExpectedRadiusMaxMm: 60,
		MinInlierFraction:    0.3,
	}

	result, err := FitSphere(cloud, cfg)
	if err != nil {
		t.Fatalf("FitSphere failed: %v", err)
	}

	centerErr := result.Center.Sub(center).Norm()
	if centerErr > 3.0 {
		t.Errorf("center error %.2f mm > 3.0 mm", centerErr)
	}

	t.Logf("center error: %.3f mm, radius: %.2f mm, visible fraction: %.2f",
		centerErr, result.Radius, result.VisibleFraction)

	// Visible fraction should be less than 0.6 for a hemisphere.
	if result.VisibleFraction > 0.7 {
		t.Errorf("visible fraction %.2f > 0.7 for a partial sphere", result.VisibleFraction)
	}
}

func TestFitSphere_TooFewPoints(t *testing.T) {
	cloud := pointcloud.NewBasicEmpty()
	cloud.Set(r3.Vector{X: 0, Y: 0, Z: 0}, nil) //nolint:errcheck
	cloud.Set(r3.Vector{X: 1, Y: 0, Z: 0}, nil) //nolint:errcheck

	cfg := SphereFitConfig{
		RANSACIterations:    100,
		InlierThresholdMm:   2.0,
		ExpectedRadiusMinMm: 25,
		ExpectedRadiusMaxMm: 60,
		MinInlierFraction:    0.5,
	}

	_, err := FitSphere(cloud, cfg)
	if err != ErrTooFewPoints {
		t.Errorf("expected ErrTooFewPoints, got %v", err)
	}
}

func TestFitSphere_RadiusOutOfRange(t *testing.T) {
	// Generate a sphere with radius 10mm (below minimum 25mm).
	center := r3.Vector{X: 0, Y: 0, Z: 0}
	radius := 10.0
	cloud := generateSyntheticSphere(center, radius, 200, 0)

	cfg := SphereFitConfig{
		RANSACIterations:    500,
		InlierThresholdMm:   2.0,
		ExpectedRadiusMinMm: 25,
		ExpectedRadiusMaxMm: 60,
		MinInlierFraction:    0.5,
	}

	_, err := FitSphere(cloud, cfg)
	if err != ErrNoSphereFound {
		t.Errorf("expected ErrNoSphereFound for out-of-range radius, got %v", err)
	}
}

func TestAlgebraicSphereFit(t *testing.T) {
	center := r3.Vector{X: 10, Y: 20, Z: 30}
	radius := 40.0

	// Generate exact points on the sphere.
	//nolint:gosec
	rng := rand.New(rand.NewSource(99))
	points := make([]r3.Vector, 100)
	for i := range points {
		theta := rng.Float64() * 2 * math.Pi
		phi := math.Acos(2*rng.Float64() - 1)
		points[i] = r3.Vector{
			X: center.X + radius*math.Sin(phi)*math.Cos(theta),
			Y: center.Y + radius*math.Sin(phi)*math.Sin(theta),
			Z: center.Z + radius*math.Cos(phi),
		}
	}

	fitCenter, fitRadius, ok := algebraicSphereFit(points)
	if !ok {
		t.Fatal("algebraicSphereFit failed")
	}

	centerErr := fitCenter.Sub(center).Norm()
	if centerErr > 0.01 {
		t.Errorf("center error %.6f > 0.01", centerErr)
	}

	radiusErr := math.Abs(fitRadius - radius)
	if radiusErr > 0.01 {
		t.Errorf("radius error %.6f > 0.01", radiusErr)
	}
}

func TestEstimateVisibleFraction_FullSphere(t *testing.T) {
	center := r3.Vector{}
	radius := 30.0

	//nolint:gosec
	rng := rand.New(rand.NewSource(7))
	points := make([]r3.Vector, 500)
	for i := range points {
		theta := rng.Float64() * 2 * math.Pi
		phi := math.Acos(2*rng.Float64() - 1)
		points[i] = r3.Vector{
			X: center.X + radius*math.Sin(phi)*math.Cos(theta),
			Y: center.Y + radius*math.Sin(phi)*math.Sin(theta),
			Z: center.Z + radius*math.Cos(phi),
		}
	}

	vis := estimateVisibleFraction(points, center, radius)
	if vis < 0.85 {
		t.Errorf("full sphere visible fraction %.2f < 0.85", vis)
	}
}

// generateSyntheticSphere creates a point cloud of points on a sphere with optional noise.
func generateSyntheticSphere(center r3.Vector, radius float64, nPoints int, noiseMm float64) pointcloud.PointCloud {
	cloud := pointcloud.NewBasicEmpty()
	//nolint:gosec
	rng := rand.New(rand.NewSource(123))

	for i := 0; i < nPoints; i++ {
		theta := rng.Float64() * 2 * math.Pi
		phi := math.Acos(2*rng.Float64() - 1)
		r := radius + noiseMm*(2*rng.Float64()-1)
		pt := r3.Vector{
			X: center.X + r*math.Sin(phi)*math.Cos(theta),
			Y: center.Y + r*math.Sin(phi)*math.Sin(theta),
			Z: center.Z + r*math.Cos(phi),
		}
		//nolint:errcheck
		cloud.Set(pt, pointcloud.NewColoredData(color.NRGBA{R: 200, G: 50, B: 50, A: 255}))
	}
	return cloud
}

// generatePartialSphere creates a point cloud of points on a hemisphere.
func generatePartialSphere(center r3.Vector, radius float64, nPoints int, minCosAngle, noiseMm float64) pointcloud.PointCloud {
	cloud := pointcloud.NewBasicEmpty()
	//nolint:gosec
	rng := rand.New(rand.NewSource(456))

	for cloud.Size() < nPoints {
		theta := rng.Float64() * 2 * math.Pi
		phi := math.Acos(2*rng.Float64() - 1)
		// Only keep points in the upper hemisphere (cos(phi) > minCosAngle).
		if math.Cos(phi) < minCosAngle {
			continue
		}
		r := radius + noiseMm*(2*rng.Float64()-1)
		pt := r3.Vector{
			X: center.X + r*math.Sin(phi)*math.Cos(theta),
			Y: center.Y + r*math.Sin(phi)*math.Sin(theta),
			Z: center.Z + r*math.Cos(phi),
		}
		//nolint:errcheck
		cloud.Set(pt, pointcloud.NewColoredData(color.NRGBA{R: 200, G: 50, B: 50, A: 255}))
	}
	return cloud
}
