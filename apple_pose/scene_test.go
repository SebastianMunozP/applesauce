package applepose

import (
	"context"
	"image/color"
	"math"
	"math/rand"
	"testing"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/pointcloud"
)

func TestRadiusClustering_TwoClusters(t *testing.T) {
	cloud := pointcloud.NewBasicEmpty()
	//nolint:gosec
	rng := rand.New(rand.NewSource(42))
	clr := pointcloud.NewColoredData(color.NRGBA{R: 200, G: 50, B: 50, A: 255})

	// Cluster 1: around (0, 0, 0)
	for i := 0; i < 100; i++ {
		pt := r3.Vector{
			X: rng.Float64()*10 - 5,
			Y: rng.Float64()*10 - 5,
			Z: rng.Float64()*10 - 5,
		}
		cloud.Set(pt, clr) //nolint:errcheck
	}

	// Cluster 2: around (100, 100, 100)
	for i := 0; i < 100; i++ {
		pt := r3.Vector{
			X: 100 + rng.Float64()*10 - 5,
			Y: 100 + rng.Float64()*10 - 5,
			Z: 100 + rng.Float64()*10 - 5,
		}
		cloud.Set(pt, clr) //nolint:errcheck
	}

	clusters, err := radiusClustering(cloud, 15.0, 10)
	if err != nil {
		t.Fatalf("radiusClustering failed: %v", err)
	}

	if len(clusters) != 2 {
		t.Errorf("expected 2 clusters, got %d", len(clusters))
	}

	for i, c := range clusters {
		t.Logf("cluster %d: %d points", i, c.Size())
		if c.Size() < 50 {
			t.Errorf("cluster %d has only %d points", i, c.Size())
		}
	}
}

func TestRadiusClustering_PruneSmall(t *testing.T) {
	cloud := pointcloud.NewBasicEmpty()
	clr := pointcloud.NewColoredData(color.NRGBA{R: 200, G: 50, B: 50, A: 255})

	//nolint:gosec
	rng := rand.New(rand.NewSource(42))

	// Big cluster.
	for i := 0; i < 100; i++ {
		pt := r3.Vector{
			X: rng.Float64()*10 - 5,
			Y: rng.Float64()*10 - 5,
			Z: rng.Float64()*10 - 5,
		}
		cloud.Set(pt, clr) //nolint:errcheck
	}

	// Small cluster (should be pruned).
	for i := 0; i < 5; i++ {
		pt := r3.Vector{
			X: 200 + rng.Float64()*2,
			Y: 200 + rng.Float64()*2,
			Z: 200 + rng.Float64()*2,
		}
		cloud.Set(pt, clr) //nolint:errcheck
	}

	clusters, err := radiusClustering(cloud, 15.0, 20)
	if err != nil {
		t.Fatalf("radiusClustering failed: %v", err)
	}

	if len(clusters) != 1 {
		t.Errorf("expected 1 cluster after pruning, got %d", len(clusters))
	}
}

func TestPreprocessScene_SyntheticPlaneAndClusters(t *testing.T) {
	cloud := pointcloud.NewBasicEmpty()
	clr := pointcloud.NewColoredData(color.NRGBA{R: 200, G: 50, B: 50, A: 255})

	//nolint:gosec
	rng := rand.New(rand.NewSource(7))

	// Generate a ground plane at Z=0.
	for i := 0; i < 500; i++ {
		pt := r3.Vector{
			X: rng.Float64()*200 - 100,
			Y: rng.Float64()*200 - 100,
			Z: rng.Float64()*2 - 1, // Slightly noisy around Z=0.
		}
		cloud.Set(pt, clr) //nolint:errcheck
	}

	// Generate a sphere cluster above the plane.
	center := r3.Vector{X: 0, Y: 0, Z: 50}
	radius := 35.0
	for i := 0; i < 200; i++ {
		theta := rng.Float64() * 2 * math.Pi
		phi := math.Acos(2*rng.Float64() - 1)
		pt := r3.Vector{
			X: center.X + radius*math.Sin(phi)*math.Cos(theta),
			Y: center.Y + radius*math.Sin(phi)*math.Sin(theta),
			Z: center.Z + radius*math.Cos(phi),
		}
		cloud.Set(pt, clr) //nolint:errcheck
	}

	cfg := SceneConfig{
		GroundNormal:       r3.Vector{X: 0, Y: 0, Z: 1},
		PlaneIterations:    1000,
		PlaneAngleThreshold: 30.0,
		PlaneDistThreshold: 5.0,
		ClusteringRadiusMm: 10.0,
		MinClusterSize:     20,
		OutlierMeanK:       8,
		OutlierStdDev:      2.0,
	}

	result, err := preprocessScene(context.Background(), cloud, cfg)
	if err != nil {
		t.Fatalf("preprocessScene failed: %v", err)
	}

	if result.plane == nil {
		t.Fatal("expected a plane to be detected")
	}

	t.Logf("plane equation: %v", result.plane.Equation())
	t.Logf("above-plane points: %d", result.abovePlane.Size())
	t.Logf("clusters: %d", len(result.clusters))

	if len(result.clusters) < 1 {
		t.Error("expected at least 1 cluster above the plane")
	}
}

func TestIsBowl(t *testing.T) {
	// Two spheres close together should be detected as a bowl.
	results := []*SphereFitResult{
		{Center: r3.Vector{X: 0, Y: 0, Z: 50}, Radius: 35},
		{Center: r3.Vector{X: 70, Y: 0, Z: 50}, Radius: 35},
	}
	clusters := make([]pointcloud.PointCloud, 2)
	clusters[0] = pointcloud.NewBasicEmpty()
	clusters[1] = pointcloud.NewBasicEmpty()

	if !isBowl(clusters, results) {
		t.Error("expected bowl detection for nearby spheres")
	}

	// Two spheres far apart should not be a bowl.
	results[1].Center = r3.Vector{X: 500, Y: 500, Z: 50}
	if isBowl(clusters, results) {
		t.Error("did not expect bowl detection for distant spheres")
	}
}
