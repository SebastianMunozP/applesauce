package applepose

import (
	"context"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/vision/segmentation"
)

// sceneResult holds the output of scene preprocessing.
type sceneResult struct {
	plane      pointcloud.Plane
	abovePlane pointcloud.PointCloud
	clusters   []pointcloud.PointCloud
}

// preprocessScene removes outliers, segments the support plane, and clusters remaining points.
func preprocessScene(ctx context.Context, cloud pointcloud.PointCloud, cfg SceneConfig) (*sceneResult, error) {
	if cloud == nil {
		return nil, ErrNilPointCloud
	}
	if cloud.Size() < 4 {
		return nil, ErrTooFewPoints
	}

	// Step 0a: Depth filter — discard points beyond MaxDepthMm from camera origin.
	current := cloud
	if cfg.MaxDepthMm > 0 {
		depthFiltered, err := filterByDepth(current, cfg.MaxDepthMm)
		if err != nil {
			return nil, err
		}
		current = depthFiltered
	}

	// Note: color filtering is applied per-cluster in fitSpheresIterative(),
	// not here, because removing non-apple colors before plane segmentation
	// strips the table surface and causes cluster fragmentation.

	// Step 1: Statistical outlier removal.
	filtered := pointcloud.NewBasicEmpty()
	filterFn, err := pointcloud.StatisticalOutlierFilter(cfg.OutlierMeanK, cfg.OutlierStdDev)
	if err != nil {
		return nil, err
	}
	if err := filterFn(current, filtered); err != nil {
		return nil, err
	}

	// Step 2: Plane segmentation (soft failure — if no plane found, skip it).
	var plane pointcloud.Plane
	var remaining pointcloud.PointCloud
	plane, remaining, err = segmentation.SegmentPlaneWRTGround(
		ctx,
		filtered,
		cfg.PlaneIterations,
		cfg.PlaneAngleThreshold,
		cfg.PlaneDistThreshold,
		cfg.GroundNormal,
	)
	if err != nil {
		// Color/depth filtering may have already removed the table plane.
		// Proceed with clustering on the filtered cloud.
		plane = nil
		remaining = filtered
	}

	// Step 3: Cluster the non-plane points directly.
	// We skip SplitPointCloudByPlane because the RDK's Plane.Distance() divides by
	// pt.Norm() instead of the normal magnitude, and the above/below heuristic fails
	// in camera coordinates where Z is depth. remaining already has exactly the
	// non-plane points we need.
	abovePlane := remaining

	clusters, err := radiusClustering(abovePlane, cfg.ClusteringRadiusMm, cfg.MinClusterSize)
	if err != nil {
		return nil, err
	}

	return &sceneResult{
		plane:      plane,
		abovePlane: abovePlane,
		clusters:   clusters,
	}, nil
}

// radiusClustering performs radius-based nearest-neighbor clustering on a point cloud.
func radiusClustering(cloud pointcloud.PointCloud, radiusMm float64, minSize int) ([]pointcloud.PointCloud, error) {
	if cloud.Size() == 0 {
		return nil, nil
	}

	kd := pointcloud.ToKDTree(cloud)
	segments := segmentation.NewSegments()

	// Map from point to cluster assignment.
	visited := make(map[r3.Vector]bool)
	clusterIdx := 0

	// Extract all points and data for iteration.
	type pd struct {
		p r3.Vector
		d pointcloud.Data
	}
	var allPoints []pd
	cloud.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
		allPoints = append(allPoints, pd{p, d})
		return true
	})

	for _, point := range allPoints {
		if visited[point.p] {
			continue
		}

		// BFS from this point.
		queue := []pd{point}
		visited[point.p] = true
		currentCluster := clusterIdx

		for len(queue) > 0 {
			curr := queue[0]
			queue = queue[1:]

			if err := segments.AssignCluster(curr.p, curr.d, currentCluster); err != nil {
				return nil, err
			}

			neighbors := kd.RadiusNearestNeighbors(curr.p, radiusMm, false)
			for _, nb := range neighbors {
				if !visited[nb.P] {
					visited[nb.P] = true
					queue = append(queue, pd{nb.P, nb.D})
				}
			}
		}

		clusterIdx++
	}

	clouds := segments.PointClouds()
	return pointcloud.PrunePointClouds(clouds, minSize), nil
}

// isBowl heuristically determines if clusters form a bowl arrangement.
// Returns true if >= 2 clusters with valid sphere radii are in close proximity.
func isBowl(_ []pointcloud.PointCloud, sphereResults []*SphereFitResult) bool {
	var centers []r3.Vector

	for _, sr := range sphereResults {
		if sr == nil {
			continue
		}
		centers = append(centers, sr.Center)
	}

	if len(centers) < 2 {
		return false
	}

	// Check if any two valid sphere centers are within 4 radii of each other.
	for i := 0; i < len(centers); i++ {
		for j := i + 1; j < len(centers); j++ {
			dist := centers[i].Sub(centers[j]).Norm()
			// If two apple centers are within ~120mm, they're likely in a bowl.
			if dist < 120.0 {
				return true
			}
		}
	}

	return false
}

// filterByDepth returns a new point cloud containing only points within maxDepthMm depth.
// Uses the Z coordinate as depth since Z is the optical axis for depth cameras.
func filterByDepth(cloud pointcloud.PointCloud, maxDepthMm float64) (pointcloud.PointCloud, error) {
	out := pointcloud.NewBasicEmpty()
	cloud.Iterate(0, 0, func(pt r3.Vector, d pointcloud.Data) bool {
		if pt.Z >= 0 && pt.Z <= maxDepthMm {
			//nolint:errcheck
			out.Set(pt, d)
		}
		return true
	})
	if out.Size() < 4 {
		return nil, ErrTooFewPoints
	}
	return out, nil
}

// filterByColor returns a new point cloud containing only apple-colored points.
// Points without color data are kept (conservative).
func filterByColor(cloud pointcloud.PointCloud, cfg SceneConfig) (pointcloud.PointCloud, error) {
	out := pointcloud.NewBasicEmpty()
	cloud.Iterate(0, 0, func(pt r3.Vector, d pointcloud.Data) bool {
		if d == nil || !d.HasColor() {
			//nolint:errcheck
			out.Set(pt, d)
			return true
		}
		r, g, b := d.RGB255()
		if isAppleColored(r, g, b, cfg) {
			//nolint:errcheck
			out.Set(pt, d)
		}
		return true
	})
	if out.Size() < 4 {
		return nil, ErrTooFewPoints
	}
	return out, nil
}
