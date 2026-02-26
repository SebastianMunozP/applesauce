package applepose

import (
	"context"
	"fmt"
	"math"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/spatialmath"
)

// Detector runs the apple detection pipeline.
type Detector struct {
	cfg Config
}

// NewDetector creates a new Detector with the given configuration.
func NewDetector(cfg *Config) *Detector {
	if cfg == nil {
		c := DefaultConfig()
		cfg = &c
	}
	return &Detector{cfg: *cfg}
}

// Detect runs the full detection pipeline on a single point cloud.
func (d *Detector) Detect(ctx context.Context, cloud pointcloud.PointCloud) (*DetectionResult, error) {
	if cloud == nil {
		return nil, ErrNilPointCloud
	}

	// Step 1-2: Preprocess scene (outlier removal, plane segmentation, clustering).
	scene, err := preprocessScene(ctx, cloud, d.cfg.Scene)
	if err != nil {
		return nil, fmt.Errorf("scene preprocessing: %w", err)
	}

	// Step 3-5: Iterative sphere fitting per cluster, then build Apple structs.
	supportNormal := planeNormal(scene.plane)
	var apples []Apple
	unclassified := pointcloud.NewBasicEmpty()

	// Collect all sphere results for bowl detection.
	var allSphereResults []*SphereFitResult

	for _, cluster := range scene.clusters {
		spheres := d.fitSpheresIterative(cluster)
		if len(spheres) == 0 {
			cluster.Iterate(0, 0, func(pt r3.Vector, dd pointcloud.Data) bool {
				//nolint:errcheck
				unclassified.Set(pt, dd)
				return true
			})
			continue
		}

		for _, sr := range spheres {
			// Reject spheres whose center is below the table plane.
			if !isAbovePlane(sr.Center, scene.plane, d.cfg.Scene.GroundNormal) {
				continue
			}

			allSphereResults = append(allSphereResults, sr)

			features := detectStemCalyx(
				sr.InlierCloud,
				sr.Center,
				sr.Radius,
				supportNormal,
				d.cfg.Feature,
			)

			pose := computeAppleOrientation(sr.Center, features)

			sphere, _ := spatialmath.NewSphere(
				spatialmath.NewPoseFromPoint(sr.Center),
				sr.Radius,
				fmt.Sprintf("apple_%d", len(apples)),
			)

			apple := Apple{
				ID:              fmt.Sprintf("apple_%d", len(apples)),
				Sphere:          sphere,
				Pose:            pose,
				Radius:          sr.Radius,
				FitResidual:     sr.RMSResidual,
				InlierFraction:  sr.InlierFraction,
				Features:        features,
				Points:          sr.InlierCloud,
				VisibleFraction: sr.VisibleFraction,
			}
			apples = append(apples, apple)
		}
	}

	bowlDetected := isBowl(nil, allSphereResults)

	if len(apples) == 0 {
		return nil, ErrNoAppleDetected
	}

	result := &DetectionResult{
		BowlDetected: bowlDetected,
		Bowl: BowlOfApples{
			Apples:             apples,
			SupportPlane:       scene.plane,
			UnclassifiedPoints: unclassified,
		},
	}

	// Step 6: View planning.
	views, actions := PlanViews(result, d.cfg.ViewPlanning)
	result.SuggestedViews = views
	result.SuggestedActions = actions

	return result, nil
}

// DetectWithHistory runs detection and merges with previous results for cross-view tracking.
func (d *Detector) DetectWithHistory(ctx context.Context, cloud pointcloud.PointCloud, prev *DetectionResult) (*DetectionResult, error) {
	result, err := d.Detect(ctx, cloud)
	if err != nil {
		return nil, err
	}

	if prev != nil {
		mergeDetections(result, prev, d.cfg.Tracking)
	}

	return result, nil
}

// AnalyzeSingleApple fits a sphere and detects features on a single apple point cloud
// with a known approximate center and radius.
func (d *Detector) AnalyzeSingleApple(ctx context.Context, cloud pointcloud.PointCloud, center r3.Vector, radius float64) (*Apple, error) {
	if cloud == nil {
		return nil, ErrNilPointCloud
	}

	sr, err := FitSphere(cloud, d.cfg.SphereFit)
	if err != nil {
		return nil, err
	}

	supportNormal := d.cfg.Scene.GroundNormal
	features := detectStemCalyx(cloud, sr.Center, sr.Radius, supportNormal, d.cfg.Feature)
	pose := computeAppleOrientation(sr.Center, features)

	sphere, _ := spatialmath.NewSphere(
		spatialmath.NewPoseFromPoint(sr.Center),
		sr.Radius,
		"apple_single",
	)

	apple := &Apple{
		ID:              "apple_0",
		Sphere:          sphere,
		Pose:            pose,
		Radius:          sr.Radius,
		FitResidual:     sr.RMSResidual,
		InlierFraction:  sr.InlierFraction,
		Features:        features,
		Points:          cloud,
		VisibleFraction: sr.VisibleFraction,
	}

	return apple, nil
}

// fitSpheresIterative fits one or more spheres to a cluster using iterative RANSAC.
// After each successful sphere fit, inlier points are removed and the process repeats.
// If the color filter is enabled, non-apple-colored points are stripped before fitting.
func (d *Detector) fitSpheresIterative(cloud pointcloud.PointCloud) []*SphereFitResult {
	// Per-cluster color filter: remove non-apple colors (e.g. bowl) before sphere fitting.
	filtered := d.filterClusterByColor(cloud)

	maxSpheres := d.cfg.SphereFit.MaxSpheresPerCluster
	if maxSpheres <= 1 {
		// Single fit mode (backwards compatible).
		sr, err := FitSphere(filtered, d.cfg.SphereFit)
		if err != nil {
			return nil
		}
		if d.cfg.SphereFit.MinInlierFraction > 0 && sr.InlierFraction < d.cfg.SphereFit.MinInlierFraction {
			return nil
		}
		if d.cfg.SphereFit.MinVisibleFraction > 0 && sr.VisibleFraction < d.cfg.SphereFit.MinVisibleFraction {
			return nil
		}
		return []*SphereFitResult{sr}
	}

	minInliers := d.cfg.SphereFit.MinInliersPerSphere
	if minInliers <= 0 {
		minInliers = d.cfg.Scene.MinClusterSize
	}
	thresh := d.cfg.SphereFit.InlierThresholdMm

	var results []*SphereFitResult
	remaining := filtered

	for len(results) < maxSpheres {
		if remaining.Size() < minInliers {
			break
		}

		sr, err := FitSphere(remaining, d.cfg.SphereFit)
		if err != nil {
			break
		}

		if d.cfg.SphereFit.MinVisibleFraction > 0 && sr.VisibleFraction < d.cfg.SphereFit.MinVisibleFraction {
			break
		}

		// Count actual inliers in remaining cloud.
		inlierCount := 0
		remaining.Iterate(0, 0, func(pt r3.Vector, dd pointcloud.Data) bool {
			dist := math.Abs(pt.Sub(sr.Center).Norm() - sr.Radius)
			if dist <= thresh {
				inlierCount++
			}
			return true
		})

		if inlierCount < minInliers {
			break
		}

		results = append(results, sr)

		// Remove inliers from remaining points.
		next := pointcloud.NewBasicEmpty()
		remaining.Iterate(0, 0, func(pt r3.Vector, dd pointcloud.Data) bool {
			dist := math.Abs(pt.Sub(sr.Center).Norm() - sr.Radius)
			if dist > thresh {
				//nolint:errcheck
				next.Set(pt, dd)
			}
			return true
		})
		remaining = next
	}

	// Exclusive reassignment: each point goes to the single closest sphere.
	// This prevents shared points from inflating adjacent sphere radii.
	if len(results) > 1 {
		results = d.reassignAndRefit(filtered, results)
	}

	return results
}

// reassignAndRefit merges overlapping spheres, assigns each point exclusively to
// the nearest sphere surface, then recomputes radii. Merging fixes cases where the
// same physical apple produces two nearby sphere fits; exclusive assignment prevents
// boundary points from inflating adjacent sphere radii.
func (d *Detector) reassignAndRefit(cloud pointcloud.PointCloud, results []*SphereFitResult) []*SphereFitResult {
	thresh := d.cfg.SphereFit.InlierThresholdMm
	cfg := d.cfg.SphereFit

	// Merge overlapping spheres: if two centers are within max(r1,r2), combine
	// their inlier clouds and refit. This handles cases where one physical apple
	// produces two competing sphere fits.
	results = d.mergeOverlappingSpheres(results)

	for iter := 0; iter < 3; iter++ {
		// Assign each point to the sphere whose surface is closest.
		assignedPts := make([][]r3.Vector, len(results))
		assignedClouds := make([]pointcloud.PointCloud, len(results))
		for i := range results {
			assignedClouds[i] = pointcloud.NewBasicEmpty()
		}

		cloud.Iterate(0, 0, func(pt r3.Vector, dd pointcloud.Data) bool {
			bestIdx := -1
			bestDist := math.MaxFloat64
			for i, sr := range results {
				dist := math.Abs(pt.Sub(sr.Center).Norm() - sr.Radius)
				if dist <= thresh && dist < bestDist {
					bestDist = dist
					bestIdx = i
				}
			}
			if bestIdx >= 0 {
				assignedPts[bestIdx] = append(assignedPts[bestIdx], pt)
				//nolint:errcheck
				assignedClouds[bestIdx].Set(pt, dd)
			}
			return true
		})

		// Refit radius for each sphere from its exclusive point set.
		changed := false
		for i, pts := range assignedPts {
			if len(pts) < 4 {
				continue
			}
			var sumDist float64
			for _, pt := range pts {
				sumDist += pt.Sub(results[i].Center).Norm()
			}
			newRadius := sumDist / float64(len(pts))
			if newRadius >= cfg.ExpectedRadiusMinMm && newRadius <= cfg.ExpectedRadiusMaxMm {
				if math.Abs(newRadius-results[i].Radius) > 0.01 {
					changed = true
				}
				results[i].Radius = newRadius
				results[i].InlierCloud = assignedClouds[i]
				results[i].InlierFraction = float64(len(pts)) / float64(cloud.Size())
				results[i].VisibleFraction = estimateVisibleFraction(pts, results[i].Center, newRadius)

				// Recompute RMS.
				var sumSqErr float64
				for _, pt := range pts {
					diff := pt.Sub(results[i].Center).Norm() - newRadius
					sumSqErr += diff * diff
				}
				results[i].RMSResidual = math.Sqrt(sumSqErr / float64(len(pts)))
			}
		}

		if !changed {
			break
		}
	}

	// Re-filter by visible fraction after reassignment.
	var kept []*SphereFitResult
	for _, sr := range results {
		if cfg.MinVisibleFraction > 0 && sr.VisibleFraction < cfg.MinVisibleFraction {
			continue
		}
		kept = append(kept, sr)
	}
	return kept
}

// mergeOverlappingSpheres combines sphere fits whose centers are within one radius
// of each other. The merged sphere is refit from the union of both inlier clouds.
func (d *Detector) mergeOverlappingSpheres(results []*SphereFitResult) []*SphereFitResult {
	if len(results) <= 1 {
		return results
	}

	merged := make([]bool, len(results))
	for i := 0; i < len(results); i++ {
		if merged[i] {
			continue
		}
		for j := i + 1; j < len(results); j++ {
			if merged[j] {
				continue
			}
			dist := results[i].Center.Sub(results[j].Center).Norm()
			maxR := math.Max(results[i].Radius, results[j].Radius)
			if dist < maxR {
				// Merge j into i: union the inlier clouds and refit.
				combined := pointcloud.NewBasicEmpty()
				results[i].InlierCloud.Iterate(0, 0, func(pt r3.Vector, dd pointcloud.Data) bool {
					//nolint:errcheck
					combined.Set(pt, dd)
					return true
				})
				results[j].InlierCloud.Iterate(0, 0, func(pt r3.Vector, dd pointcloud.Data) bool {
					//nolint:errcheck
					combined.Set(pt, dd)
					return true
				})

				sr, err := FitSphere(combined, d.cfg.SphereFit)
				if err == nil {
					results[i] = sr
				}
				merged[j] = true
			}
		}
	}

	var kept []*SphereFitResult
	for i, sr := range results {
		if !merged[i] {
			kept = append(kept, sr)
		}
	}
	return kept
}

// filterClusterByColor removes non-apple-colored points from a cluster.
// Skipped if the color filter is disabled (AppleHueMax <= 0).
// Unlike the scene-level color filter, this runs per-cluster and won't interfere
// with plane segmentation.
func (d *Detector) filterClusterByColor(cloud pointcloud.PointCloud) pointcloud.PointCloud {
	if d.cfg.Scene.AppleHueMax <= 0 {
		return cloud
	}
	filtered := pointcloud.NewBasicEmpty()
	cloud.Iterate(0, 0, func(pt r3.Vector, dd pointcloud.Data) bool {
		if dd == nil || !dd.HasColor() {
			//nolint:errcheck
			filtered.Set(pt, dd)
			return true
		}
		r, g, b := dd.RGB255()
		if isAppleColored(r, g, b, d.cfg.Scene) {
			//nolint:errcheck
			filtered.Set(pt, dd)
		}
		return true
	})
	// Fall back to unfiltered if too few points survive.
	if filtered.Size() < 4 {
		return cloud
	}
	return filtered
}

// planeNormal extracts the normal vector from a plane, defaulting to Z-up.
func planeNormal(plane pointcloud.Plane) r3.Vector {
	if plane == nil {
		return r3.Vector{Z: 1}
	}
	eq := plane.Equation()
	n := r3.Vector{X: eq[0], Y: eq[1], Z: eq[2]}
	norm := n.Norm()
	if norm < 1e-9 {
		return r3.Vector{Z: 1}
	}
	return n.Mul(1.0 / norm)
}

// isAbovePlane returns true if the point is above (or on) the plane, where "above"
// is defined as the side in the direction of groundNormal.
func isAbovePlane(pt r3.Vector, plane pointcloud.Plane, groundNormal r3.Vector) bool {
	if plane == nil {
		return true
	}
	eq := plane.Equation()
	normal := r3.Vector{X: eq[0], Y: eq[1], Z: eq[2]}
	d := eq[3]

	// Ensure the normal points in the same direction as groundNormal (i.e. "up").
	if normal.Dot(groundNormal) < 0 {
		normal = normal.Mul(-1)
		d = -d
	}

	// Signed distance: positive = above the plane.
	signedDist := normal.X*pt.X + normal.Y*pt.Y + normal.Z*pt.Z + d
	return signedDist >= 0
}
