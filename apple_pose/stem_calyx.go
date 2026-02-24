package applepose

import (
	"math"
	"sort"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/spatialmath"
)

// anomalyCluster represents a group of anomalous points on an apple surface.
type anomalyCluster struct {
	points     []r3.Vector
	data       []pointcloud.Data
	colors     []colorClass
	centroid   r3.Vector
	meanDev    float64 // Mean signed deviation from sphere
	stemFrac   float64 // Fraction of stem-like colored points
	compactness float64 // Smaller = more compact
}

// pointInfo holds per-point analysis results.
type pointInfo struct {
	pt        r3.Vector
	d         pointcloud.Data
	deviation float64
	curvature float64
	color     colorClass
	isAnomaly bool
}

// detectStemCalyx finds stem and calyx features on a fitted apple sphere.
func detectStemCalyx(
	cloud pointcloud.PointCloud,
	center r3.Vector,
	radius float64,
	supportNormal r3.Vector,
	cfg FeatureConfig,
) []FeatureDetection {
	kd := pointcloud.ToKDTree(cloud)

	// Step 1: Compute sphere deviation and curvature for each point.

	var infos []pointInfo
	cloud.Iterate(0, 0, func(pt r3.Vector, d pointcloud.Data) bool {
		dev := pt.Sub(center).Norm() - radius
		curv := estimateLocalCurvature(kd, pt, cfg.CurvatureNeighbors)

		var cc colorClass
		if d != nil && d.HasColor() {
			r, g, b := d.RGB255()
			cc = classifyColor(r, g, b, cfg)
		}

		anomaly := math.Abs(dev) > cfg.DeviationThresholdMm || curvatureIsHigh(curv.Curvature, radius)

		infos = append(infos, pointInfo{
			pt:        pt,
			d:         d,
			deviation: dev,
			curvature: curv.Curvature,
			color:     cc,
			isAnomaly: anomaly,
		})
		return true
	})

	// Step 2: Cluster anomaly points.
	anomalyCloud := pointcloud.NewBasicEmpty()
	for _, info := range infos {
		if info.isAnomaly {
			//nolint:errcheck
			anomalyCloud.Set(info.pt, info.d)
		}
	}

	if anomalyCloud.Size() < cfg.MinAnomalyPoints {
		return nil
	}

	anomalyClusters, err := radiusClustering(anomalyCloud, cfg.AnomalyClusterRadius, cfg.MinAnomalyPoints)
	if err != nil || len(anomalyClusters) == 0 {
		return nil
	}

	// Step 3: Score each anomaly cluster.
	scored := make([]anomalyCluster, 0, len(anomalyClusters))
	for _, ac := range anomalyClusters {
		sc := scoreAnomalyCluster(ac, center, radius, cfg, infos)
		scored = append(scored, sc)
	}

	// Step 4: Classify top candidates as stem/calyx.
	return classifyFeatures(scored, center, radius, supportNormal)
}

// scoreAnomalyCluster computes scoring metrics for an anomaly cluster.
func scoreAnomalyCluster(
	cluster pointcloud.PointCloud,
	center r3.Vector,
	radius float64,
	cfg FeatureConfig,
	allInfos []pointInfo,
) anomalyCluster {
	var points []r3.Vector
	var data []pointcloud.Data
	var colors []colorClass
	var sumDev float64
	var cx, cy, cz float64

	// Build lookup for quick info retrieval.
	infoMap := make(map[r3.Vector]*pointInfo, len(allInfos))
	for i := range allInfos {
		infoMap[allInfos[i].pt] = &allInfos[i]
	}

	cluster.Iterate(0, 0, func(pt r3.Vector, d pointcloud.Data) bool {
		points = append(points, pt)
		data = append(data, d)
		cx += pt.X
		cy += pt.Y
		cz += pt.Z

		if info, ok := infoMap[pt]; ok {
			sumDev += info.deviation
			colors = append(colors, info.color)
		}
		return true
	})

	n := float64(len(points))
	centroid := r3.Vector{X: cx / n, Y: cy / n, Z: cz / n}

	// Compute compactness (mean distance from centroid).
	var sumDist float64
	for _, pt := range points {
		sumDist += pt.Sub(centroid).Norm()
	}
	compactness := sumDist / n

	return anomalyCluster{
		points:     points,
		data:       data,
		colors:     colors,
		centroid:   centroid,
		meanDev:    sumDev / n,
		stemFrac:   stemLikeFraction(colors),
		compactness: compactness,
	}
}

// classifyFeatures assigns stem/calyx labels to the top anomaly clusters.
func classifyFeatures(
	clusters []anomalyCluster,
	center r3.Vector,
	radius float64,
	supportNormal r3.Vector,
) []FeatureDetection {
	if len(clusters) == 0 {
		return nil
	}

	// Score each cluster: higher = more likely a feature.
	type scoredCluster struct {
		cluster anomalyCluster
		score   float64
		relZ    float64 // Height relative to sphere center along support normal
	}

	scored := make([]scoredCluster, 0, len(clusters))
	for _, c := range clusters {
		// Direction from center to cluster centroid.
		dir := c.centroid.Sub(center)
		relZ := dir.Dot(supportNormal)

		// Score: combination of non-skin color, compactness, and deviation.
		score := c.stemFrac*0.4 + (1.0-c.compactness/radius)*0.3
		// Cavities (negative deviation) are more likely calyx.
		if c.meanDev < 0 {
			score += 0.3
		} else {
			score += 0.15
		}

		scored = append(scored, scoredCluster{
			cluster: c,
			score:   score,
			relZ:    relZ,
		})
	}

	// Sort by score descending.
	sort.Slice(scored, func(i, j int) bool {
		return scored[i].score > scored[j].score
	})

	var features []FeatureDetection

	if len(scored) >= 2 {
		// Two candidates: higher one (along support normal) is stem, lower is calyx.
		top := scored[0]
		second := scored[1]

		var stem, calyx scoredCluster
		if top.relZ >= second.relZ {
			stem = top
			calyx = second
		} else {
			stem = second
			calyx = top
		}

		features = append(features,
			makeFeatureDetection(FeatureStem, stem.cluster, center, math.Min(stem.score, 0.9)),
			makeFeatureDetection(FeatureCalyx, calyx.cluster, center, math.Min(calyx.score, 0.85)),
		)
	} else if len(scored) == 1 {
		c := scored[0]
		// Single candidate: classify based on position and color.
		if c.relZ > 0 && c.cluster.stemFrac > 0.3 {
			features = append(features, makeFeatureDetection(FeatureStem, c.cluster, center, c.score*0.7))
		} else {
			features = append(features, makeFeatureDetection(FeatureCalyx, c.cluster, center, c.score*0.6))
		}
	}

	return features
}

// makeFeatureDetection creates a FeatureDetection from an anomaly cluster.
func makeFeatureDetection(feature AppleFeature, cluster anomalyCluster, sphereCenter r3.Vector, confidence float64) FeatureDetection {
	// Feature pose: position = centroid, orientation = outward normal from sphere center.
	outward := cluster.centroid.Sub(sphereCenter)
	norm := outward.Norm()
	if norm < 1e-9 {
		outward = r3.Vector{Z: 1}
	} else {
		outward = outward.Mul(1.0 / norm)
	}

	ov := &spatialmath.OrientationVector{
		OX: outward.X,
		OY: outward.Y,
		OZ: outward.Z,
	}

	pose := spatialmath.NewPose(cluster.centroid, ov)

	// Build point cloud for the feature.
	featureCloud := pointcloud.NewBasicEmpty()
	for i, pt := range cluster.points {
		//nolint:errcheck
		featureCloud.Set(pt, cluster.data[i])
	}

	return FeatureDetection{
		Feature:    feature,
		Pose:       pose,
		Confidence: confidence,
		Points:     featureCloud,
	}
}

// computeAppleOrientation determines the apple's orientation from stem and calyx positions.
// The calyx-to-stem vector defines the apple's "up" axis.
func computeAppleOrientation(center r3.Vector, features []FeatureDetection) spatialmath.Pose {
	var stemPos, calyxPos *r3.Vector
	for _, f := range features {
		pos := f.Pose.Point()
		switch f.Feature {
		case FeatureStem:
			stemPos = &pos
		case FeatureCalyx:
			calyxPos = &pos
		}
	}

	if stemPos != nil && calyxPos != nil {
		// Calyx-to-stem direction defines the apple's up axis.
		axis := stemPos.Sub(*calyxPos)
		norm := axis.Norm()
		if norm > 1e-9 {
			axis = axis.Mul(1.0 / norm)
			ov := &spatialmath.OrientationVector{
				OX: axis.X,
				OY: axis.Y,
				OZ: axis.Z,
			}
			return spatialmath.NewPose(center, ov)
		}
	}

	if stemPos != nil {
		axis := stemPos.Sub(center)
		norm := axis.Norm()
		if norm > 1e-9 {
			axis = axis.Mul(1.0 / norm)
			ov := &spatialmath.OrientationVector{
				OX: axis.X,
				OY: axis.Y,
				OZ: axis.Z,
			}
			return spatialmath.NewPose(center, ov)
		}
	}

	// No features found; use identity orientation.
	return spatialmath.NewPoseFromPoint(center)
}
