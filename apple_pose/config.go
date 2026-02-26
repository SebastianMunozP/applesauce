package applepose

import "github.com/golang/geo/r3"

// Config holds all configuration for the apple pose detection pipeline.
type Config struct {
	SphereFit    SphereFitConfig
	Scene        SceneConfig
	Feature      FeatureConfig
	ViewPlanning ViewPlanningConfig
	Tracking     TrackingConfig
}

// SphereFitConfig holds parameters for RANSAC sphere fitting.
type SphereFitConfig struct {
	RANSACIterations     int     // Number of RANSAC iterations
	InlierThresholdMm    float64 // Max distance from sphere surface to count as inlier
	ExpectedRadiusMinMm  float64 // Minimum expected apple radius in mm
	ExpectedRadiusMaxMm  float64 // Maximum expected apple radius in mm
	MinInlierFraction    float64 // Minimum fraction of points that must be inliers
	MinVisibleFraction   float64 // Min visible sphere fraction to accept; 0 = no filter
	MaxSpheresPerCluster int     // Max spheres to fit per cluster via iterative RANSAC; 0 or 1 = single fit
	MinInliersPerSphere  int     // Min inlier count for iterative sphere fits; 0 = use MinClusterSize
}

// SceneConfig holds parameters for scene preprocessing.
type SceneConfig struct {
	GroundNormal       r3.Vector // Expected ground plane normal
	PlaneIterations    int       // RANSAC iterations for plane segmentation
	PlaneAngleThreshold float64  // Max angle (degrees) between detected and expected normal
	PlaneDistThreshold float64   // Max distance to plane for a point to belong to it
	ClusteringRadiusMm float64   // Radius for neighbor-based clustering
	MinClusterSize     int       // Minimum points per cluster
	OutlierMeanK       int       // K for statistical outlier filter
	OutlierStdDev      float64   // Standard deviation threshold for outlier filter
	MaxDepthMm         float64   // Max distance from camera origin; 0 = no limit
	AppleHueMax        float64   // Upper hue bound (covers red→green); 0 = no color filter
	AppleHueRedWrapMin float64   // Lower bound for red wraparound hues (e.g. 330-360)
	AppleMinSaturation float64   // Min saturation for apple color filter
	AppleMinValue      float64   // Min brightness for apple color filter
}

// FeatureConfig holds parameters for stem/calyx detection.
type FeatureConfig struct {
	CurvatureNeighbors    int     // K for PCA curvature estimation
	DeviationThresholdMm  float64 // Min sphere deviation to flag anomaly
	AnomalyClusterRadius  float64 // Radius for clustering anomaly points (mm)
	MinAnomalyPoints      int     // Minimum points in an anomaly cluster
	StemHueMin            float64 // Min HSV hue for stem-like color (degrees)
	StemHueMax            float64 // Max HSV hue for stem-like color (degrees)
	StemMaxSaturation     float64 // Max saturation for stem-like color
}

// ViewPlanningConfig holds parameters for view suggestion.
type ViewPlanningConfig struct {
	ViewDistanceMm         float64 // Distance from apple center for suggested camera poses
	NumCandidateViews      int     // Number of candidate views to generate
	MinVisibleFractionGap  float64 // Min gap in visibility to trigger suggestion
}

// TrackingConfig holds parameters for cross-view tracking.
type TrackingConfig struct {
	MaxMatchDistanceMm float64 // Max center distance for matching apples across views
	CarryForwardViews  int     // Number of views to carry forward unmatched apples
}

// DefaultConfig returns a Config with sensible defaults.
func DefaultConfig() Config {
	return Config{
		SphereFit: SphereFitConfig{
			RANSACIterations:     2000,
			InlierThresholdMm:    4.0,
			ExpectedRadiusMinMm:  25.0,
			ExpectedRadiusMaxMm:  50.0,
			MinInlierFraction:    0.0,
			MinVisibleFraction:   0.18,
			MaxSpheresPerCluster: 8,
			MinInliersPerSphere:  200,
		},
		Scene: SceneConfig{
			GroundNormal:       r3.Vector{X: 0, Y: 0, Z: 1},
			PlaneIterations:    2000,
			PlaneAngleThreshold: 5.0,
			PlaneDistThreshold: 3.0,
			ClusteringRadiusMm: 15.0,
			MinClusterSize:     50,
			OutlierMeanK:       8,
			OutlierStdDev:      1.25,
			MaxDepthMm:         0,
			AppleHueMax:        160.0,
			AppleHueRedWrapMin: 310.0,
			AppleMinSaturation: 0.15,
			AppleMinValue:      0.10,
		},
		Feature: FeatureConfig{
			CurvatureNeighbors:    15,
			DeviationThresholdMm:  2.0,
			AnomalyClusterRadius:  5.0,
			MinAnomalyPoints:      5,
			StemHueMin:            20.0,
			StemHueMax:            90.0,
			StemMaxSaturation:     0.8,
		},
		ViewPlanning: ViewPlanningConfig{
			ViewDistanceMm:         200.0,
			NumCandidateViews:      8,
			MinVisibleFractionGap:  0.3,
		},
		Tracking: TrackingConfig{
			MaxMatchDistanceMm: 120.0,
			CarryForwardViews:  3,
		},
	}
}
