package applepose

import (
	"testing"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/spatialmath"
)

func TestMergeDetections_MatchByDistance(t *testing.T) {
	prevApple := Apple{
		ID:              "apple_0",
		Pose:            spatialmath.NewPoseFromPoint(r3.Vector{X: 0, Y: 0, Z: 50}),
		Radius:          35,
		Points:          pointcloud.NewBasicEmpty(),
		VisibleFraction: 0.4,
		Features: []FeatureDetection{
			{
				Feature:    FeatureStem,
				Pose:       spatialmath.NewPoseFromPoint(r3.Vector{X: 0, Y: 0, Z: 85}),
				Confidence: 0.7,
				Points:     pointcloud.NewBasicEmpty(),
			},
		},
	}

	prev := &DetectionResult{
		Bowl: BowlOfApples{
			Apples: []Apple{prevApple},
		},
	}

	// Current detection: same apple, slightly shifted, with calyx but no stem.
	curApple := Apple{
		ID:              "apple_new",
		Pose:            spatialmath.NewPoseFromPoint(r3.Vector{X: 2, Y: 1, Z: 51}),
		Radius:          35,
		Points:          pointcloud.NewBasicEmpty(),
		VisibleFraction: 0.5,
		Features: []FeatureDetection{
			{
				Feature:    FeatureCalyx,
				Pose:       spatialmath.NewPoseFromPoint(r3.Vector{X: 0, Y: 0, Z: 15}),
				Confidence: 0.8,
				Points:     pointcloud.NewBasicEmpty(),
			},
		},
	}

	current := &DetectionResult{
		Bowl: BowlOfApples{
			Apples: []Apple{curApple},
		},
	}

	cfg := TrackingConfig{
		MaxMatchDistanceMm: 120,
		CarryForwardViews:  3,
	}

	mergeDetections(current, prev, cfg)

	if len(current.Bowl.Apples) != 1 {
		t.Errorf("expected 1 apple after merge, got %d", len(current.Bowl.Apples))
	}

	merged := current.Bowl.Apples[0]

	// Should have the stable ID from previous.
	if merged.ID != "apple_0" {
		t.Errorf("expected ID 'apple_0', got '%s'", merged.ID)
	}

	// Should have both features.
	if len(merged.Features) != 2 {
		t.Errorf("expected 2 features after merge, got %d", len(merged.Features))
	}

	hasStem := false
	hasCalyx := false
	for _, f := range merged.Features {
		switch f.Feature {
		case FeatureStem:
			hasStem = true
		case FeatureCalyx:
			hasCalyx = true
		}
	}
	if !hasStem {
		t.Error("expected stem feature after merge")
	}
	if !hasCalyx {
		t.Error("expected calyx feature after merge")
	}

	t.Logf("merged apple: ID=%s, features=%d, visible=%.2f", merged.ID, len(merged.Features), merged.VisibleFraction)
}

func TestMergeDetections_CarryForward(t *testing.T) {
	// Previous has 2 apples. Current only sees 1 (the other is occluded).
	prev := &DetectionResult{
		Bowl: BowlOfApples{
			Apples: []Apple{
				{
					ID:              "apple_0",
					Pose:            spatialmath.NewPoseFromPoint(r3.Vector{X: 0, Y: 0, Z: 50}),
					Radius:          35,
					Points:          pointcloud.NewBasicEmpty(),
					VisibleFraction: 0.5,
				},
				{
					ID:              "apple_1",
					Pose:            spatialmath.NewPoseFromPoint(r3.Vector{X: 100, Y: 0, Z: 50}),
					Radius:          35,
					Points:          pointcloud.NewBasicEmpty(),
					VisibleFraction: 0.5,
				},
			},
		},
	}

	current := &DetectionResult{
		Bowl: BowlOfApples{
			Apples: []Apple{
				{
					ID:              "new_apple",
					Pose:            spatialmath.NewPoseFromPoint(r3.Vector{X: 1, Y: 0, Z: 50}),
					Radius:          35,
					Points:          pointcloud.NewBasicEmpty(),
					VisibleFraction: 0.6,
				},
			},
		},
	}

	cfg := TrackingConfig{
		MaxMatchDistanceMm: 120,
		CarryForwardViews:  3,
	}

	mergeDetections(current, prev, cfg)

	// Should have 2 apples: 1 matched + 1 carried forward.
	if len(current.Bowl.Apples) != 2 {
		t.Errorf("expected 2 apples after merge (1 matched + 1 carried), got %d", len(current.Bowl.Apples))
	}

	for _, a := range current.Bowl.Apples {
		t.Logf("apple: ID=%s, pos=%v", a.ID, a.Pose.Point())
	}
}

func TestMergeFeatures_HigherConfidence(t *testing.T) {
	current := []FeatureDetection{
		{Feature: FeatureStem, Confidence: 0.5, Pose: spatialmath.NewZeroPose(), Points: pointcloud.NewBasicEmpty()},
	}
	prev := []FeatureDetection{
		{Feature: FeatureStem, Confidence: 0.9, Pose: spatialmath.NewZeroPose(), Points: pointcloud.NewBasicEmpty()},
		{Feature: FeatureCalyx, Confidence: 0.7, Pose: spatialmath.NewZeroPose(), Points: pointcloud.NewBasicEmpty()},
	}

	merged := mergeFeatures(current, prev)

	if len(merged) != 2 {
		t.Fatalf("expected 2 features, got %d", len(merged))
	}

	for _, f := range merged {
		switch f.Feature {
		case FeatureStem:
			if f.Confidence != 0.9 {
				t.Errorf("stem should have confidence 0.9 (from prev), got %.1f", f.Confidence)
			}
		case FeatureCalyx:
			if f.Confidence != 0.7 {
				t.Errorf("calyx should have confidence 0.7, got %.1f", f.Confidence)
			}
		}
	}
}

func TestMatchApplesByDistance(t *testing.T) {
	candidates := []Apple{
		{Pose: spatialmath.NewPoseFromPoint(r3.Vector{X: 0, Y: 0, Z: 50})},
		{Pose: spatialmath.NewPoseFromPoint(r3.Vector{X: 100, Y: 0, Z: 50})},
		{Pose: spatialmath.NewPoseFromPoint(r3.Vector{X: 200, Y: 0, Z: 50})},
	}

	// Should match the first candidate.
	idx, found := matchApplesByDistance(r3.Vector{X: 5, Y: 0, Z: 50}, candidates, 50)
	if !found || idx != 0 {
		t.Errorf("expected match at index 0, got idx=%d found=%v", idx, found)
	}

	// Should match the second candidate.
	idx, found = matchApplesByDistance(r3.Vector{X: 105, Y: 0, Z: 50}, candidates, 50)
	if !found || idx != 1 {
		t.Errorf("expected match at index 1, got idx=%d found=%v", idx, found)
	}

	// Too far: no match.
	_, found = matchApplesByDistance(r3.Vector{X: 500, Y: 0, Z: 50}, candidates, 50)
	if found {
		t.Error("expected no match for distant point")
	}
}
