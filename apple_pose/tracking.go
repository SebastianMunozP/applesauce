package applepose

import (
	"fmt"
	"math"

	"github.com/golang/geo/r3"
)

// mergeDetections merges new detection results with previous results for cross-view tracking.
func mergeDetections(current, prev *DetectionResult, cfg TrackingConfig) {
	if prev == nil || len(prev.Bowl.Apples) == 0 {
		return
	}

	prevApples := prev.Bowl.Apples
	curApples := current.Bowl.Apples

	// Match each current apple to the closest previous apple by sphere center distance.
	matched := make(map[int]bool)    // Indices into prevApples that have been matched.
	matchMap := make(map[int]int)    // curIdx -> prevIdx

	for ci, curApple := range curApples {
		curCenter := curApple.Pose.Point()
		bestDist := math.MaxFloat64
		bestPrev := -1

		for pi, prevApple := range prevApples {
			if matched[pi] {
				continue
			}
			prevCenter := prevApple.Pose.Point()
			dist := curCenter.Sub(prevCenter).Norm()

			maxDist := cfg.MaxMatchDistanceMm
			if maxDist <= 0 {
				maxDist = curApple.Radius * 2
			}

			if dist < maxDist && dist < bestDist {
				bestDist = dist
				bestPrev = pi
			}
		}

		if bestPrev >= 0 {
			matched[bestPrev] = true
			matchMap[ci] = bestPrev
		}
	}

	// Merge matched apples.
	for ci, pi := range matchMap {
		mergeApple(&curApples[ci], &prevApples[pi])
	}
	current.Bowl.Apples = curApples

	// Carry forward unmatched previous apples (they may be temporarily occluded).
	for pi, prevApple := range prevApples {
		if matched[pi] {
			continue
		}
		// Carry forward with a note that it's from a previous view.
		carried := prevApple
		carried.ID = fmt.Sprintf("%s_carried", prevApple.ID)
		current.Bowl.Apples = append(current.Bowl.Apples, carried)
	}
}

// mergeApple merges a previous apple's data into the current detection.
func mergeApple(current, prev *Apple) {
	// Preserve stable ID from previous detection.
	current.ID = prev.ID

	// Merge features: keep higher-confidence versions.
	current.Features = mergeFeatures(current.Features, prev.Features)

	// Update visible fraction: union of observed areas.
	current.VisibleFraction = math.Min(1.0, current.VisibleFraction+prev.VisibleFraction*0.5)

	// Recompute orientation if we now have more features.
	current.Pose = computeAppleOrientation(current.Pose.Point(), current.Features)
}

// mergeFeatures combines feature detections from two views, keeping the highest confidence for each type.
func mergeFeatures(current, prev []FeatureDetection) []FeatureDetection {
	// Index current features by type.
	byType := make(map[AppleFeature]*FeatureDetection)
	for i := range current {
		existing, ok := byType[current[i].Feature]
		if !ok || current[i].Confidence > existing.Confidence {
			byType[current[i].Feature] = &current[i]
		}
	}

	// Merge in previous features.
	for i := range prev {
		existing, ok := byType[prev[i].Feature]
		if !ok || prev[i].Confidence > existing.Confidence {
			byType[prev[i].Feature] = &prev[i]
		}
	}

	result := make([]FeatureDetection, 0, len(byType))
	for _, f := range byType {
		result = append(result, *f)
	}
	return result
}

// matchApplesByDistance finds the best matching apple in candidates for a given center position.
func matchApplesByDistance(center r3.Vector, candidates []Apple, maxDist float64) (int, bool) {
	bestIdx := -1
	bestDist := math.MaxFloat64

	for i, c := range candidates {
		dist := center.Sub(c.Pose.Point()).Norm()
		if dist < maxDist && dist < bestDist {
			bestDist = dist
			bestIdx = i
		}
	}

	return bestIdx, bestIdx >= 0
}
