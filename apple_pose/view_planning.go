package applepose

import (
	"math"
	"sort"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/spatialmath"
)

// PlanViews generates suggested camera poses and physical actions to reveal hidden features.
func PlanViews(result *DetectionResult, cfg ViewPlanningConfig) ([]SuggestedView, []ActionSuggestion) {
	if result == nil {
		return nil, nil
	}

	var views []SuggestedView
	var actions []ActionSuggestion

	supportNormal := planeNormal(result.Bowl.SupportPlane)

	for i, apple := range result.Bowl.Apples {
		appleViews, appleActions := planViewsForApple(i, apple, supportNormal, cfg)
		views = append(views, appleViews...)
		actions = append(actions, appleActions...)
	}

	// Sort views by priority descending.
	sort.Slice(views, func(i, j int) bool {
		return views[i].Priority > views[j].Priority
	})

	return views, actions
}

func planViewsForApple(
	index int,
	apple Apple,
	supportNormal r3.Vector,
	cfg ViewPlanningConfig,
) ([]SuggestedView, []ActionSuggestion) {
	var views []SuggestedView
	var actions []ActionSuggestion

	center := apple.Pose.Point()

	// Determine what features are missing.
	hasStem := false
	hasCalyx := false
	for _, f := range apple.Features {
		switch f.Feature {
		case FeatureStem:
			hasStem = true
		case FeatureCalyx:
			hasCalyx = true
		}
	}

	// Compute observed hemisphere direction.
	observedDir := computeObservedDirection(apple.Points, center)

	// Hidden direction is antipodal.
	hiddenDir := observedDir.Mul(-1)

	// Check if hidden direction faces the support surface (below the apple).
	hiddenDotNormal := hiddenDir.Dot(supportNormal)

	if !hasStem || !hasCalyx {
		if hiddenDotNormal < -0.5 {
			// Hidden area faces the support surface — suggest flipping.
			flipAxis := hiddenDir.Cross(supportNormal)
			norm := flipAxis.Norm()
			if norm > 1e-9 {
				flipAxis = flipAxis.Mul(1.0 / norm)
			} else {
				flipAxis = r3.Vector{X: 1}
			}

			actions = append(actions, ActionSuggestion{
				Type:             ActionFlipApple,
				Description:      "Flip apple to reveal hidden hemisphere facing support surface",
				TargetAppleIndex: index,
				Rotation: &spatialmath.R4AA{
					Theta: math.Pi,
					RX:    flipAxis.X,
					RY:    flipAxis.Y,
					RZ:    flipAxis.Z,
				},
			})
		} else {
			// Generate candidate camera views in the hidden hemisphere.
			purpose := FindStem
			if hasStem && !hasCalyx {
				purpose = FindCalyx
			}

			candidates := generateCandidateViews(
				center, hiddenDir, observedDir, supportNormal,
				cfg.ViewDistanceMm, cfg.NumCandidateViews,
			)

			for _, cam := range candidates {
				views = append(views, SuggestedView{
					CameraPose:       cam.pose,
					TargetAppleIndex: index,
					Purpose:          purpose,
					Priority:         cam.priority,
				})
			}
		}
	}

	// If visibility is low, suggest additional views regardless of features.
	if apple.VisibleFraction < (1.0 - cfg.MinVisibleFractionGap) {
		candidates := generateCandidateViews(
			center, hiddenDir, observedDir, supportNormal,
			cfg.ViewDistanceMm, cfg.NumCandidateViews/2+1,
		)
		for _, cam := range candidates {
			views = append(views, SuggestedView{
				CameraPose:       cam.pose,
				TargetAppleIndex: index,
				Purpose:          IncreaseVisibility,
				Priority:         cam.priority * 0.7, // Lower priority than feature finding.
			})
		}
	}

	return views, actions
}

type candidateView struct {
	pose     spatialmath.Pose
	priority float64
}

func generateCandidateViews(
	appleCenter, hiddenDir, observedDir, supportNormal r3.Vector,
	distance float64,
	numCandidates int,
) []candidateView {
	var candidates []candidateView

	// Generate views in the hidden hemisphere.
	goldenAngle := math.Pi * (3 - math.Sqrt(5))

	for i := 0; i < numCandidates; i++ {
		// Distribute points in the hidden hemisphere.
		t := float64(i) / float64(numCandidates)
		phi := math.Acos(1 - t) // 0 to pi/2 — only hidden hemisphere
		theta := goldenAngle * float64(i)

		// Construct direction in a local frame where hiddenDir is the pole.
		localDir := r3.Vector{
			X: math.Sin(phi) * math.Cos(theta),
			Y: math.Sin(phi) * math.Sin(theta),
			Z: math.Cos(phi),
		}

		// Rotate local frame to align Z with hiddenDir.
		worldDir := rotateToAlign(localDir, hiddenDir)

		// Camera position = apple center + direction * distance.
		camPos := appleCenter.Add(worldDir.Mul(distance))

		// Camera must be above support plane.
		if camPos.Dot(supportNormal) < appleCenter.Dot(supportNormal) {
			continue
		}

		// Camera looks toward apple center.
		lookDir := appleCenter.Sub(camPos)
		norm := lookDir.Norm()
		if norm < 1e-9 {
			continue
		}
		lookDir = lookDir.Mul(1.0 / norm)

		ov := &spatialmath.OrientationVector{
			OX: lookDir.X,
			OY: lookDir.Y,
			OZ: lookDir.Z,
		}
		camPose := spatialmath.NewPose(camPos, ov)

		// Priority: higher for directions further from observed area.
		cosAngleFromObserved := worldDir.Dot(observedDir)
		priority := (1.0 - cosAngleFromObserved) / 2.0 // Map [-1,1] to [0,1]

		candidates = append(candidates, candidateView{
			pose:     camPose,
			priority: priority,
		})
	}

	return candidates
}

// computeObservedDirection computes the mean direction of observed points relative to center.
func computeObservedDirection(cloud pointcloud.PointCloud, center r3.Vector) r3.Vector {
	var sum r3.Vector
	cloud.Iterate(0, 0, func(pt r3.Vector, _ pointcloud.Data) bool {
		d := pt.Sub(center)
		n := d.Norm()
		if n > 1e-9 {
			sum = sum.Add(d.Mul(1.0 / n))
		}
		return true
	})

	norm := sum.Norm()
	if norm < 1e-9 {
		return r3.Vector{Z: 1}
	}
	return sum.Mul(1.0 / norm)
}

// rotateToAlign rotates a vector from a frame where Z is "up" to a frame where targetZ is "up".
func rotateToAlign(v, targetZ r3.Vector) r3.Vector {
	z := r3.Vector{Z: 1}

	// If targetZ is already Z, no rotation needed.
	dot := z.Dot(targetZ)
	if dot > 0.9999 {
		return v
	}
	if dot < -0.9999 {
		return r3.Vector{X: -v.X, Y: -v.Y, Z: -v.Z}
	}

	// Rodrigues' rotation formula.
	axis := z.Cross(targetZ)
	axisNorm := axis.Norm()
	axis = axis.Mul(1.0 / axisNorm)

	cosA := dot
	sinA := axisNorm

	// v_rot = v*cos(a) + (axis x v)*sin(a) + axis*(axis.v)*(1-cos(a))
	return v.Mul(cosA).Add(axis.Cross(v).Mul(sinA)).Add(axis.Mul(axis.Dot(v) * (1 - cosA)))
}
