package applepose

import (
	"github.com/golang/geo/r3"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/spatialmath"
)

// AppleFeature identifies the type of surface feature on an apple.
type AppleFeature int

const (
	// FeatureStem is the stem attachment point.
	FeatureStem AppleFeature = iota
	// FeatureCalyx is the calyx (blossom end).
	FeatureCalyx
)

func (f AppleFeature) String() string {
	switch f {
	case FeatureStem:
		return "stem"
	case FeatureCalyx:
		return "calyx"
	default:
		return "unknown"
	}
}

// ViewPurpose describes why a particular viewpoint is suggested.
type ViewPurpose int

const (
	FindStem ViewPurpose = iota
	FindCalyx
	ConfirmStem
	ConfirmCalyx
	IncreaseVisibility
)

func (v ViewPurpose) String() string {
	switch v {
	case FindStem:
		return "find_stem"
	case FindCalyx:
		return "find_calyx"
	case ConfirmStem:
		return "confirm_stem"
	case ConfirmCalyx:
		return "confirm_calyx"
	case IncreaseVisibility:
		return "increase_visibility"
	default:
		return "unknown"
	}
}

// ActionType describes a physical action suggestion.
type ActionType int

const (
	ActionFlipApple ActionType = iota
	ActionMoveCamera
)

func (a ActionType) String() string {
	switch a {
	case ActionFlipApple:
		return "flip_apple"
	case ActionMoveCamera:
		return "move_camera"
	default:
		return "unknown"
	}
}

// Apple represents a detected apple with fitted geometry and features.
type Apple struct {
	ID              string
	Sphere          spatialmath.Geometry
	Pose            spatialmath.Pose
	Radius          float64
	FitResidual     float64
	InlierFraction  float64
	Features        []FeatureDetection
	Points          pointcloud.PointCloud
	VisibleFraction float64
}

// FeatureDetection represents a detected stem or calyx on an apple.
type FeatureDetection struct {
	Feature    AppleFeature
	Pose       spatialmath.Pose
	Confidence float64
	Points     pointcloud.PointCloud
}

// BowlOfApples represents a group of apples detected together.
type BowlOfApples struct {
	Apples             []Apple
	SupportPlane       pointcloud.Plane
	UnclassifiedPoints pointcloud.PointCloud
}

// DetectionResult is the output of the detection pipeline.
type DetectionResult struct {
	BowlDetected     bool
	Bowl             BowlOfApples
	SuggestedViews   []SuggestedView
	SuggestedActions []ActionSuggestion
}

// SuggestedView is a camera pose suggestion to reveal hidden features.
type SuggestedView struct {
	CameraPose       spatialmath.Pose
	TargetAppleIndex int
	Purpose          ViewPurpose
	Priority         float64
}

// ActionSuggestion is a physical manipulation suggestion.
type ActionSuggestion struct {
	Type             ActionType
	Description      string
	TargetAppleIndex int
	Rotation         *spatialmath.R4AA
}

// SphereFitResult holds the output of sphere fitting.
type SphereFitResult struct {
	Center          r3.Vector
	Radius          float64
	RMSResidual     float64
	InlierFraction  float64
	InlierCloud     pointcloud.PointCloud
	VisibleFraction float64
}
