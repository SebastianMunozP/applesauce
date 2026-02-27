package applesauce

import (
	"github.com/golang/geo/r3"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// Joint positions recorded from the live robot on 2026-02-24.
var (
	// PrimaryViewingJoints is the joint position for the primary arm (xarm6)
	// to view the bowl area. Recorded from xarm6's position.
	PrimaryViewingJoints = []referenceframe.Input{
		0.24424977600574496, 0.48869219422340393, -2.2238824367523193, -0.032807692885398865, 1.754069805145264, -1.5615639686584473,
	}

	// PeelingCrankGraspJoints is the joint position for the peeling arm
	// to grasp the crank handle. Recorded from peeling-arm's position.
	PeelingCrankGraspJoints []referenceframe.Input

	// SecondaryViewingJoints is the joint position for the secondary arm
	// to view the apple in the gripper. Recorded 2026-02-26.
	SecondaryViewingJoints = []referenceframe.Input{
		-2.503503, 0.622075, 2.234387, -2.208932, -1.992098, 3.766498,
	}

	// RemoveAppleApproachJoints is the joint position for the primary arm
	// to approach the peeled apple on the peeler before linearly descending.
	// Recorded 2026-02-26
	RemoveAppleApproachJoints []referenceframe.Input

	// PrimaryBowlScanAngles is a set of joint positions for the primary arm
	// to scan the bowl from multiple angles during grasp planning.
	// STUB: needs recording of 3-4 viewpoints around the bowl.
	PrimaryBowlScanAngles [][]referenceframe.Input
)

// World-frame poses for key locations.
// Poses that are nil are stubs and must be measured before use.
var (
	// PeelerDevicePose is the world-frame pose of the peeler spike assembly.
	// STUB: measure the peeler device location relative to the world frame.
	PeelerDevicePose spatialmath.Pose

	// PeelerJawsPose is the world-frame pose of the peeler jaws (where the apple
	// is lowered into before being pushed onto spikes).
	// STUB: measure jaw position relative to world frame.
	PeelerJawsPose spatialmath.Pose

	// PeeledAppleBowlPose is the world-frame pose of the bowl/bin for peeled apples.
	// STUB: measure peeled apple deposit location.
	PeeledAppleBowlPose spatialmath.Pose

	// WasteBinPose is the world-frame pose of the waste bin for cores/scraps.
	// STUB: measure waste bin location.
	WasteBinPose spatialmath.Pose

	// SecondaryReleaseLeverApproach is the world-frame pose for the secondary arm
	// to approach the release lever. Recorded 2026-02-25.
	SecondaryReleaseLeverApproach spatialmath.Pose

	// SecondaryReleaseLeverPressPose is the world-frame pose where the secondary arm
	// presses down on the release lever. Recorded 2026-02-25.
	SecondaryReleaseLeverPressPose spatialmath.Pose

	// PeelerCorePose is the world-frame pose of the apple core on the peeler
	// after peeling is complete (where to grab it from).
	// STUB: measure the cored apple location on peeler.
	PeelerCorePose spatialmath.Pose

	PeelerAppleGrabPose spatialmath.Pose

	// PeelerSpikeRetractPose is the position the peeling arm moves to
	// in order to fully retract the spikes from the core. Recorded 2026-02-25.
	PeelerSpikeRetractPose spatialmath.Pose

	// CrankCenter is the center of the crank circle in the world frame.
	// Recorded 2026-02-25: peeling-gripper world XY + (gripper Z + 59.5mm).
	CrankCenter r3.Vector

	// CrankAxis is the unit vector along which the spiral advances.
	// Defaults to -X (the peeler spike direction).
	CrankAxis = r3.Vector{X: 1, Y: 0, Z: 0}

	// BowlRegionBox is a world-frame bounding box used to filter point clouds
	// so that only points within the bowl region are passed to apple detection.
	BowlRegionBox spatialmath.Geometry

	PeelAbovePose = spatialmath.NewPose(
		r3.Vector{X: -20.0367459504155, Y: 455.09934339883876, Z: 489.76104212680764},
		&spatialmath.OrientationVector{OX: 0.007941793676801446, OY: -0.04479245698646934, OZ: -0.9989647459797121, Theta: -1.4246285716599694},
	)

	PeelAlignPose = spatialmath.NewPose(
		r3.Vector{X: -5.488100718055788, Y: 464.83175452406, Z: 255.7844815950868},
		&spatialmath.OrientationVector{OX: 0.007941793676801446, OY: -0.04479245698646934, OZ: -0.9989647459797121, Theta: -1.4246285716599694},
	)

	PeelPiercePose = spatialmath.NewPose(
		r3.Vector{X: -45, Y: 464.83175452406, Z: 255.7844815950868},
		&spatialmath.OrientationVector{OX: 0.007941793676801446, OY: -0.04479245698646934, OZ: -0.9989647459797121, Theta: -1.4246285716599694},
	)

	CrankStartPose = spatialmath.NewPose(
		r3.Vector{X: -334.8481615183904, Y: 470, Z: 170},
		&spatialmath.OrientationVector{OX: 0.006, OY: -0.04411434417065586, OZ: -0.9989968896575422, Theta: -1.4286907032643186},
	)
)
