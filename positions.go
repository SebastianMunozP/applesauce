package applesauce

import (
	"github.com/golang/geo/r3"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// Joint positions recorded from the live robot on 2026-02-24.
var (
	// PrimaryViewingJoints is the joint position for the primary arm (xarm7)
	// to view the bowl area. Recorded from xarm7's position.
	PrimaryViewingJoints = []referenceframe.Input{
		1.230044, -1.299491, -0.010149, 1.676676, 0.410073, 2.003047, 0.437884,
	}

	// PeelingCrankGraspJoints is the joint position for the peeling arm
	// to grasp the crank handle. Recorded from peeling-arm's position.
	PeelingCrankGraspJoints = []referenceframe.Input{
		-2.294511, -0.168569, -1.236994, 3.271444, 0.133731, -0.133307,
	}

	// SecondaryViewingJoints is the joint position for the secondary arm
	// to view the apple in the gripper.
	// STUB: needs separate recording session.
	SecondaryViewingJoints []referenceframe.Input

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
	SecondaryReleaseLeverApproach spatialmath.Pose = spatialmath.NewPose(
		r3.Vector{X: 393.915651, Y: 575.522783, Z: 1097.009589},
		&spatialmath.OrientationVectorDegrees{OX: 0.152865, OY: -0.943185, OZ: -0.295017, Theta: -2.790274},
	)

	// SecondaryReleaseLeverPressPose is the world-frame pose where the secondary arm
	// presses down on the release lever. Recorded 2026-02-25.
	SecondaryReleaseLeverPressPose spatialmath.Pose = spatialmath.NewPose(
		r3.Vector{X: 399.667893, Y: 537.463280, Z: 1096.999404},
		&spatialmath.OrientationVectorDegrees{OX: 0.152865, OY: -0.943185, OZ: -0.295017, Theta: -2.790273},
	)

	// PeelerCorePose is the world-frame pose of the apple core on the peeler
	// after peeling is complete (where to grab it from).
	// STUB: measure the cored apple location on peeler.
	PeelerCorePose spatialmath.Pose

	// PeelerSpikeRetractPose is the position the peeling arm moves to
	// in order to fully retract the spikes from the core. Recorded 2026-02-25.
	PeelerSpikeRetractPose spatialmath.Pose = spatialmath.NewPose(
		r3.Vector{X: 603.053212, Y: 302.081609, Z: 914.735288},
		&spatialmath.OrientationVectorDegrees{OX: -0.032604, OY: 0.674660, OZ: 0.737409, Theta: -92.336915},
	)

	// CrankCenter is the center of the crank circle in the world frame.
	// The crank rotates in the YZ plane at this center point.
	// Recorded 2026-02-25: peeling-gripper world XY + (gripper Z + 59.5mm).
	CrankCenter = r3.Vector{X: 596.858401, Y: 430.266941, Z: 1114.342907}

)
