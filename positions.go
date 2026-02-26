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
		-2.286582, -0.211757, -1.216493, 3.367374, 0.112762, -0.228739,
	}

	// SecondaryViewingJoints is the joint position for the secondary arm
	// to view the apple in the gripper.
	// STUB: needs separate recording session.
	SecondaryViewingJoints []referenceframe.Input

	// RemoveAppleApproachJoints is the joint position for the primary arm
	// to approach the peeled apple on the peeler before linearly descending.
	// Recorded 2026-02-26
	RemoveAppleApproachJoints = []referenceframe.Input{
		-1.354595, -0.679198, -4.706562, 2.271673, 0.941830, 2.145737, 0.403602,
	}

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

	// PeeledAppleBowlPose is the world-frame pose above the bowl/bin for peeled apples.
	// The arm moves directly here and opens the gripper to drop.
	// Recorded 2026-02-25.
	PeeledAppleBowlPose spatialmath.Pose = spatialmath.NewPose(
		r3.Vector{X: -650.867109, Y: 118.290116, Z: 1073.602847},
		&spatialmath.OrientationVectorDegrees{OX: -0.020939, OY: -0.121366, OZ: -0.992387, Theta: -30.325381},
	)

	// WasteBinPose is the world-frame pose of the waste bin for cores/scraps.
	// Recorded 2026-02-26.
	WasteBinPose spatialmath.Pose = spatialmath.NewPose(
		r3.Vector{X: 223.607667, Y: 286.245697, Z: 1370.946137},
		&spatialmath.OrientationVectorDegrees{OX: 0.074031, OY: -0.004713, OZ: -0.997245, Theta: 41.535133},
	)

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
	// Recorded 2026-02-26.
	PeelerCorePose spatialmath.Pose = spatialmath.NewPose(
		r3.Vector{X: 90.823915, Y: 463.229148, Z: 1170.803074},
		&spatialmath.OrientationVectorDegrees{OX: 0.999315, OY: 0.036202, OZ: -0.007734, Theta: -10.991430},
	)

	// PeelerAppleGrabPose is the world-frame pose where the primary arm
	// can grab the peeled apple off the peeler. The gripper descends from
	// above (+Z) and then pulls the apple off in -X.
	// Recorded 2026-02-25.
	PeelerAppleGrabPose spatialmath.Pose = spatialmath.NewPose(
		r3.Vector{X: 223.472639, Y: 467.593875, Z: 1211.834733},
		&spatialmath.OrientationVectorDegrees{OX: -0.114097, OY: 0.046392, OZ: -0.992386, Theta: 136.576857},
	)

	// PeelerSpikeRetractPose is the position the peeling arm moves to
	// in order to fully retract the spikes from the core. Recorded 2026-02-25.
	PeelerSpikeRetractPose spatialmath.Pose = spatialmath.NewPose(
		r3.Vector{X: 603.053212, Y: 302.081609, Z: 914.735288},
		&spatialmath.OrientationVectorDegrees{OX: -0.032604, OY: 0.674660, OZ: 0.737409, Theta: -92.336915},
	)

	// CrankCenter is the center of the crank circle in the world frame.
	// Recorded 2026-02-25: peeling-gripper world XY + (gripper Z + 59.5mm).
	CrankCenter = r3.Vector{X: 583.131634, Y: 412.400876, Z: 1113.198631}

	// CrankAxis is the unit vector along which the spiral advances.
	// Defaults to -X (the peeler spike direction).
	CrankAxis = r3.Vector{X: -1, Y: 0, Z: 0.05}

)
