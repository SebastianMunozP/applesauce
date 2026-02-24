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

	// SecondaryReleaseLeverApproach is the joint position for the secondary arm
	// to approach the release lever on the peeling machine.
	// Recorded from secondary-arm's position.
	SecondaryReleaseLeverApproach = []referenceframe.Input{
		0.434442, 1.405237, 1.217567, 0.444506, -1.727239, 0.045132,
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

	// SecondaryReleaseLeverPressPose is the world-frame pose where the secondary arm
	// presses down on the release lever.
	// STUB: measure the lever press-down position.
	SecondaryReleaseLeverPressPose spatialmath.Pose

	// PeelerCorePose is the world-frame pose of the apple core on the peeler
	// after peeling is complete (where to grab it from).
	// STUB: measure the cored apple location on peeler.
	PeelerCorePose spatialmath.Pose

	// PeelerSpikeRetractPose is the position the peeling arm moves to
	// in order to fully retract the spikes from the core.
	// STUB: measure the retracted spike position.
	PeelerSpikeRetractPose spatialmath.Pose

	// CrankCenter is the center of the crank circle in the world frame.
	// The crank rotates in the YZ plane at this center point.
	// Recorded 2026-02-24: peeling-gripper pose in world + 59.5mm Z.
	CrankCenter = r3.Vector{X: 596.858401, Y: 430.266941, Z: 1114.342907}
)
