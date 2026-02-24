package applesauce

import (
	"context"
	"fmt"

	"github.com/golang/geo/r3"

	applepose "github.com/biotinker/applesauce/apple_pose"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
)

// Robot holds all hardware references, services, and state for the peeling pipeline.
type Robot struct {
	logger  logging.Logger
	machine robot.Robot

	// Arms
	primaryArm   arm.Arm
	secondaryArm arm.Arm
	peelingArm   arm.Arm

	// Grippers
	appleGripper   gripper.Gripper
	peelingGripper gripper.Gripper

	// Cameras
	primaryCam   camera.Camera
	secondaryCam camera.Camera

	// Services
	motion motion.Service

	// Detection
	detector *applepose.Detector

	// State
	state *PeelingState
}

// PeelingState tracks the state of the current peeling cycle.
type PeelingState struct {
	// Target apple from detection.
	TargetApple *applepose.Apple

	// Current apple pose in world frame (updated after grasp).
	ApplePose spatialmath.Pose

	// Apple radius in mm.
	AppleRadius float64

	// Stem and calyx poses in world frame.
	StemPose  spatialmath.Pose
	CalyxPose spatialmath.Pose

	// Whether stem and calyx have been found with sufficient confidence.
	StemFound  bool
	CalyxFound bool

	// Feature vector: unit vector from calyx to stem (the apple's axis).
	FeatureVector *r3.Vector

	// Attempt counters.
	GraspAttempts   int
	RegraspAttempts int

	// Last detection result from the camera pipeline.
	LastDetection *applepose.DetectionResult

	// Merged point cloud from multi-angle scanning.
	MergedCloud pointcloud.PointCloud

	// Total apples processed this session.
	ApplesProcessed int
}

// NewRobot creates a Robot by looking up all hardware resources from the machine.
// All resources are required; NewRobot returns an error if any are missing.
func NewRobot(ctx context.Context, machine robot.Robot, logger logging.Logger) (*Robot, error) {
	r := &Robot{
		logger:  logger,
		machine: machine,
		state:   &PeelingState{},
	}

	// Primary arm (xarm7) — required.
	primaryArm, err := arm.FromProvider(machine, "xarm7")
	if err != nil {
		return nil, fmt.Errorf("primary arm (xarm7): %w", err)
	}
	r.primaryArm = primaryArm

	// Secondary arm — required.
	secondaryArm, err := arm.FromProvider(machine, "secondary-arm")
	if err != nil {
		return nil, fmt.Errorf("secondary arm: %w", err)
	}
	r.secondaryArm = secondaryArm

	// Peeling arm — required.
	peelingArm, err := arm.FromProvider(machine, "peeling-arm")
	if err != nil {
		return nil, fmt.Errorf("peeling arm: %w", err)
	}
	r.peelingArm = peelingArm

	// Apple gripper — required.
	appleGripper, err := gripper.FromProvider(machine, "arm_mount")
	if err != nil {
		return nil, fmt.Errorf("apple gripper (arm_mount): %w", err)
	}
	r.appleGripper = appleGripper

	// Peeling gripper — required.
	peelingGripper, err := gripper.FromProvider(machine, "peeling-gripper")
	if err != nil {
		return nil, fmt.Errorf("peeling gripper: %w", err)
	}
	r.peelingGripper = peelingGripper

	// Primary camera — required.
	primaryCam, err := camera.FromProvider(machine, "primary-cam")
	if err != nil {
		return nil, fmt.Errorf("primary camera: %w", err)
	}
	r.primaryCam = primaryCam

	// Secondary camera — required.
	secondaryCam, err := camera.FromProvider(machine, "secondary-cam")
	if err != nil {
		return nil, fmt.Errorf("secondary camera: %w", err)
	}
	r.secondaryCam = secondaryCam

	// Motion service — required.
	motionSvc, err := motion.FromProvider(machine, "builtin")
	if err != nil {
		return nil, fmt.Errorf("motion service: %w", err)
	}
	r.motion = motionSvc

	// Apple pose detector.
	r.detector = applepose.NewDetector(nil)

	return r, nil
}

// moveLinear moves a component to the destination pose using a linear constraint.
// The path will stay within 1mm of a straight line and 2 degrees of orientation.
func (r *Robot) moveLinear(ctx context.Context, componentName string, dest spatialmath.Pose, worldState *referenceframe.WorldState) error {
	constraints := motionplan.NewConstraints(
		[]motionplan.LinearConstraint{{
			LineToleranceMm:          1.0,
			OrientationToleranceDegs: 2.0,
		}},
		nil, nil, nil,
	)

	_, err := r.motion.Move(ctx, motion.MoveReq{
		ComponentName: componentName,
		Destination:   referenceframe.NewPoseInFrame("world", dest),
		WorldState:    worldState,
		Constraints:   constraints,
	})
	return err
}

// moveFree moves a component to the destination pose with no path constraints.
// The motion planner chooses the optimal collision-free path.
func (r *Robot) moveFree(ctx context.Context, componentName string, dest spatialmath.Pose, worldState *referenceframe.WorldState) error {
	_, err := r.motion.Move(ctx, motion.MoveReq{
		ComponentName: componentName,
		Destination:   referenceframe.NewPoseInFrame("world", dest),
		WorldState:    worldState,
	})
	return err
}

// moveToJoints moves an arm to the given joint positions via the motion service,
// which plans a collision-free path that respects configured obstacles.
// Returns an error if joints are nil (stub guard).
func (r *Robot) moveToJoints(ctx context.Context, componentName string, joints []referenceframe.Input) error {
	if joints == nil {
		return fmt.Errorf("cannot move to nil joint positions (position not yet recorded)")
	}
	jointValues := make([]interface{}, len(joints))
	for i, v := range joints {
		jointValues[i] = v
	}
	_, err := r.motion.Move(ctx, motion.MoveReq{
		ComponentName: componentName,
		Extra: map[string]interface{}{
			"goal_state": map[string]interface{}{
				"configuration": map[string]interface{}{
					componentName: jointValues,
				},
			},
		},
	})
	return err
}

// poseAbove returns a new pose shifted upward (+Z) by the given offset in mm.
func poseAbove(base spatialmath.Pose, offsetMm float64) spatialmath.Pose {
	pt := base.Point()
	orient := base.Orientation()
	return spatialmath.NewPose(
		r3.Vector{X: pt.X, Y: pt.Y, Z: pt.Z + offsetMm},
		orient,
	)
}

// resetState clears all peeling state for the next cycle.
func (r *Robot) resetState() {
	r.state = &PeelingState{
		ApplesProcessed: r.state.ApplesProcessed,
	}
}
