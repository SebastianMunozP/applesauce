package applesauce

import (
	"context"
	"encoding/json"
	"fmt"
	"math"
	"os"
	"path/filepath"

	"github.com/go-viper/mapstructure/v2"
	"github.com/golang/geo/r3"
	"google.golang.org/protobuf/encoding/protojson"

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

// motionServiceName is the resource name of the builtin motion service.
const motionServiceName = "builtin"

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

	// PlansDir, when set, is a directory for persisting cached
	// trajectories to disk. If empty, plans are cached in memory only.
	PlansDir string

	// Cached trajectories — planned once via DoPlan, reused via DoExecute.
	crankSpiralTrajectory   motionplan.Trajectory
	crankRetractTrajectory  motionplan.Trajectory
	leverDescentTrajectory  motionplan.Trajectory
	leverPressTrajectory    motionplan.Trajectory
	leverReleaseTrajectory  motionplan.Trajectory
	leverRetractTrajectory  motionplan.Trajectory
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
		//~ return nil, fmt.Errorf("primary arm (xarm7): %w", err)
	}
	r.primaryArm = primaryArm

	// Secondary arm — required.
	secondaryArm, err := arm.FromProvider(machine, "secondary-arm")
	if err != nil {
		//~ return nil, fmt.Errorf("secondary arm: %w", err)
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
		//~ return nil, fmt.Errorf("apple gripper (arm_mount): %w", err)
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
		//~ return nil, fmt.Errorf("primary camera: %w", err)
	}
	r.primaryCam = primaryCam

	// Secondary camera — required.
	secondaryCam, err := camera.FromProvider(machine, "secondary-cam")
	if err != nil {
		//~ return nil, fmt.Errorf("secondary camera: %w", err)
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
//
// The motion service requires goal inputs for ALL components with nonzero DOF,
// so we fetch current inputs for every component and only override the target.
func (r *Robot) moveToJoints(ctx context.Context, componentName string, joints []referenceframe.Input) error {
	if joints == nil {
		return fmt.Errorf("cannot move to nil joint positions (position not yet recorded)")
	}

	currentInputs, err := r.machine.CurrentInputs(ctx)
	if err != nil {
		return fmt.Errorf("get current inputs: %w", err)
	}
	fmt.Println("currentInputs", currentInputs)

	configuration := make(map[string]interface{}, len(currentInputs))
	for name, inputs := range currentInputs {
		vals := make([]interface{}, len(inputs))
		for i, v := range inputs {
			vals[i] = v
		}
		configuration[name] = vals
	}

	// Override the target component with the desired goal joints.
	goalVals := make([]interface{}, len(joints))
	for i, v := range joints {
		goalVals[i] = v
	}
	configuration[componentName] = goalVals
	fmt.Println("goal inputs", configuration)

	ret, err := r.motion.Move(ctx, motion.MoveReq{
		ComponentName: componentName,
		Extra: map[string]interface{}{
			"goal_state": map[string]interface{}{
				"configuration": configuration,
			},
		},
	})
	fmt.Println(ret)
	fmt.Println(err)
	return err
}

// secondaryArmJointTolerance is the maximum acceptable joint deviation (radians)
// when verifying the secondary arm reached its target (~0.6 degrees).
const secondaryArmJointTolerance = 0.01

// secondaryArmJoints reads the secondary arm's current joint positions.
func (r *Robot) secondaryArmJoints(ctx context.Context) ([]referenceframe.Input, error) {
	inputs, err := r.secondaryArm.CurrentInputs(ctx)
	if err != nil {
		return nil, fmt.Errorf("read secondary arm joints: %w", err)
	}
	return inputs, nil
}

// confirmSecondaryArmAt verifies the secondary arm reached the expected joint positions.
func (r *Robot) confirmSecondaryArmAt(ctx context.Context, expected []referenceframe.Input) error {
	actual, err := r.secondaryArmJoints(ctx)
	if err != nil {
		return err
	}
	if len(actual) != len(expected) {
		return fmt.Errorf("secondary arm joint count mismatch: got %d, want %d", len(actual), len(expected))
	}
	for i := range expected {
		if math.Abs(actual[i]-expected[i]) > secondaryArmJointTolerance {
			return fmt.Errorf("secondary arm joint %d off target (got %.4f, want %.4f rad)", i, actual[i], expected[i])
		}
	}
	return nil
}

// confirmSecondaryArmMoved verifies the secondary arm actually moved by comparing
// current joints against a pre-move snapshot. Returns an error if all joints are unchanged.
func (r *Robot) confirmSecondaryArmMoved(ctx context.Context, before []referenceframe.Input) error {
	after, err := r.secondaryArmJoints(ctx)
	if err != nil {
		return err
	}
	for i := range before {
		if i >= len(after) {
			break
		}
		if math.Abs(after[i]-before[i]) > secondaryArmJointTolerance {
			return nil // at least one joint moved
		}
	}
	return fmt.Errorf("secondary arm did not move: all joints unchanged after motion command")
}

// doPlan calls the motion service's DoPlan DoCommand to generate a trajectory
// without executing it. The trajectory can be cached and replayed via doExecute.
func (r *Robot) doPlan(ctx context.Context, req motion.MoveReq) (motionplan.Trajectory, error) {
	proto, err := req.ToProto(motionServiceName)
	if err != nil {
		return nil, fmt.Errorf("build plan proto: %w", err)
	}
	bytes, err := protojson.Marshal(proto)
	if err != nil {
		return nil, fmt.Errorf("marshal plan request: %w", err)
	}
	resp, err := r.motion.DoCommand(ctx, map[string]interface{}{
		"plan": string(bytes),
	})
	if err != nil {
		return nil, fmt.Errorf("DoPlan: %w", err)
	}
	raw, ok := resp["plan"]
	if !ok {
		return nil, fmt.Errorf("DoPlan response missing 'plan' key")
	}
	var trajectory motionplan.Trajectory
	if err := mapstructure.Decode(raw, &trajectory); err != nil {
		return nil, fmt.Errorf("decode trajectory: %w", err)
	}
	return trajectory, nil
}

// doExecute calls the motion service's DoExecute DoCommand to replay a cached trajectory.
func (r *Robot) doExecute(ctx context.Context, trajectory motionplan.Trajectory) error {
	r.logger.Debugf("doExecute: %d trajectory steps", len(trajectory))
	if len(trajectory) > 0 {
		r.logger.Debugf("doExecute: first step: %v", trajectory[0])
		r.logger.Debugf("doExecute: last step:  %v", trajectory[len(trajectory)-1])
	}

	cmd := map[string]interface{}{
		"execute": trajectory,
	}
	r.logger.Debugf("doExecute: cmd type for 'execute' key: %T", cmd["execute"])

	resp, err := r.motion.DoCommand(ctx, cmd)
	if err != nil {
		return fmt.Errorf("DoExecute: %w", err)
	}
	if ok, _ := resp["execute"].(bool); !ok {
		return fmt.Errorf("DoExecute returned non-true response: %v", resp["execute"])
	}
	return nil
}

// cachedLinearMove plans (or replays from cache) a linear-constrained move to
// dest for the given component. traj must point to a trajectory field on Robot;
// it is populated on first call and reused thereafter.
func (r *Robot) cachedLinearMove(ctx context.Context, componentName string, dest spatialmath.Pose, traj *motionplan.Trajectory, cacheFile string) error {
	if *traj == nil {
		*traj = r.loadCachedTrajectory(cacheFile)
	}
	if *traj == nil {
		r.logger.Infof("Planning %s (first run; will be cached)", cacheFile)
		req := motion.MoveReq{
			ComponentName: componentName,
			Destination:   referenceframe.NewPoseInFrame("world", dest),
			Constraints: motionplan.NewConstraints(
				[]motionplan.LinearConstraint{{
					LineToleranceMm:          1.0,
					OrientationToleranceDegs: 2.0,
				}},
				nil, nil, nil,
			),
		}
		planned, err := r.doPlan(ctx, req)
		if err != nil {
			return err
		}
		*traj = planned
		r.saveCachedTrajectory(cacheFile, planned)
	}
	return r.doExecute(ctx, *traj)
}

// loadCachedTrajectory loads a trajectory from PlansDir/filename.
// Returns nil if PlansDir is unset, the file doesn't exist, or parsing fails.
func (r *Robot) loadCachedTrajectory(filename string) motionplan.Trajectory {
	if r.PlansDir == "" {
		return nil
	}
	path := filepath.Join(r.PlansDir, filename)
	data, err := os.ReadFile(path)
	if err != nil {
		return nil
	}
	var traj motionplan.Trajectory
	if err := json.Unmarshal(data, &traj); err != nil {
		r.logger.Warnf("Failed to parse cached trajectory %s: %v", path, err)
		return nil
	}
	r.logger.Infof("Loaded cached trajectory from %s (%d steps)", path, len(traj))
	return traj
}

// saveCachedTrajectory writes a trajectory to CrankPlansDir/filename as JSON.
// No-op if CrankPlansDir is unset; logs a warning on write failure.
func (r *Robot) saveCachedTrajectory(filename string, traj motionplan.Trajectory) {
	if r.PlansDir == "" {
		return
	}
	if err := os.MkdirAll(r.PlansDir, 0o755); err != nil {
		r.logger.Warnf("Failed to create plans dir %s: %v", r.PlansDir, err)
		return
	}
	path := filepath.Join(r.PlansDir, filename)
	data, err := json.Marshal(traj)
	if err != nil {
		r.logger.Warnf("Failed to serialize trajectory for %s: %v", path, err)
		return
	}
	if err := os.WriteFile(path, data, 0o644); err != nil {
		r.logger.Warnf("Failed to write trajectory to %s: %v", path, err)
		return
	}
	r.logger.Infof("Saved trajectory to %s (%d steps)", path, len(traj))
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

// LastDetection returns the most recent detection result from the camera pipeline.
func (r *Robot) LastDetection() *applepose.DetectionResult {
	if r.state == nil {
		return nil
	}
	return r.state.LastDetection
}

// PrimaryCam returns the primary depth camera.
func (r *Robot) PrimaryCam() camera.Camera {
	return r.primaryCam
}

// resetState clears all peeling state for the next cycle.
func (r *Robot) resetState() {
	r.state = &PeelingState{
		ApplesProcessed: r.state.ApplesProcessed,
	}
}
