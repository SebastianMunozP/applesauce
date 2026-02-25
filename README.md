# Applesauce

Autonomous apple peeling robot. Connects to a Viam robot over WebRTC, runs a
loop that detects apples in a bowl, grasps them, identifies stem/calyx
features, peels them on a mechanical peeler, and resets for the next apple.

## CLI

Run individual pipeline stages for testing and development:

```sh
go run ./cli -creds /path/to/creds.json -step <step>
```

Available steps:

| Step       | Description                                                             |
|------------|-------------------------------------------------------------------------|
| `watch`    | Move to viewing position, detect bowl, print apple positions, visualize |
| `grasp`    | Multi-angle scan, select apple, attempt grasp                           |
| `identify` | Detect stem/calyx features, regrasp if needed                           |
| `peel`     | Orient apple and place on peeler spikes                                 |
| `crank`    | Drive the peeling spiral                                                |
| `remove`   | Pull peeled apple off core and deposit                                  |
| `reset`    | Full machine reset (core removal, lever, retract, deposit)              |
| `retract`  | Press lever, retract spikes, return secondary arm (no core handling)    |

The `watch` step prints each detected apple's center position, radius, and
visibility percentage, then visualizes the point cloud and fitted spheres in
[motion-tools](https://github.com/viam-labs/motion-tools). If motion-tools is
not running, visualization is skipped gracefully.

## Credentials

Both the autonomous loop and the CLI require a `-creds` flag pointing to a JSON
file with the robot's connection details:

```json
{
  "address": "robot-main.abcdef1234.viam.cloud",
  "entity_id": "abcdef12-3456-7890-abcd-ef1234567890",
  "api_key": "your-api-key-here"
}
```

| Field       | Description                                               |
|-------------|-----------------------------------------------------------|
| `address`   | Robot address from the Viam app (Connect tab)             |
| `entity_id` | API key entity ID (the UUID shown when creating the key)  |
| `api_key`   | API key secret                                            |

Credential files are gitignored (`creds.json`, `*-creds.json`).

## Building

```sh
go build .           # library (type-checks the package)
go build ./main/     # autonomous loop binary
go build ./cli/      # step-by-step CLI
go vet ./...         # lint
```

Requires Go 1.25+ and the Viam RDK checked out alongside this repo (used via
`replace` directive in `go.mod`):

```
replace go.viam.com/rdk => ../rdk
```

## Running the Autonomous Loop

```sh
go run ./main -creds /path/to/creds.json
```

Connects to the robot and runs the full Watch-to-Reset cycle in a loop until
interrupted with Ctrl-C.

## Hardware

All components are required by `NewRobot` and must be configured in the Viam
robot config:

| Component         | Name               | Description                          |
|-------------------|--------------------|--------------------------------------|
| Primary arm       | `xarm7`            | 7-DOF arm for grasping and placement |
| Secondary arm     | `secondary-arm`    | 6-DOF arm for pressing release lever |
| Peeling arm       | `peeling-arm`      | 6-DOF arm for driving the crank      |
| Apple gripper     | `arm_mount`        | Gripper on the primary arm           |
| Peeling gripper   | `peeling-gripper`  | Grips the crank handle               |
| Primary camera    | `primary-cam`      | Depth camera viewing the bowl        |
| Secondary camera  | `secondary-cam`    | Depth camera for feature analysis    |
| Motion service    | `builtin`          | Viam motion planning service         |

## Pipeline

The main loop (`Run`) cycles through seven stages. Each stage is a function
with signature `func(context.Context, *Robot) error`. If any stage fails, the
full cycle retries.

```
Watch -> Grasp -> IdentifyFeatures -> Peel -> Crank -> RemoveApple -> ResetMachine
```

### Watch

Moves both arms to their viewing positions and polls `primary-cam` for a point
cloud every 3 seconds. Runs the apple pose detector on each frame. Returns when
a bowl of apples is detected.

### Grasp

1. **Multi-angle scan** -- moves `xarm7` through `PrimaryBowlScanAngles` and
   merges detections with `DetectWithHistory`. Falls back to the last Watch
   detection if no scan angles are configured.
2. **Apple selection** -- scores apples by Z height and visible fraction
   (highest and most visible = most accessible).
3. **Obstacle avoidance** -- other apples become sphere obstacles with a 5 mm
   safety margin in the `WorldState`.
4. **Grasp loop** (3 attempts) -- move 200 mm above, open gripper, linear
   descent to 20 mm above center, grab, retreat, verify with
   `IsHoldingSomething`.

### IdentifyFeatures

Extracts stem and calyx from the detected apple's feature list (confidence
threshold 0.6). If either is missing, analyzes with the secondary camera. If
still missing, regrasps up to 4 times (rotating the gripper 45 deg each time)
and re-analyzes. Computes a feature vector (calyx-to-stem unit vector) when
both are found.

### Peel

Converts the feature vector into an arm orientation that aligns the apple axis
with the peeler spike axis (+X). Moves above the peeler, descends linearly into
the jaws, pushes onto the spikes, releases the gripper, and retreats.

### Crank

Drives the peeling arm through a spiral path using the motion service with
linear constraints for each 1 mm step:

- Radius: 59.5 mm
- Pitch: 6 mm per revolution in -X
- Revolutions: 23
- Step size: ~1 mm arc length (~374 steps/rev, ~8,579 total)

### RemoveApple

Approaches the peeled apple on the peeler core from -X, grasps it, pulls it off
linearly, moves to the peeled apple bowl, and releases.

### ResetMachine

1. Primary arm grabs the core from the peeler.
2. Secondary arm presses the release lever.
3. Peeling arm retracts the spikes.
4. Secondary arm returns to viewing position.
5. Primary arm deposits the core in the waste bin and returns to viewing
   position.
6. Cycle state is reset.

There is also a standalone `Retract` function that runs only steps 2-4 (lever,
spikes, secondary arm return) without touching the core.

## Motion Helpers

All arm movement goes through three helpers on `Robot`:

- **`moveLinear`** -- `motion.Move` with a 1 mm / 2 deg linear constraint. Used
  for precise paths (descending to apple, pushing onto spikes, crank steps).
- **`moveFree`** -- `motion.Move` with no path constraints. Used for
  repositioning between stations.
- **`moveToJoints`** -- `motion.Move` with a joint-space goal. Used for
  recorded positions. Returns an error on nil joints (stub guard).

## File Layout

```
main/main.go            Entry point: connect to robot, run main loop
robot.go                Robot struct, PeelingState, NewRobot, motion helpers
run.go                  Run loop and runCycle orchestration
positions.go            Hardcoded joint positions and world-frame pose stubs
watch.go                Watch stage
grasp.go                Grasp stage, multi-angle scan, apple selection
features.go             IdentifyFeatures stage, regrasp logic
peel.go                 Peel stage, feature vector to orientation math
crank.go                Crank stage, spiral path generation
cleanup.go              RemoveApple, ResetMachine, Retract, and sub-helpers

cli/main.go             Step-by-step CLI for manual operation
internal/creds/         Robot credential loading from JSON

apple_pose/             Apple detection library (see below)
  config.go             Config structs with defaults
  types.go              Apple, Bowl, DetectionResult, features, suggestions
  errors.go             Sentinel errors
  detector.go           Detector: Detect, DetectWithHistory, AnalyzeSingleApple
  scene.go              Point cloud preprocessing, clustering, bowl heuristic
  sphere_fit.go         RANSAC sphere fitting + least-squares refinement
  curvature.go          PCA-based local curvature estimation
  color.go              RGB-to-HSV, color classification
  stem_calyx.go         Stem/calyx detection via anomaly clustering
  view_planning.go      Suggested camera views and flip actions
  tracking.go           Cross-view apple matching and feature merging
  cmd/test_apples/      Live camera test binary

cmd/record_positions/   Read current arm joint positions from the live robot
```

## Apple Pose Detection Library

The `apple_pose` package (`github.com/biotinker/applesauce/apple_pose`) is a
standalone library for detecting apples in 3D point clouds.

### Detection Pipeline

1. **Preprocessing** -- depth filter, statistical outlier removal, plane
   segmentation (soft fail), radius-based clustering.
2. **Sphere fitting** -- iterative RANSAC with 4-point sphere solver and
   algebraic least-squares refinement. Rejects fits below a minimum visible
   fraction.
3. **Bowl detection** -- heuristic: two or more sphere centers within 120 mm.
4. **Stem/calyx detection** -- sphere deviation analysis + PCA curvature +
   HSV color scoring, clustered into anomaly regions, classified as stem
   (higher along support normal) or calyx.
5. **View planning** -- analyzes observed vs hidden hemispheres, suggests
   camera poses and physical actions (e.g. flip apple) to reveal missing
   features.
6. **Tracking** -- cross-view apple matching by center distance, feature merge
   by confidence, carry-forward of occluded apples.

### Public API

```go
// Create a detector (nil config uses defaults).
detector := applepose.NewDetector(nil)

// Single-frame detection.
result, err := detector.Detect(ctx, cloud)

// Multi-view: pass previous result to merge across frames.
result, err = detector.DetectWithHistory(ctx, cloud, prevResult)

// Analyze a single known apple (e.g. held in gripper).
apple, err := detector.AnalyzeSingleApple(ctx, cloud, center, radius)
```

`DetectionResult` contains:
- `BowlDetected` -- whether a bowl was found
- `Bowl.Apples` -- slice of detected apples, each with pose, radius, fitted
  sphere, inlier/visibility fractions, and stem/calyx features
- `SuggestedViews` -- camera poses to improve detection
- `SuggestedActions` -- physical actions (flip, reposition)

### Configuration

`applepose.DefaultConfig()` returns sensible defaults. Key tuning parameters:

| Parameter                      | Default | Description                           |
|--------------------------------|---------|---------------------------------------|
| `SphereFit.RANSACIterations`   | 2000    | RANSAC iterations per sphere          |
| `SphereFit.InlierThresholdMm`  | 4.0     | Max distance to count as inlier       |
| `SphereFit.ExpectedRadiusMinMm`| 25.0    | Minimum accepted apple radius         |
| `SphereFit.ExpectedRadiusMaxMm`| 50.0    | Maximum accepted apple radius         |
| `SphereFit.MinVisibleFraction` | 0.18    | Reject spheres with less visible area |
| `Scene.ClusteringRadiusMm`     | 15.0    | Radius for point clustering           |
| `Scene.MinClusterSize`         | 50      | Minimum points per cluster            |
| `Feature.CurvatureNeighbors`   | 15      | K for PCA curvature estimation        |

### Testing

```sh
go test ./apple_pose/ -v
```

Tests use synthetic point clouds and require no external data. To run the
real-data test, first capture a PCD file from the live camera:

```sh
go run ./apple_pose/cmd/test_apples -creds /path/to/creds.json
```

This saves a point cloud to `apple_pose/testdata/applecam.pcd`, which
`TestDetect_RealData` will pick up on subsequent test runs.

## Recording Positions

Use the position recording utility to capture current arm joint angles from the
live robot. Manually move the arms to the desired positions first, then run:

```sh
go run ./cmd/record_positions -creds /path/to/creds.json
```

This prints joint values in `[]referenceframe.Input{...}` format ready to paste
into `positions.go`.

## Stub Strategy

The codebase is designed to run with partial hardware. Each routine nil-checks
its positions/poses and either:

- Logs a warning and returns nil (stage skipped, pipeline continues), or
- Returns a descriptive error (for moves that cannot be skipped).

Currently recorded positions (in `positions.go`):
- `PrimaryViewingJoints` -- xarm7 bowl-viewing position
- `SecondaryReleaseLeverApproach` -- secondary arm near the release lever
- `PeelingCrankGraspJoints` -- peeling arm at the crank handle
- `CrankCenter` -- world-frame center of the crank circle

Stubs needing recording or measurement (grep for `STUB`):
- `SecondaryViewingJoints`, `PrimaryBowlScanAngles` (joint positions)
- `PeelerDevicePose`, `PeelerJawsPose`, `PeeledAppleBowlPose`, `WasteBinPose`,
  `SecondaryReleaseLeverPressPose`, `PeelerCorePose`,
  `PeelerSpikeRetractPose` (world-frame poses)
