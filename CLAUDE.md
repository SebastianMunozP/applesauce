# Applesauce — Autonomous Apple Peeling Robot

Module: `github.com/biotinker/applesauce` | Package: `main`

Connects to a Viam robot over WebRTC, runs an autonomous loop that detects apples in a bowl, grasps them, identifies stem/calyx features, peels them on a mechanical peeler, and resets for the next apple.

## Architecture

The main loop (`Run` in `main.go`) cycles through seven stages. Each stage is a standalone function with signature `func(context.Context, *Robot) error`. If any stage fails, the full cycle retries.

```
Watch → Grasp → IdentifyFeatures → Peel → Crank → RemoveApple → ResetMachine
```

All state for a single cycle lives in `PeelingState`; it's reset between cycles.

## File Layout

```
main.go         Entry point: connect to robot, create Robot, run main loop
robot.go        Robot struct, PeelingState, NewRobot(), motion helpers
positions.go    Hardcoded joint positions (recorded 2026-02-24), stub world-frame poses
watch.go        Watch(): arms to viewing positions, poll camera for bowl
grasp.go        Grasp(): multi-angle scan, apple selection, approach/grab/verify
features.go     IdentifyFeatures(), Regrasp(): stem/calyx detection, re-grasping
peel.go         Peel(): orient apple, lower onto peeler spikes
crank.go        Crank(): spiral path (59.5mm radius, 6mm pitch, 23 revs, 1mm steps)
cleanup.go      RemoveApple(), ResetMachine(): pull apple off, press lever, deposit waste
apple_pose/     Detection library (separate CLAUDE.md in that directory)
cmd/record_positions/  Utility to read current arm joint positions from the live robot
```

## Robot Struct (`robot.go`)

Holds all hardware references, the motion service, the `applepose.Detector`, and cycle state.

**All resources are required** (NewRobot fails without any of these):
- `xarm7` — primary arm (7-DOF), does grasping, peeling placement, core removal
- `secondary-arm` — secondary arm (6-DOF), presses release lever
- `peeling-arm` — drives the crank for peeling
- `arm_mount` — gripper on the primary arm
- `peeling-gripper` — grips the crank handle
- `primary-cam`, `secondary-cam` — depth cameras
- `builtin` — motion service

## Motion Helpers

All arm movement goes through three helpers on `Robot`:

- **`moveLinear(ctx, componentName, destPose, worldState)`** — `motion.Move` with `LinearConstraint{1mm, 2deg}`. For precise linear paths (descending to apple, pushing onto spikes).
- **`moveFree(ctx, componentName, destPose, worldState)`** — `motion.Move` with no constraints. For repositioning between stations.
- **`moveToJoints(ctx, componentName, joints)`** — `motion.Move` with joint goal in Extra map. Returns error on nil joints (stub guard). Used for recorded positions.

The crank routine calls `moveLinear` for each 1mm step.

## Positions (`positions.go`)

**Hardcoded joint positions** (recorded from the live robot on 2026-02-24):
- `PrimaryViewingJoints` — xarm7 bowl-viewing position (7 values, radians)
- `SecondaryReleaseLeverApproach` — secondary arm near the release lever (6 values)
- `PeelingCrankGraspJoints` — peeling arm at the crank handle (6 values)

**Stub joint positions** (nil, need recording):
- `SecondaryViewingJoints` — secondary arm looking at the gripper
- `PrimaryBowlScanAngles` — multiple xarm7 viewpoints for multi-angle scanning

**Stub world-frame poses** (nil, need measuring):
- `PeelerDevicePose`, `PeelerJawsPose` — peeler machine locations
- `PeeledAppleBowlPose`, `WasteBinPose` — deposit locations
- `SecondaryReleaseLeverPressPose` — where to push the lever down
- `PeelerCorePose` — where the cored apple sits after peeling
- `PeelerSpikeRetractPose` — fully retracted spike position
- `CrankCenter` — center of the crank circle (r3.Vector)

Every routine checks for nil poses/joints before use and either logs a warning and skips (for optional stages) or returns an error (for required moves).

## Stage Details

### Watch (`watch.go`)
Moves arms to viewing positions, polls `primaryCam.NextPointCloud` every 3s, runs `detector.Detect`, returns when `BowlDetected` is true. If no camera is configured, returns immediately (allows testing downstream stages).

### Grasp (`grasp.go`)
1. Multi-angle scan: moves xarm7 through `PrimaryBowlScanAngles`, calls `DetectWithHistory` to merge. Falls back to last Watch detection if no scan angles configured.
2. Apple selection: scores by Z height + visible fraction (highest & most visible = most accessible).
3. Obstacle avoidance: other apples become sphere obstacles with 5mm safety margin in `WorldState`.
4. Grasp loop (3 attempts): `moveFree` 200mm above → open gripper → `moveLinear` to 20mm above center → `Grab` → `moveLinear` retreat → verify with `IsHoldingSomething`.

### IdentifyFeatures (`features.go`)
Extracts stem/calyx from `TargetApple.Features` (confidence threshold 0.6). If missing, analyzes with secondary camera via `AnalyzeSingleApple`. If still missing, regrasps up to 4 times (rotating gripper 45deg each time) and re-analyzes. Computes feature vector (calyx→stem unit vector) when both are found.

### Peel (`peel.go`)
Converts the feature vector into an arm orientation aligning the apple axis with the peeler spike axis (+X). Moves above peeler → linear descent into jaws → linear push onto spikes → open gripper → retreat. Skips if peeler poses aren't configured.

### Crank (`crank.go`)
Spiral: 59.5mm radius, 6mm pitch/rev in -X, 23 revolutions. Circle in YZ plane. ~374 steps/rev at 1mm arc. Uses `moveLinear` per step. Logs each revolution.

### RemoveApple (`cleanup.go`)
Approaches peeled apple on peeler from -X → open → linear approach → grab → linear pull in -X → moveFree to peeled bowl → open.

### ResetMachine (`cleanup.go`)
1. Primary arm grabs core from peeler
2. Secondary arm presses release lever (moveToJoints → moveFree above → moveLinear press → moveLinear retract)
3. Peeling arm retracts spikes
4. Secondary arm returns to viewing position
5. Primary arm deposits core in waste bin, returns to viewing position
6. State is reset for the next cycle

## Building

```sh
go build .        # builds the main binary
go vet .          # lint
go build ./...    # includes apple_pose (but cmd/test_apples has a stale import path)
```

Uses `replace go.viam.com/rdk => ../rdk` in go.mod — the RDK must be checked out alongside this repo.

## Recording New Positions

Use `cmd/record_positions/` to read current joint positions from the live robot:

```sh
go run ./cmd/record_positions/
```

Then paste the output into `positions.go`.

## Stub Strategy

The codebase is designed to run with partial hardware. Each routine nil-checks its resources/positions and either:
- Logs a warning and returns nil (stage skipped, pipeline continues)
- Returns a descriptive error (for moves that can't be skipped)

This lets you test stages in isolation as hardware comes online. Grep for `STUB` to find all placeholders.
