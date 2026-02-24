# Apple Pose Estimation Library

Module: `github.com/biotinker/applesauce/apple-pose` | Package: `applepose`

## What This Is

A Go library that takes a color 3D pointcloud (from a depth camera), detects apples, fits spheres to them, estimates stem/calyx poses, and suggests additional camera viewpoints or physical actions (like flipping) to reveal hidden features. Integrates with Viam's RDK for pointcloud and spatial math types.

## Dependencies

- `go.viam.com/rdk` — pointcloud, spatialmath, segmentation APIs. Linked via `replace ../rdk` in go.mod.
- `gonum.org/v1/gonum` — linear algebra (QR decomposition, eigenvalue decomposition).
- `github.com/golang/geo/r3` — 3D vector math.

## File Layout

```
config.go          Config structs with defaults (DefaultConfig())
types.go           Apple, FeatureDetection, BowlOfApples, DetectionResult, SuggestedView, ActionSuggestion, SphereFitResult
errors.go          Sentinel errors (ErrTooFewPoints, ErrNoSphereFound, etc.)
detector.go        Detector struct — Detect(), DetectWithHistory(), AnalyzeSingleApple()
scene.go           preprocessScene(): depth filter, color filter, outlier filter, plane segmentation (soft fail), radius-based clustering, bowl heuristic
sphere_fit.go      FitSphere(): RANSAC 4-point sphere solver + algebraic least-squares refinement + visible fraction estimation
curvature.go       PCA-based local curvature estimation, sphere deviation computation
color.go           RGB-to-HSV, color classification (apple skin vs stem-like vs calyx-like), isAppleColored() filter
stem_calyx.go      detectStemCalyx(): anomaly detection + clustering + scoring + classification, apple orientation
view_planning.go   PlanViews(): hidden hemisphere analysis, candidate camera poses, flip action suggestions
tracking.go        mergeDetections(): cross-view apple matching by center distance, feature merging, carry-forward
cmd/test_apples/   Integration test binary that connects to a live Viam robot camera
```

## Detection Pipeline (inside Detect())

1. **Preprocessing** (`scene.go`): depth filter (Z ≤ MaxDepthMm) → optional color filter → `StatisticalOutlierFilter` → `SegmentPlaneWRTGround` (soft fail) → radius-based clustering
2. **Iterative sphere fitting** (`detector.go` + `sphere_fit.go`): for each cluster, iteratively fit spheres via RANSAC, removing inliers after each fit. Rejects fits with VisibleFraction < MinVisibleFraction. Stops after MaxSpheresPerCluster or when inlier count < MinInliersPerSphere.
3. **Bowl detection** (`scene.go`): heuristic — ≥2 sphere centers within 120mm
4. **Stem/calyx** (`stem_calyx.go`): sphere deviation + PCA curvature + HSV color → anomaly clustering → scoring → classification
5. **View planning** (`view_planning.go`): observed/hidden hemisphere → camera poses via Rodrigues rotation or flip suggestions
6. **Tracking** (`tracking.go`): center-distance matching, feature merge (keep higher confidence), carry-forward occluded apples

## Public API

```go
func NewDetector(cfg *Config) *Detector
func DefaultConfig() Config
func (d *Detector) Detect(ctx context.Context, cloud pointcloud.PointCloud) (*DetectionResult, error)
func (d *Detector) DetectWithHistory(ctx context.Context, cloud pointcloud.PointCloud, prev *DetectionResult) (*DetectionResult, error)
func (d *Detector) AnalyzeSingleApple(ctx context.Context, cloud pointcloud.PointCloud, center r3.Vector, radius float64) (*Apple, error)
func FitSphere(cloud pointcloud.PointCloud, cfg SphereFitConfig) (*SphereFitResult, error)
func PlanViews(result *DetectionResult, cfg ViewPlanningConfig) ([]SuggestedView, []ActionSuggestion)
```

## Tests

Run: `go test ./... -v`

All tests use synthetic pointclouds (no external data needed). 20 tests covering:
- Sphere fitting: perfect, noisy, partial, too-few-points, out-of-range radius
- Scene: two-cluster separation, small-cluster pruning, plane+cluster segmentation, bowl detection
- Stem/calyx: synthetic apple with indentations, perfect sphere (no false positives), orientation, HSV conversion
- View planning: missing features, hidden-below-surface flip suggestion, rotation math, observed direction
- Tracking: distance matching, carry-forward, feature merge by confidence

## Live Testing

`cmd/test_apples/main.go` connects to a Viam robot camera (`applecam`) via WebRTC and runs the full pipeline. Run with `go run ./cmd/test_apples/ -creds /path/to/armcam-creds.json`.

## Known Issues / Future Work

- **Ground normal**: `DefaultConfig()` sets `GroundNormal: {0,0,1}` (Z-up), but depth cameras typically have Z as forward/depth. When the camera is mounted looking down at a table, the true gravity vector must be set correctly for stem-above-calyx heuristics to work. Consider auto-detecting from the dominant plane normal.
- **Stem/calyx detection**: works on synthetic data but needs tuning for real-world apple scans. Feature detection on iteratively-fit sphere inlier clouds may need adjusted thresholds.
- **Color filter disabled by default**: the `isAppleColored()` filter (`AppleHueMax` etc.) is available but disabled because it strips the table surface before plane segmentation, causing cluster fragmentation. Enable for scenes where plane segmentation isn't useful.
- **DetectWithHistory**: the carry-forward logic uses a simple distance threshold. Could add ICP or feature-based registration for better cross-view alignment.
