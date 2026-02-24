package applepose

import "errors"

var (
	// ErrTooFewPoints is returned when a point cloud has insufficient points for an operation.
	ErrTooFewPoints = errors.New("too few points for operation")

	// ErrNoSphereFound is returned when RANSAC fails to fit a sphere to the given points.
	ErrNoSphereFound = errors.New("no valid sphere found")

	// ErrRadiusOutOfRange is returned when a fitted sphere radius falls outside the expected range.
	ErrRadiusOutOfRange = errors.New("sphere radius outside expected range")

	// ErrLowInlierFraction is returned when too few points agree with the fitted sphere model.
	ErrLowInlierFraction = errors.New("inlier fraction below threshold")

	// ErrNilPointCloud is returned when a nil point cloud is passed.
	ErrNilPointCloud = errors.New("point cloud is nil")

	// ErrSingularMatrix is returned when a linear system cannot be solved.
	ErrSingularMatrix = errors.New("singular matrix in sphere fitting")

	// ErrNoAppleDetected is returned when no valid apple clusters are found.
	ErrNoAppleDetected = errors.New("no apple detected in point cloud")
)
