package applepose

import (
	"math"

	"github.com/golang/geo/r3"
	"gonum.org/v1/gonum/mat"

	"go.viam.com/rdk/pointcloud"
)

// curvatureResult holds local curvature and normal information for a point.
type curvatureResult struct {
	Curvature float64    // Ratio of smallest eigenvalue to sum; higher = more curved
	Normal    r3.Vector  // Surface normal (eigenvector of smallest eigenvalue)
}

// estimateLocalCurvature computes PCA-based curvature for a single point using its K nearest neighbors.
func estimateLocalCurvature(kd *pointcloud.KDTree, point r3.Vector, k int) curvatureResult {
	neighbors := kd.KNearestNeighbors(point, k, true)
	if len(neighbors) < 3 {
		return curvatureResult{}
	}

	// Compute centroid of neighbors.
	var cx, cy, cz float64
	for _, nb := range neighbors {
		cx += nb.P.X
		cy += nb.P.Y
		cz += nb.P.Z
	}
	n := float64(len(neighbors))
	cx /= n
	cy /= n
	cz /= n

	// Build covariance matrix.
	var cov [9]float64 // 3x3 row-major
	for _, nb := range neighbors {
		dx := nb.P.X - cx
		dy := nb.P.Y - cy
		dz := nb.P.Z - cz
		cov[0] += dx * dx
		cov[1] += dx * dy
		cov[2] += dx * dz
		cov[3] += dy * dx
		cov[4] += dy * dy
		cov[5] += dy * dz
		cov[6] += dz * dx
		cov[7] += dz * dy
		cov[8] += dz * dz
	}
	for i := range cov {
		cov[i] /= n
	}

	covMat := mat.NewSymDense(3, []float64{
		cov[0], cov[1], cov[2],
		cov[3], cov[4], cov[5],
		cov[6], cov[7], cov[8],
	})

	var eigen mat.EigenSym
	ok := eigen.Factorize(covMat, true)
	if !ok {
		return curvatureResult{}
	}

	vals := eigen.Values(nil)
	var vecs mat.Dense
	eigen.VectorsTo(&vecs)

	// Eigenvalues are in ascending order.
	// Smallest eigenvalue corresponds to the surface normal direction.
	smallest := vals[0]
	sum := vals[0] + vals[1] + vals[2]

	var curvature float64
	if sum > 1e-15 {
		curvature = smallest / sum
	}

	// Normal is the eigenvector corresponding to smallest eigenvalue (column 0).
	normal := r3.Vector{
		X: vecs.At(0, 0),
		Y: vecs.At(1, 0),
		Z: vecs.At(2, 0),
	}

	return curvatureResult{
		Curvature: curvature,
		Normal:    normal,
	}
}

// computeSphereDeviation computes signed distance from the sphere surface for each point.
// Positive = protruding, negative = indented.
func computeSphereDeviation(points []r3.Vector, center r3.Vector, radius float64) []float64 {
	deviations := make([]float64, len(points))
	for i, pt := range points {
		deviations[i] = pt.Sub(center).Norm() - radius
	}
	return deviations
}

// orientNormal ensures the normal points outward from the sphere center.
func orientNormal(normal, point, sphereCenter r3.Vector) r3.Vector {
	outward := point.Sub(sphereCenter)
	if normal.Dot(outward) < 0 {
		return normal.Mul(-1)
	}
	return normal
}

// curvatureIsHigh returns true if the curvature significantly exceeds the expected sphere curvature.
func curvatureIsHigh(curv float64, radius float64) bool {
	// For a sphere, the expected curvature ratio depends on the neighborhood size
	// relative to the radius. High local curvature indicates a feature.
	// Empirically, curvature > 0.1 indicates a non-spherical region.
	expectedCurv := math.Min(0.05, 1.0/(radius*radius))
	_ = expectedCurv
	return curv > 0.1
}
