package applepose

import (
	"math"
	"math/rand"

	"github.com/golang/geo/r3"
	"gonum.org/v1/gonum/mat"

	"go.viam.com/rdk/pointcloud"
)

// FitSphere fits a sphere to a point cloud using RANSAC followed by algebraic least-squares refinement.
func FitSphere(cloud pointcloud.PointCloud, cfg SphereFitConfig) (*SphereFitResult, error) {
	if cloud == nil {
		return nil, ErrNilPointCloud
	}
	if cloud.Size() < 4 {
		return nil, ErrTooFewPoints
	}

	points := pointcloud.CloudToPoints(cloud)
	n := len(points)

	var bestCenter r3.Vector
	var bestRadius float64
	bestScore := math.MaxFloat64
	threshSq := cfg.InlierThresholdMm * cfg.InlierThresholdMm

	//nolint:gosec
	rng := rand.New(rand.NewSource(42))

	for iter := 0; iter < cfg.RANSACIterations; iter++ {
		// Sample 4 random points.
		idx := sampleFourDistinct(rng, n)
		p := [4]r3.Vector{points[idx[0]], points[idx[1]], points[idx[2]], points[idx[3]]}

		// Check for coplanarity: volume of tetrahedron should be non-negligible.
		v1 := p[1].Sub(p[0])
		v2 := p[2].Sub(p[0])
		v3 := p[3].Sub(p[0])
		vol := math.Abs(v1.Dot(v2.Cross(v3)))
		if vol < 1e-6 {
			continue
		}

		center, radius, ok := sphereFrom4Points(p)
		if !ok || radius <= 0 {
			continue
		}

		// Check radius range.
		if radius < cfg.ExpectedRadiusMinMm || radius > cfg.ExpectedRadiusMaxMm {
			continue
		}

		// MSAC scoring: sum of min(dist², threshold²) per point.
		// Unlike inlier counting, this prefers spheres where points fit tightly
		// rather than spheres that merely capture more points (which biases toward
		// larger radii when adjacent structures are present).
		var score float64
		for _, pt := range points {
			d := pt.Sub(center).Norm() - radius
			dSq := d * d
			if dSq < threshSq {
				score += dSq
			} else {
				score += threshSq
			}
		}

		if score < bestScore {
			bestScore = score
			bestCenter = center
			bestRadius = radius
		}
	}

	if bestScore == math.MaxFloat64 {
		return nil, ErrNoSphereFound
	}

	// Collect inlier points.
	inlierCloud := pointcloud.NewBasicEmpty()
	var inlierPoints []r3.Vector
	cloud.Iterate(0, 0, func(pt r3.Vector, d pointcloud.Data) bool {
		dist := math.Abs(pt.Sub(bestCenter).Norm() - bestRadius)
		if dist <= cfg.InlierThresholdMm {
			inlierPoints = append(inlierPoints, pt)
			//nolint:errcheck
			inlierCloud.Set(pt, d)
		}
		return true
	})

	// Iterative inlier re-selection: compute radius as mean distance to center,
	// then re-collect inliers using the new radius. This tightens the fit because
	// an initially-too-large RANSAC radius includes outlying points that inflate
	// the mean; shrinking the radius drops those points, further tightening the mean.
	// Center is kept fixed (RANSAC center from 4 exact points is well-constrained).
	for refineIter := 0; refineIter < 3; refineIter++ {
		if len(inlierPoints) < 4 {
			break
		}
		var sumDist float64
		for _, pt := range inlierPoints {
			sumDist += pt.Sub(bestCenter).Norm()
		}
		meanRadius := sumDist / float64(len(inlierPoints))
		if meanRadius < cfg.ExpectedRadiusMinMm || meanRadius > cfg.ExpectedRadiusMaxMm {
			break
		}
		bestRadius = meanRadius

		// Re-collect inliers using updated radius.
		newInlierCloud := pointcloud.NewBasicEmpty()
		var newInlierPoints []r3.Vector
		cloud.Iterate(0, 0, func(pt r3.Vector, d pointcloud.Data) bool {
			dist := math.Abs(pt.Sub(bestCenter).Norm() - bestRadius)
			if dist <= cfg.InlierThresholdMm {
				newInlierPoints = append(newInlierPoints, pt)
				//nolint:errcheck
				newInlierCloud.Set(pt, d)
			}
			return true
		})
		inlierPoints = newInlierPoints
		inlierCloud = newInlierCloud
	}

	// Compute RMS residual.
	var sumSqErr float64
	for _, pt := range inlierPoints {
		diff := pt.Sub(bestCenter).Norm() - bestRadius
		sumSqErr += diff * diff
	}
	rmsResidual := math.Sqrt(sumSqErr / float64(len(inlierPoints)))

	inlierFraction := float64(len(inlierPoints)) / float64(n)

	// Estimate visible fraction.
	visFrac := estimateVisibleFraction(inlierPoints, bestCenter, bestRadius)

	return &SphereFitResult{
		Center:          bestCenter,
		Radius:          bestRadius,
		RMSResidual:     rmsResidual,
		InlierFraction:  inlierFraction,
		InlierCloud:     inlierCloud,
		VisibleFraction: visFrac,
	}, nil
}

// sphereFrom4Points solves for the sphere passing through 4 non-coplanar points.
// Subtracts the first equation from the other 3 to get a 3x3 linear system.
func sphereFrom4Points(p [4]r3.Vector) (center r3.Vector, radius float64, ok bool) {
	// Sphere equation: (x-cx)^2 + (y-cy)^2 + (z-cz)^2 = r^2
	// Expanded: x^2+y^2+z^2 - 2*cx*x - 2*cy*y - 2*cz*z + (cx^2+cy^2+cz^2-r^2) = 0
	// For point i: |pi|^2 - 2*cx*pi.x - 2*cy*pi.y - 2*cz*pi.z + K = 0
	// Subtract equation 0 from equations 1,2,3:
	// 2*(p[i].x - p[0].x)*cx + 2*(p[i].y - p[0].y)*cy + 2*(p[i].z - p[0].z)*cz = |p[i]|^2 - |p[0]|^2

	sq := func(v r3.Vector) float64 {
		return v.X*v.X + v.Y*v.Y + v.Z*v.Z
	}
	sq0 := sq(p[0])

	a := mat.NewDense(3, 3, nil)
	b := mat.NewVecDense(3, nil)
	for i := 0; i < 3; i++ {
		a.Set(i, 0, 2*(p[i+1].X-p[0].X))
		a.Set(i, 1, 2*(p[i+1].Y-p[0].Y))
		a.Set(i, 2, 2*(p[i+1].Z-p[0].Z))
		b.SetVec(i, sq(p[i+1])-sq0)
	}

	var x mat.VecDense
	err := x.SolveVec(a, b)
	if err != nil {
		return r3.Vector{}, 0, false
	}

	center = r3.Vector{X: x.AtVec(0), Y: x.AtVec(1), Z: x.AtVec(2)}
	radius = center.Sub(p[0]).Norm()
	return center, radius, true
}

// boundedGeometricRefine refines a sphere by minimizing sum((|p-c| - r)²)
// with center movement clamped to maxShift from the initial center.
func boundedGeometricRefine(points []r3.Vector, initCenter r3.Vector, initRadius, maxShift float64) (r3.Vector, float64) {
	center := initCenter
	radius := initRadius
	n := float64(len(points))

	for iter := 0; iter < 30; iter++ {
		var gradCx, gradCy, gradCz float64
		var sumDist float64

		for _, pt := range points {
			d := pt.Sub(center)
			dist := d.Norm()
			if dist < 1e-9 {
				continue
			}
			residual := dist - radius
			gradCx += residual * (-d.X / dist)
			gradCy += residual * (-d.Y / dist)
			gradCz += residual * (-d.Z / dist)
			sumDist += dist
		}

		// Update radius to mean distance (closed-form optimum for fixed center).
		radius = sumDist / n

		// Gradient step for center.
		scale := 1.0 / n
		center.X -= scale * gradCx
		center.Y -= scale * gradCy
		center.Z -= scale * gradCz

		// Clamp center movement to maxShift from initial center.
		shift := center.Sub(initCenter)
		shiftDist := shift.Norm()
		if shiftDist > maxShift {
			shift = shift.Mul(maxShift / shiftDist)
			center = initCenter.Add(shift)
		}
	}

	return center, radius
}

// geometricSphereFit refines a sphere fit by minimizing the geometric objective:
// sum((|p_i - center| - radius)²) over center and radius.
// This is unbiased for partial sphere data, unlike the algebraic objective.
func geometricSphereFit(points []r3.Vector, initCenter r3.Vector, initRadius float64) (r3.Vector, float64) {
	center := initCenter
	radius := initRadius
	n := float64(len(points))

	for iter := 0; iter < 50; iter++ {
		var gradCx, gradCy, gradCz float64
		var sumDist float64

		for _, pt := range points {
			d := pt.Sub(center)
			dist := d.Norm()
			if dist < 1e-9 {
				continue
			}
			residual := dist - radius
			// Gradient of residual² w.r.t center components:
			// d/dc_x (dist - r)² = 2*(dist - r) * d(dist)/d(c_x)
			// d(dist)/d(c_x) = -(p_x - c_x) / dist
			gradCx += residual * (-d.X / dist)
			gradCy += residual * (-d.Y / dist)
			gradCz += residual * (-d.Z / dist)
			sumDist += dist
		}

		// Update radius to mean distance (closed-form optimum for fixed center).
		radius = sumDist / n

		// Gradient step for center.
		scale := 1.0 / n
		center.X -= scale * gradCx
		center.Y -= scale * gradCy
		center.Z -= scale * gradCz
	}

	return center, radius
}

// algebraicSphereFit refines a sphere fit using algebraic least-squares.
// Linearizes: x*A + y*B + z*C + D = -(x^2+y^2+z^2), where center = (-A/2, -B/2, -C/2).
func algebraicSphereFit(points []r3.Vector) (center r3.Vector, radius float64, ok bool) {
	n := len(points)
	if n < 4 {
		return r3.Vector{}, 0, false
	}

	// Build Nx4 matrix and Nx1 RHS.
	A := mat.NewDense(n, 4, nil)
	b := mat.NewVecDense(n, nil)
	for i, pt := range points {
		A.Set(i, 0, pt.X)
		A.Set(i, 1, pt.Y)
		A.Set(i, 2, pt.Z)
		A.Set(i, 3, 1.0)
		b.SetVec(i, -(pt.X*pt.X + pt.Y*pt.Y + pt.Z*pt.Z))
	}

	// Solve via QR decomposition: A * x = b.
	var qr mat.QR
	qr.Factorize(A)
	var x mat.VecDense
	err := qr.SolveVecTo(&x, false, b)
	if err != nil {
		return r3.Vector{}, 0, false
	}

	aCoeff := x.AtVec(0)
	bCoeff := x.AtVec(1)
	cCoeff := x.AtVec(2)
	dCoeff := x.AtVec(3)

	center = r3.Vector{
		X: -aCoeff / 2,
		Y: -bCoeff / 2,
		Z: -cCoeff / 2,
	}

	rSquared := aCoeff*aCoeff/4 + bCoeff*bCoeff/4 + cCoeff*cCoeff/4 - dCoeff
	if rSquared <= 0 {
		return r3.Vector{}, 0, false
	}
	radius = math.Sqrt(rSquared)

	return center, radius, true
}

// estimateVisibleFraction estimates what fraction of the sphere surface is observed.
// Uses ~200 directions on a golden-angle spiral and checks if any point lies near each direction.
func estimateVisibleFraction(points []r3.Vector, center r3.Vector, radius float64) float64 {
	const numDirections = 200
	goldenAngle := math.Pi * (3 - math.Sqrt(5))

	covered := 0
	// Angular threshold: a point covers a direction if within this angle.
	cosThreshold := math.Cos(math.Pi / (math.Sqrt(float64(numDirections)) * 1.5))

	// Precompute point directions from center.
	dirs := make([]r3.Vector, 0, len(points))
	for _, pt := range points {
		d := pt.Sub(center)
		n := d.Norm()
		if n > 1e-9 {
			dirs = append(dirs, d.Mul(1.0/n))
		}
	}

	for i := 0; i < numDirections; i++ {
		// Golden spiral distribution on unit sphere.
		t := float64(i) / float64(numDirections-1)
		phi := math.Acos(1 - 2*t)
		theta := goldenAngle * float64(i)

		dx := math.Sin(phi) * math.Cos(theta)
		dy := math.Sin(phi) * math.Sin(theta)
		dz := math.Cos(phi)

		// Check if any point direction is close to this direction.
		for _, dir := range dirs {
			cosAngle := dir.X*dx + dir.Y*dy + dir.Z*dz
			if cosAngle >= cosThreshold {
				covered++
				break
			}
		}
	}

	return float64(covered) / float64(numDirections)
}

func sampleFourDistinct(rng *rand.Rand, n int) [4]int {
	var idx [4]int
	for i := 0; i < 4; i++ {
		for {
			idx[i] = rng.Intn(n)
			unique := true
			for j := 0; j < i; j++ {
				if idx[i] == idx[j] {
					unique = false
					break
				}
			}
			if unique {
				break
			}
		}
	}
	return idx
}
