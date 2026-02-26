package applesauce

import (
	"context"
	"fmt"
	"os"

	"github.com/golang/geo/r3"

	applepose "github.com/biotinker/applesauce/apple_pose"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/spatialmath"
)

// downsamplePointCloud downsamples a point cloud to approximately the target number of points.
// Returns the downsampled point cloud.
func downsamplePointCloud(r *Robot, cloud pointcloud.PointCloud, targetPoints int) pointcloud.PointCloud {
	r.logger.Infof("Point cloud has %d points, downsampling to ~%d...", cloud.Size(), targetPoints)

	downsampled := pointcloud.NewBasicEmpty()
	step := cloud.Size() / targetPoints
	if step < 1 {
		step = 1
	}
	i := 0
	cloud.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
		if i%step == 0 {
			err := downsampled.Set(p, d)
			if err != nil {
				r.logger.Warnf("Failed to add point: %v", err)
			}
		}
		i++
		return true
	})

	r.logger.Infof("Downsampled to %d points", downsampled.Size())
	return downsampled
}

// detectApples captures point clouds from all available cameras, merges them in world frame,
// filters to the bowl region, runs detection, and saves/visualizes results.
func (r *Robot) detectApples(ctx context.Context) (*applepose.DetectionResult, error) {
	if r.primaryCam == nil {
		return nil, fmt.Errorf("no primary camera available")
	}

	// Get primary camera point cloud in world frame.
	worldCloud, err := r.getCameraWorldCloud(ctx, r.primaryCam)
	if err != nil {
		return nil, fmt.Errorf("primary camera: %w", err)
	}

	// If secondary camera and viewing joints are configured, merge its cloud.
	if SecondaryViewingJoints != nil && r.secondaryCam != nil {
		secondaryCloud, err := r.getCameraWorldCloud(ctx, r.secondaryCam)
		if err != nil {
			r.logger.Warnf("Secondary camera error: %v", err)
		} else {
			before := worldCloud.Size()
			secondaryCloud.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
				if err := worldCloud.Set(p, d); err != nil {
					r.logger.Warnf("Failed to merge point: %v", err)
				}
				return true
			})
			r.logger.Infof("Merged secondary cloud (%d points) into world cloud (%d -> %d points)",
				secondaryCloud.Size(), before, worldCloud.Size())
		}
	}

	// Filter point cloud to only include points within the bowl region box.
	worldCloud, err = filterCloudToBox(worldCloud, BowlRegionBox, r)
	if err != nil {
		return nil, fmt.Errorf("bowl region filter: %w", err)
	}

	// Run detection on the merged world-frame point cloud.
	result, err := r.detector.Detect(ctx, worldCloud)
	r.logger.Info("Detection complete")
	if err != nil {
		return nil, fmt.Errorf("detection: %w", err)
	}

	// Save world-frame point clouds.
	if err := savePointClouds(r, worldCloud, result, "world"); err != nil {
		r.logger.Warnf("Failed to save world-frame point clouds: %v", err)
	}

	// Visualize the merged, filtered world-frame cloud and detection results.
	visualizeWatch(r, worldCloud, result)

	return result, nil
}

// filterCloudToBox filters a point cloud to only include points that fall within the given
// bounding box geometry. If the box is nil, the cloud is returned unmodified.
func filterCloudToBox(cloud pointcloud.PointCloud, box spatialmath.Geometry, r *Robot) (pointcloud.PointCloud, error) {
	if box == nil {
		r.logger.Warn("BowlRegionBox not configured; skipping point cloud filtering")
		return cloud, nil
	}

	octree, err := pointcloud.ToBasicOctree(cloud, 0)
	if err != nil {
		return nil, fmt.Errorf("convert to octree for filtering: %w", err)
	}

	pts := octree.PointsCollidingWith([]spatialmath.Geometry{box}, 0)

	filtered := pointcloud.NewBasicPointCloud(len(pts))
	for _, p := range pts {
		if d, ok := cloud.At(p.X, p.Y, p.Z); ok {
			if err := filtered.Set(p, d); err != nil {
				return nil, fmt.Errorf("set filtered point: %w", err)
			}
		} else {
			if err := filtered.Set(p, nil); err != nil {
				return nil, fmt.Errorf("set filtered point: %w", err)
			}
		}
	}

	r.logger.Infof("Filtered point cloud from %d to %d points using bowl region box", cloud.Size(), filtered.Size())
	return filtered, nil
}

// getCameraWorldCloud gets a point cloud from the given camera and transforms it to world frame.
func (r *Robot) getCameraWorldCloud(ctx context.Context, cam camera.Camera) (pointcloud.PointCloud, error) {
	cloud, err := cam.NextPointCloud(ctx, nil)
	if err != nil {
		return nil, fmt.Errorf("get point cloud from %s: %w", cam.Name().Name, err)
	}

	camPose, err := r.fsSvc.GetPose(ctx, cam.Name().Name, "", nil, nil)
	if err != nil {
		return nil, fmt.Errorf("get pose of %s in world frame: %w", cam.Name().Name, err)
	}

	worldCloud := pointcloud.NewBasicPointCloud(cloud.Size())
	if err := pointcloud.ApplyOffset(cloud, camPose.Pose(), worldCloud); err != nil {
		return nil, fmt.Errorf("transform %s cloud to world frame: %w", cam.Name().Name, err)
	}

	r.logger.Infof("Transformed %s cloud to world frame (%d points, camera at %v)",
		cam.Name().Name, worldCloud.Size(), camPose.Pose().Point())
	return worldCloud, nil
}

// savePointCloudToPCD writes a point cloud to a PCD file in binary format.
func savePointCloudToPCD(cloud pointcloud.PointCloud, path string) error {
	file, err := os.Create(path)
	if err != nil {
		return fmt.Errorf("create file: %w", err)
	}
	defer file.Close()

	if err := pointcloud.ToPCD(cloud, file, pointcloud.PCDBinary); err != nil {
		return fmt.Errorf("write PCD: %w", err)
	}

	return nil
}
