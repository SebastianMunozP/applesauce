package applesauce

import (
	"context"
	"fmt"
	"os"

	"github.com/golang/geo/r3"

	applepose "github.com/biotinker/applesauce/apple_pose"
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

// transformDetectionToWorldFrame transforms the entire detection result (apple poses,
// point clouds, and features) from camera frame to world frame. This modifies the
// detection result in place and saves the transformed camera point cloud.
func transformDetectionToWorldFrame(ctx context.Context, r *Robot, cameraCloud pointcloud.PointCloud, result *applepose.DetectionResult) error {
	if r.primaryCam == nil {
		return fmt.Errorf("no primary camera available")
	}

	// Get camera pose in world frame
	cameraPoseInWorld, err := r.fsSvc.GetPose(ctx, r.primaryCam.Name().Name, "", nil, nil)
	if err != nil {
		return err
	}

	r.logger.Infof("Camera pose in world frame: %v", cameraPoseInWorld.Pose())

	// Transform the full camera point cloud to world frame and save it directly
	transformedCameraCloud := pointcloud.NewBasicPointCloud(cameraCloud.Size())
	err = pointcloud.ApplyOffset(cameraCloud, cameraPoseInWorld.Pose(), transformedCameraCloud)
	if err != nil {
		return fmt.Errorf("failed to transform camera point cloud: %w", err)
	}

	// Save the transformed camera cloud (world frame)
	outputDir := "pointclouds"
	cameraWorldPath := fmt.Sprintf("%s/camera_full_world.pcd", outputDir)
	if err := savePointCloudToPCD(transformedCameraCloud, cameraWorldPath); err != nil {
		r.logger.Warnf("Failed to save world-frame camera cloud: %v", err)
	} else {
		r.logger.Infof("Saved world-frame camera point cloud to %s (%d points)", cameraWorldPath, transformedCameraCloud.Size())
	}

	// Transform each apple's pose, point cloud, and features to world frame.
	for i := range result.Bowl.Apples {
		// Transform apple pose
		result.Bowl.Apples[i].Pose = spatialmath.Compose(cameraPoseInWorld.Pose(), result.Bowl.Apples[i].Pose)

		// Transform apple point cloud
		if result.Bowl.Apples[i].Points != nil {
			pc := result.Bowl.Apples[i].Points
			pcInWorld := pointcloud.NewBasicPointCloud(pc.Size())
			err = pointcloud.ApplyOffset(pc, cameraPoseInWorld.Pose(), pcInWorld)
			if err != nil {
				return fmt.Errorf("failed to transform apple point cloud: %w", err)
			}
			result.Bowl.Apples[i].Points = pcInWorld
		}

		// Transform feature poses and point clouds
		for j := range result.Bowl.Apples[i].Features {
			result.Bowl.Apples[i].Features[j].Pose = spatialmath.Compose(cameraPoseInWorld.Pose(), result.Bowl.Apples[i].Features[j].Pose)

			if result.Bowl.Apples[i].Features[j].Points != nil {
				pc := result.Bowl.Apples[i].Features[j].Points
				pcInWorld := pointcloud.NewBasicPointCloud(pc.Size())
				err = pointcloud.ApplyOffset(pc, cameraPoseInWorld.Pose(), pcInWorld)
				if err != nil {
					return fmt.Errorf("failed to transform feature point cloud: %w", err)
				}
				result.Bowl.Apples[i].Features[j].Points = pcInWorld
			}
		}
	}

	r.logger.Infof("Transformed %d apples (poses, point clouds, features) from camera frame to world frame", len(result.Bowl.Apples))
	return nil
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
