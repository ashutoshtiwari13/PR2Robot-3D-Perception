
In brief, The file ```RANSAC.py``` performs these operations.

- Downsamples the point cloud by applying a Voxel Grid Filter.
- Apples a Pass Through Filter to isolate the table and objects.
- Performs RANSAC plane fitting to identify the table.
- Use the ExtractIndices Filter to create new point clouds containing the table and objects separately.

Each ```pcd file``` can be viewed uisng ```pcl_viewer <filename>.pcd```

## 1. VoxelGrid Downsampling Filter

A voxel grid filter allows you to downsample the data by taking a spatial average of the points in the cloud confined by each voxel. You can adjust the sampling size by setting the voxel size along each dimension. The set of points which lie within the bounds of a voxel are assigned to that voxel and statistically combined into one output point.

<div align="center">
<img src="https://github.com/ashutoshtiwari13/PR2Robot-3D-Perception/blob/master/ransac_pointcloud/voxel.png" height ="500px" width="400px"</img>
</div>
