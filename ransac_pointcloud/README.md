
In brief, The file ```RANSAC.py``` performs these operations.

- Downsample your point cloud by applying a Voxel Grid Filter.
- Apply a Pass Through Filter to isolate the table and objects.
- Perform RANSAC plane fitting to identify the table.
- Use the ExtractIndices Filter to create new point clouds containing the table and objects separately.

Each ```pcd file``` can be viewed uisng ```pcl_viewer <filename>.pcd```
