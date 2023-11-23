# ROS2_LiDAR_Camera_detection

https://github.com/SeungHwi0613/ROS2_LiDAR_Detection/assets/108920644/8cb92e8c-5de8-4ea1-a0f1-aecb33d9a7fe  

## Dataset
kitti_10/27 --->(convert) ROS2 bagfile
https://github.com/tomas789/kitti2bag

*ROS2 bagfile one-drive lilnk*
https://konkukackr-my.sharepoint.com/:u:/g/personal/iish0613_konkuk_ac_kr/EXDNlwweqCRIkb9MNLB5ePUB_vVPutxy_6CVfjkxACreIA?e=GuwF1U

## Preprocessing
1) ROI
2) Road Extraction
3) Distance cut
4) RANSAC  

## Detection
1) Euclidean Clustering
2) Vehicle estimation
3) Centroid Extraction  

*i will upload 'Vehicle estimation' part soon*

## Comparison  
1) 2D centroid comparison
2) [1st filter] Cloest points(first, second) extraction
3) [2ed filter] Single camera geomtry criteria
4) 3D centroid publish

![structure](https://github.com/SeungHwi0613/ROS2_LiDAR_Camera_detection/assets/108920644/8976cfd6-8361-4a37-98b8-63cbdd3cf014)


https://github.com/SeungHwi0613/ROS2_LiDAR_Detection/assets/108920644/76078724-dec0-4edd-8679-f60203f0228b  
