# trajectory_matcher
This program is used to match transformation between two trajectories.
It can be used to calculate transformation from locally created map to gnss cooridinate.

## Subscribe Topic:
- ndt_pose
- gnss_pose

## Installation
1. cd ~/catkin_ws/src/
2. git clone https://github.com/mitsudome-r/trajectory_matcher
3. cd ..
4. catkin_make

## Usage

1. Do ndt_matching
2. Start nmea2tfpose
3. rosrun trajectory_matcher trajectory_matcher_node

## Transforming pointcloud map
1. sudo apt install pcl-tools
2. pcl_transform_point_cloud input.pcd output.pcd -trans x,y,z -matrix v1,v2,v3,v4,v5,v6,v7,v8.v9.v10,v11,v12
