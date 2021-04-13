# Xweld - Local Perception Server

* [Description](#Description)
* [Prerequisites](#Prerequisites)
* [Distribution](#Distribution)
* [Usage](#Usage)
* [Error List](#Error)

### <a name="Description"></a>1. Description

This package implements a local perception pipeline to estimate joint welding points using point clouds. For further detailed methodology description check this [paper]().

![alt text](local_perception_server/images/pipeline.jpg)
<p align="center">
Joint welding estimation workflow.
</p>

### <a name="Prerequisites"></a>2. Prerequisites
1. [ROS](http://wiki.ros.org/ROS/Installation)
2. [Catkin Tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)
3. [PCL](https://pointclouds.org/downloads/)
4. [Eigen 3](https://gitlab.com/libeigen/eigen/-/releases/3.3.9)
5. [Point Cloud IO](https://github.com/carlosmccosta/point_cloud_io) (ONLY TO FAST DEBUG PURPOSE)

### <a name="Installation"></a>3. Installation

1. Setup all prerequisites.
2. Download the [local_perception_welding_xweld](https://github.com/ItzMeJP/local_perception_welding_xweld) repository to the src folder of your catkin workspace.
3. Build the messages and then the server packages. Assuming your catkin workspace is located in ~/catkin_ws:

    ```
    cd ~/catkin_ws
    catkin build grasp_estimation_skill_msgs
    catkin build grasp_estimation_skill_server
    ```
4. To fast debug (without a camera setup), donwload the [Point Cloud IO](https://github.com/carlosmccosta/point_cloud_io) repository to the src folder and compile it:
    ```
    cd ~/catkin_ws
    catkin build point_cloud_io
    ```
5. Uncomment the follwoing lines in /local_perception_server/package.xml:

    ```
    <!-- run_depend> point_cloud_io <run_depend-->
    ```

### <a name="Usage"></a>4. Usage

### Standalone usage
Run the following launch:
 ```
    roslaunch local_perception_server run.launch 
 ```
Request action skill by sending the following goal:
 ```
    rostopic pub /LocalPerceptionActionServer/goal local_perception_msgs/LocalPerceptionActionGoal "header:
     seq: 0
     stamp:
       secs: 0
       nsecs: 0
     frame_id: ''
    goal_id:
     stamp:
       secs: 0
       nsecs: 0
     id: ''
    goal:
     acquisition_distance: 0.2
     offset_compensation: [0, 0, 0]
     edge_tolerance: 0.0" 
 ```
  The result will be published at ...
  
### Windows bridge usage

Install the rosbridge package:
 ```
sudo apt install ros-$ROS_DISTRO$-rosbridge-server
 ```
 
Run the main launch:
 ```
    roslaunch local_perception_server run.launch 
 ```
 
 Run the bridge launch:
 ```
    roslaunch ros_bridge_server rosbridge_websocket.launch
 ```
 
Therefore the [Local Perception Client](https://github.com/ItzMeJP/local_perception_client) API can be used to develop C# local perception clients.


