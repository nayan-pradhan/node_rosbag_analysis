# NODE ROSBAG ANALYSIS

Hello, welcome to my project! 

## Installation and Setup

### Local Installation
1. Install ROS Noetic and setup catkin_ws  
http://wiki.ros.org/noetic/Installation/Ubuntu

2. Source catkin_ws   
`cd catkin_ws`  
`source devel/setup.bash` 

3. Clone git repository  
`cd src/`  
`git clone https://github.com/nayan-pradhan/node_rosbag_analysis.git`

4. catkin_make  
`roscd`  
`cd ..`  
`catkin_make`

5. Update path to BAG files in launch files  
`cd src/node_rosbag_analysis/launch/`  
Use your favourite code editor to update launch files: real_time_error_detection_launch.launch, review_error_detection_launch.launch, and rviz_launch.launch  
`<arg name="path_to_rosbag" value=" INSERT UPDATED PATH TO BAG FILE " />`  