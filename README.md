# NODE ROSBAG ANALYSIS

------------------------------------------

## Installation and Setup
A local installation and docker installation option is available. Since the docker installation uses **ros:noetic** and not **osrf/ros:noetic-desktop-full**, nodes that launch RVIZ cannot be launched. 

------------------------------------------

### Local Installation

1. Install ROS Noetic and setup catkin_ws.  
http://wiki.ros.org/noetic/Installation/Ubuntu

2. Source catkin_ws.   
`cd catkin_ws`  
`source devel/setup.bash` 

3. Clone git repository.  
`cd src/`  
`git clone https://github.com/nayan-pradhan/node_rosbag_analysis.git`

4. catkin_make.  
`roscd`  
`cd ..`  
`catkin_make`

5. Update path to BAG files in launch files.  
`cd src/node_rosbag_analysis/launch/`  
Use your favourite code editor to update launch files: **real_time_error_detection_launch.launch**, **review_error_detection_launch.launch**, and **rviz_launch.launch**.  
`<arg name="path_to_rosbag" value=" INSERT UPDATED PATH TO BAG FILE " />`  

------------------------------------------

### Launching Nodes Locally  

* Launch **rviz** for only visualization.  
`roslaunch node_rosbag_analysis rviz_launch.launch`  

* Launch **real_time_error_detection_node** for real time error detection using ROS Subscribers.    
`roslaunch node_rosbag_analysis real_time_error_detection_launch.launch`

* Launch **review_error_detection_node** for error detection review using Python ROSBAG API.  
`roslaunch node_rosbag_analysis review_error_detection_launch.launch`  

------------------------------------------

### Docker Installation

1. Install and setup Docker.  
https://docs.docker.com/engine/install/ubuntu/

2. Download Docker file from git repository.  
https://github.com/nayan-pradhan/node_rosbag_analysis/blob/master/Dockerfile 

3. Navigate to directory where Dockerfile was downloaded and build Docker Image.  
`docker build -t node-docker-image:1.0 .`

4. Check if Docker Image is built and copy **IMAGE_ID**.    
`docker images`  
A repository named **node-docker-image** should appear. Copy the **IMAGE_ID** of the repository.

5. Run Docker Image. Replace **/PATH-to-rosbag-files** with local path to directory with ROS BAG files.    
`docker run -t -d -v /PATH-to-rosbag-files:/mnt/NODE_Robotics/environment_files [IMAGE ID]`

6. Check if Docker Container is running. Copy the **CONTAINER ID** of the **IMAGE**.    
`docker ps`

7. Execute Docker Container and open bash in Container.  
`docker exec -it [CONTAINER ID] bash`

8. Source workspace.  
`cd /home`  
`source devel/setup.bash`  

9. Clone git repository.  
`cd src/`  
`git clone https://github.com/nayan-pradhan/node_rosbag_analysis.git`

10. catkin_make.  
`cd ..`  
`catkin_make`  

------------------------------------------