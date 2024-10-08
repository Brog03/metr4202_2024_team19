# Node Structure
## map_explorer
### Publishers
/goal_pose

### Subscriptions
/odom
/scan
/behaviour_tree_log
/map
/aruco_markers (Only subscribed if aruco_detect is True in params.yaml)

# Installing Packages
1) Navigate to your worspace's src folder and clone this repository
```
cd path_to_worspace/src/
git clone https://github.com/Brog03/metr4202_2024_team19.git
git clone https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco.git
```

3) Once both repos have been cloned, navigate to the following folder and open up the following file:
```
cd path_to_ws/src/ros2_aruco/ros2_aruco/ros2_aruco
```
You will need to change lines 148 and 149 to the following in aruco_node.py

```
line 148 -> cv2.aruco.getPredefinedDictionary(dictionary_id)
line 149 -> self.aruco_parameters = cv2.aruco.DetectorParameters()  
```

4) In order to use Aruco detection, you must install the necessary python package by running
```
pip3 install opencv-contrib-python transforms3d
```

5) Naviagte back to the worspace and build all packages
```
cd ../
colcon build --symlink-install
source ~/.bashrc
```

# Running Package
## map_explorer
1) Open three new terminals, in Terminal 1, launch your gazebo world
```
ros2 launch turtlebot3_gazebo world_name.launch.py
```

Terminal 2 - launch the nav2 package
```
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True
```

Terminal 3 - luanch slam
```
ros2 launch slam_toolbox online_async_launch.py
```

2) Finally launch the package by creating a new termianl and running
```
ros2 launch map_explorer_bringup map_explorer_bringup.launch.py
```
## Map_explorer options
The map_explorer node is loaded with parameters located in params.yaml in map_explorer_bringup
aruco_detect -> Whether to enable aruco detection
num_aruco_markers -> Total number of markers the node will look for

Initial pose is automatically set through loading the varibles from params.yaml

## aruco_detection
To run Aruco Detection, open the params.yaml file and change aruco_detect to True, and run the following in a new terminal
```
ros2 run ros2_aruco aruco_node
```





