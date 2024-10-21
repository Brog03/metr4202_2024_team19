# The Team
Name - Student ID - Username

Jack Boyd - 47500037 - Brog03
Ole Fjeld Haugstvedt - 49055942 - olefhau
Ngo Sen Lui - 47266641 - Gordon-Lui517
Amaana Hussain - 45827758 - AmaanaHussain

# Node Structure
## map_explorer
### Publishers 
/goal_pose

### Subscriptions
/odom <br />
/scan <br />
/behaviour_tree_log <br />
/map <br />
/global_costmap_costmap <br />
/aruco_markers <br />

# Installing Packages
1) Navigate to your worspace's src folder and clone this repository
```
cd path_to_worspace/src/
git clone git@github.com:Brog03/metr4202_2024_team19.git
```

2) You will need to install tf_transformatiosn for ros2 humble, do this by running the following command
```
sudo apt install ros-humble-tf-transformations
```

3) Naviagte back to the worspace and build all packages
```
cd ../
colcon build --symlink-install
source install/setup.bash
```

# Running Package
1) Open two new terminals, in Terminal 1, launch your gazebo world
```
ros2 launch turtlebot3_gazebo <world_name>.launch.py
```

Terminal 2 - launch the nav2 package
```
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True slam:=True
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True slam:=True
```

## Without Aruco Detection
NOTE - Before Launching map_explorer: <br />
- Make sure aruco_detect is set to False in params.yaml

Open another new terminal and launch the package by creating a new termianl and run
```
ros2 launch map_explorer_bringup map_explorer_bringup.launch.py
```

## With Aruco Dectecion
NOTE - Before Launching map_explorer: <br />
- Make sure aruco_detect is set to True in params.yaml
- When map_explorer is launched, it will wait until aruco_node has created the /aruco_markers topic


Open 2 new terminals, in Terminal 1, launch the map_explorer node
```
ros2 launch map_explorer_bringup map_explorer_bringup.launch.py
```

Terminal 2 - launch the aruco_node
```
ros2 launch ros2_aruco aruco_recognition.launch.py
```
<br />
When map_explorer node is running and has outputted READY, open the rviz window, and give an initial waypoint that is close to the turtlebot (Testing)
<br />


## Map_explorer options
The map_explorer node is loaded with parameters located in params.yaml in map_explorer_bringup

aruco_detect -> Whether to enable aruco detection <br />
num_aruco_markers -> Total number of markers the node will look for <br />
debug -> Whether to enable output from the map_explorer node <br />

Initial pose is automatically set through loading the varibles from params.yaml







