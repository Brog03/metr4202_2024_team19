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

# Installing Packages
1) Navigate to your worspace's src folder and clone this repository
```
cd path_to_worspace/src/
git clone git@github.com:Brog03/metr4202_2024_team19.git
```

2) Naviagte back to the worspace and build all packages
```
cd ../
colcon build --symlink-install
source install/setup.bash
```

# Running Package
## map_explorer
1) Open three new terminals, in Terminal 1, launch your gazebo world
```
ros2 launch turtlebot3_gazebo <world_name>.launch.py
```

Terminal 2 - launch the nav2 package
```
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True
```

Terminal 3 - launch the SLAM package 
```
ros2 launch slam_toolbox online_async_launch.py
```

Terminal 4 - Finally launch the package by creating a new termianl and run
```
ros2 launch map_explorer_bringup map_explorer_bringup.launch.py
```

When map_explorer node is running and has outputted READY, open the rviz window, and give an initial waypoint that is close to the turtlebot (Testing)

## Map_explorer options
The map_explorer node is loaded with parameters located in params.yaml in map_explorer_bringup

aruco_detect -> Whether to enable aruco detection <br />
num_aruco_markers -> Total number of markers the node will look for <br />
debug -> Whether to enable output from the map_explorer node <br />

Initial pose is automatically set through loading the varibles from params.yaml







