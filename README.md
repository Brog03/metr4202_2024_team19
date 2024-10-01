# Installing Package
1) Navigate to your worspace's src folder and clone this repository
```
cd path_to_worspace/src/
git clone https://github.com/Brog03/metr4202_2024_team09.git
```
2) Naviagte back to the worspace and build the desired package
```
cd ../
colcon build --symlink-install --packages-select package_name
```

# Running Package
## map_explorer
1) Open two new terminals, in Terminal 1, launch your gazebo world
```
ros2 launch turtlebot3_gazebo world_name.launch.py
```

Terminal 2 - launch the nav2 package with slam
```
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True slam:=True
```

2) Finally luanch the package by creating a new termianl and running:
```
ros2 run map_explorer map_explorer
```

## aruco_detection





