# Installing Package
1) Clone git to your desired location
2) Copy the map explorer directory to a new folder on your pc

```
cp -r path_to_repo/packages/4202_map_explorer ~/
cd ~/4202_map_explorer
```

3) Build the package

```
colcon build --symlink-install --packages-select map_explorer
```

4) Open two new terminals, in Terminal 1, launch your gazebo world
```
ros2 launch turtlebot3_gazebo world_name.launch.py
```

Terminal 2 - launch the nav2 package with slam
```
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True slam:=True
```

5) Finally luanch the package by creating a ne wtermianl and running:
```
ros2 run map_explorer map_explorer
```





