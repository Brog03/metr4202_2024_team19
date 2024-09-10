#########################################
###                                   ###
###              METR4202             ###
###                REPO               ###
###   CREATED 22/07/2024 - JACK BOYD  ###
###                                   ###
#########################################

Hi, welcome to the METR4202 repo, this is where all code for the course will be stored for this group

# TEAM MEMBERS
Jack Boyd - 47500037

amaana hussain - 4582775

Ngo Sen Lui - 47266641

Ole Fjeld Haugstvedt - 402216998


# Repo Struture
    /maps
        /Floor_Plans 
            floor_plan_name.sdf

        /map_name_1
            map_name_1.world
            map_name_1.launch.py

        /map_name_2
            map_name_2.world
            map_name_2.launch.py

        ...

    /packages
        /package_name
        setup.py
        packges.xml
            /package_name
                __init__.py
                package_name.py
                ... (Other python files or subdirectories ...)

# Maps 
## Creating Maps



1) First you need to create a folder where all the maps and floor plans will be saved
    ```
    mkdir -p path_to_folder/metr4202_maps
    mkdir path_to_folder/metr4202_maps/Floor_Plans
    ```

2) Create a new folder in maps:
    ```
    mkdir path_to_folder/metr4202_maps/Maps/map_name
    ```

3) Now open Gazebo and start by creating a floor plan by navigating to Editor -> Building editor
![alt text](image.png)

4) Once you have created a floor plan, navigate to File -> Save As, name your file name_Floor_Plan and make sure the loaction is path_to_folder/metr4202_maps/Floor_Plans

5) Once you have saved the floor plan, exit teh building editor by naviagting to File -> Exit Building Editor. The floorplan should be automatically added to your world. If not, then you can now insert your Floorplan into your world by navigating to Insert, and your should see the FloorPlan folder below. 

6) Once the floor plan is added, you can now start adding obstacles and AaruCo markers around the world using gazebo, (Make sure obstacles are static)

7) Once you have finalised the design, navigate to File -> Save World As
make sure the location is: path_to_folder/metr4202_maps/Maps/map_name and the name is map_name.world

8) You will need to create a launch.py script. Copy the launch.py script from this repo to a file and call it map_name.launch.py and make sure it is saved in location 
path_to_folder/metr4202_maps/Maps/map_name, and make sure to edit where needs editing

9) Once this is done, you will need copy the .world and .launch file to the gazebo folders:
    map_name.world path_to_turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds

    map_name.launch.py: path_to_turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch

10) Then run the following commands:
    ```
    cd path_to_turtlebot3_ws/
    colcon build
    source install/setup.bash
    ```

11) Then you should be able to open the world with gazebo by running:
    ```
    ros2 launch turtlebot3_gazebo map_name.launch.py
    ```
#

When creating a world, in order for ros to be able to find your world, it must be saved in:

    path_to_turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds

and have a respective .launch.py script saved in

    path_to_turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch

I would recomend saving the maps and lucnh files to some folder in your WSL, then copy the .world and .launch.py files to these locations after saving. 

## Using Maps From Repo
1) Navigate to the Maps folder in the repo and do the following
    Copy the .world file to:
    path_to_turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds

    Once this is done, you will need copy the .launch.py file to:
    path_to_turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch

2) Then run the following commands:
    ```
    cd path_to_turtlebot3_ws/
    colcon build
    install/setup.bash
    ```
3) Then you should be able to open the world with gazebo by running:
    ```
    ros2 launch turtlebot3_gazebo map_name.launch.py
    ```

# Packages #
## Using Repo Packages ##
In order to use a package uploaded to the repo, first start by creating a new empty package in workspace src directory, calling it the package name:
```
cd path_to_work_space/src
ros2 pkg create --build-type ament_python <package_name>
```

Then you will need to copy all the python package files from the repo to this package, DO NOT INCLUDE setup.py

Once this is done, you will need to edit both package.xml and setup.py

1) package.xml -> copy all lines <depend>package</depened> and paste them into your package.xml file

2) setup.py -> copy the line 'package_name = package_name.package_name:main' to your setup.py file

Once this is done, you should be able to build an run the package:
```
colcon build --symlink-install --packages-select package_name
ros2 run package_name package_name
```

# ROS2
Here are some important commands

## Launch Gazebo World
```
ros2 launch turtlebot3_gazebo world_name.launch.py
```
## Launching SLAM
```
ros2 launch slam_toolbox online_async_launch.py
```
## Using NAV2 library
Using SLAM
```
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True slam:=True
```
Using a saved map
```
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/path/to/turtlebot3_world_map.yaml
```




    

            



