VERSION = "1.1.5"

import rclpy
import rclpy.logging
from rclpy.node import Node

import os

from geometry_msgs.msg import PoseStamped
from nav2_msgs.msg import BehaviorTreeLog
import rclpy.subscription
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from ros2_aruco_interfaces.msg import ArucoMarkers

import time
import numpy as np
import math

FUNCTION_HANDLE_INDEX_ARGUMENT = 3
FUNCTION_HANDLE_INDEX = 2
NODE_STATUS_INDEX = 1
NODE_NAME_INDEX = 0

CONSTANT_PI = math.pi

ANGLE_WIDTH_DEG = 2 # set robot to take scans in 2 degree increments 
ANGLE_WIDTH_RAD = ANGLE_WIDTH_DEG*(CONSTANT_PI/180.) # convert 2 degree increments to radians 

NUM_DIRECTIONS = int(360/ANGLE_WIDTH_DEG) # number of directions robot can take in a 360 deg scan

DISTANCE_STEP = 2 # set robot to have step size of 2
LASER_SCAN_MIN_RANGE = 50

LAMBDA_deg_to_rad = lambda x : x*(CONSTANT_PI/180.)
LAMBDA_rad_to_deg = lambda x : x*(180./CONSTANT_PI)

MAP_OUT_OF_BOUNDS = 200
EMPTY_WAYPOINT = np.zeros(2)
print(EMPTY_WAYPOINT)

class BehaviourTreeLog_Handler(object):
    """
    Subscription to the Behaviour Tree Log
    """
    subscription: rclpy.subscription = None
    functionHandlers: list[str, str] = None

    def __init__(self, node: Node, functionHandlers: list[tuple[str, str]]):
        """
        Sets up a subscription to the /behavior_tree_log topic

        params:
            node -> Node that is subscribing to the topic
            functionHandlers -> An array of tuples that store what function should run based on node and its status
                [
                    (Node1 Name, Node1 Status, function, argument_to_function), 
                    (..., ..., ..., ...), 
                    ...
                ]
        """
        self.functionHandlers = functionHandlers

        self.subscription = node.create_subscription(
            BehaviorTreeLog,
            "behavior_tree_log",
            self.callback, 
            10
        )
    
    def callback(self, msg: BehaviorTreeLog) -> None:
        """
            Check the current state of the behaviour tree.

            Params:
                msg -> conatins data about the behaviour tree

        """

        # Checks to see if any of the handler conditins have been met
        for event in msg.event_log:
            for handler in self.functionHandlers:
                if event.node_name == handler[NODE_NAME_INDEX] and event.current_status == handler[NODE_STATUS_INDEX]:
                    handlerArgument = handler[FUNCTION_HANDLE_INDEX_ARGUMENT]
                    handler[FUNCTION_HANDLE_INDEX](handlerArgument)

class LaserScan_Subscriber(object):
    """
    Subscription to the LaserScan topic
    """
    
    subscription = None # initialise subscription 

    data: list[float] = None # Scan data (meters)
    minAngle: float = None # Scan angle lower bound (rads)
    maxAngle: float = None # Scan angle upper bound (rads)
    angleIncrement: float = None # Scan angle increment (rads)

    def __init__(self, node: Node) -> None:
        """
            Sets up a subscription to the /scan topic

            params:
                node -> Node that is subscribing to the topic
        """
        self.subscription = node.create_subscription(
            LaserScan, 
            "scan",
            self.callback,
            10
        )

    def callback(self, msg: LaserScan) -> None:
        """
            Call back for laserScan Subscription - reads the current scan data

            Params:
                msg -> conatins data of the laser scan
        """
        self.data = msg.ranges
        self.minAngle = msg.angle_min
        self.maxAngle = msg.angle_max
        self.angleIncrement = msg.angle_increment

    def get_Data(self) -> list[float]:
        """
            stores laser scan data 

            Returns: 
                a list of laser scan data 
        """
        return self.data
    
    def get_scan_average(self, startIndex: float, endIndex: float) -> float:
        """
            Gets the average value of the laser scan data between two indexes

            Params:
                startIndex -> first index in the laser scan data array
                endIndex -> last index in the laser scan data array

            Returns:
                Average value bewteen these two indexes
        """

        dataSum = 0
        #sum all values in laser scan data list 
        for i in range(startIndex, endIndex):
            dataSum += self.data[i]

        return dataSum/(endIndex-startIndex)
        
    

class Odom_Subscriber(object):
    """
    Subscription to the Odom topic
    """
    
    subscription = None

    w: float = None # Current w (rads)
    x: float = None # Current x (meters)
    y: float = None # Current y (meters)

    def __init__(self, node: Node) -> None:
        """
        Sets up a subscription to the /odom topic

        params:
            node -> Node that is subscribing to the topic
        """
        self.subscription = node.create_subscription(
            Odometry, # odometry msg class
            "odom", # /odom topic 
            self.callback, # callback function for /odom 
            10 
        )

    def callback(self, msg: Odometry) -> None:
        """
            Callback function for the topic - reads odom

            Params: 
                msg -> contains data of the robots odometry
        
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.w = self.euler_from_quaternion(msg.pose.pose.orientation)
    
    def euler_from_quaternion(self, quaterion: list[float, float, float, float]) -> tuple[float, float, float]:
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)

            Params: 
                quaterion -> the robots orientation in the form of a quaternion
            
            Returns:
                roll, pitch and yaw (rads)
        
        """
        x = quaterion.x # robots position in x 
        y = quaterion.y # robots position in y 
        z = quaterion.z # robots position in z
        w = quaterion.w # robots orientation in quaterions  

        # robots orientation in euler_co-ordinates
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return yaw_z # in radians

    def get_X(self) -> float:
        """
        Returns robots pose in x as a float 
        """
        return self.x
    
    def get_Y(self) -> float:
        """
         Returns robots pose in y as a float 
        """
        return self.y
    
    def get_W(self) -> float:
        """
         Returns robots euler angles as floats
        """
        return self.w
    
class Subscriber_Map(object):
    """
    Subscription to the map topic
    """
    subscription: rclpy.subscription = None # initialise subscription to ros python library 

    mapData: list[float] = [] # Map data (int)
    resolution: float = None # Map resolution (meters)
    dimX: float = None # Map width (units)
    dimY: float = None # Map height (units)

    minX: float = None # Map lower x bound (meters)
    minY: float = None # Map lower y bound (meters)
    maxX: float = None # Map upper x bound (meters)
    maxY: float = None # Map upper y bound (meters)

    def __init__(self, node: Node, subscription_: str) -> None:
        """
            Sets up a subscription to the topic

            params:
                node -> Node that is subscribing to the topic
        """
        
        self.subscription = node.create_subscription(
            OccupancyGrid, # Occupancy Grid msg class 
            subscription_, # subscription parameter 
            self.callback, # callback function for laserScan subscription 
            10
        )

    def callback(self, msg: OccupancyGrid) -> None:
        """
            Call back for laserScan Subscription - reads the current scan data

            Params:
                msg -> conatins data of the laser scan
        """

        # Extract data from message
        self.dimX = msg.info.width
        self.dimY = msg.info.height
        self.mapData = msg.data
        self.resolution = msg.info.resolution

        # Calculate Map upper and lower x and y bounds
        self.minX = msg.info.origin.position.x
        self.maxX = self.minX + (self.dimX*self.resolution)

        self.minY = msg.info.origin.position.y
        self.maxY = self.minY + (self.dimY*self.resolution)

    def array_index_to_cartesian(self, index: int) -> tuple[int, int]:
        """
            Converts map data index (1D array) to cartesian coordinates

            Params:
                index -> index to be converted to cartesian coordiantes

            Returns:
                x and y value of cartesian coordinate
        """
        cartesianX = self.minX + (index % self.dimX)*self.resolution # x coordinate from index
        cartesianY = self.minY + (index // self.dimX)*self.resolution # y coordinate from index

        return cartesianX, cartesianY
    
    def cartesian_to_array_index(self, x: float, y: float) -> int:
        """
            Converts cartesian coordinates to the corresponding grid cell index in the map data array

            Params:
                x -> x value
                y -> y value

            Returns:
                None if index is out of range otherwise the corresponding index
        """

        index = 0

        # Check to see if the given x, y are within the occupancy grid bounds
        if not self.cartesian_coordinate_in_occupancy_grid(x, y):
            # If not, return None
            index = None
        else:
            # If so, return the corresponding index
            index += abs(((x - self.minX)//self.resolution))
            index += abs(((y - self.minY)//self.resolution)*self.dimX)
            index = int(index)

        return index
    
    def cartesian_coordinate_in_occupancy_grid(self, x: float, y: float) -> bool:
        """
            Checks to see whether a point is contained within the occupancy grid

            Params:
                x -> x value of grid cell 
                y -> y value of grid cell

            Returns:
                True if point is contained otherwise false
        """
        return ((x > self.minX) and (x < self.maxX) and (y > self.minY) and (y < self.maxY))
    
    def get_data_array(self) -> list[float]:
        """
            Stores occupancy map data in a list 
        """
        return self.mapData
    
    def get_data_with_cartesian(self, x: float, y: float) -> float:
        """
            Get vaulue of map data from cartesian co-ordinates 

            Params: 
                x-> x value of grid cell 
                y-> y value of grid cell 

            Returns: 
                value of the map data at an index if true, otherwise 100 

        """
        data = None

        index = self.cartesian_to_array_index(x, y) # get map index for each cartesian co-ordinate 
       
        if (index != None):
            # checks if map index is not none and returns the value at that map index 
            data = self.mapData[index]
        else:
            # if map index is none then return 100 
            data = MAP_OUT_OF_BOUNDS

        return data

    def get_data_with_index(self, index: int) -> float:
        """
            get map data by an index 

            Params: 
                index -> index of the map data array 

            Returns: 
                value of map at an index 
        """
        return self.mapData[index]
    
    def average(self, x: float, y: float, radius: int) -> float:
        """
            Returns the average value of a occupancy grid cell and its surrounding grid cells
            The search radius is defined by self.averageSquareSize:
                For a sqaure size of 5, it will search 5 grid cells to the left, right, above and below the grid cell
                and sum up the value in that grid cell

                If the surrounding grid cells are not withing the occupancy grid, they are excluded from the average:
            
                Params:
                    x -> x value of grid cell
                    y -> y value of grid cell

                Returns:
                    Average value of grid cell (x, y)
        """

        average = 0
        count = 0

        # loop through surrounding grid cells
        for xSearch in range(-radius, radius):
            for ySearch in range(-radius, radius):
                # Calculate surrounding grid cell (x, y)
                newX = xSearch*self.resolution + x
                newY = ySearch*self.resolution + y
                
                # Get the index of this new grid cell
                index = self.cartesian_to_array_index(newX, newY)
                
                # Check if new grid cell is within occupancy map
                if index != None:
                    # New grid cell is within occupancy map, include in the average
        
                    average += self.mapData[index]
                    count += 1

        if (count != 0):
            average /= count
        else:
            average = MAP_OUT_OF_BOUNDS
        
        return average


class OccupancyGrid_Subscriber(Subscriber_Map):
    """
    Subscription to the LaserScan topic
    """

    def __init__(self, node: Node) -> None:
        """
            Sets up a subscription to the /scan topic

            params:
                node -> Node that is subscribing to the topic
        """

        super().__init__(node, "map") 


    def is_point_on_frontier(self, x: float, y: float, radius) -> bool:
        """
            Used to search for frontiers: will search grid cells surrounding the given grid cell. If a frontier is present,
            50% of the grid cells will be unoccupied (-1) and the other 50% will be empty (0)
            
                Params:
                    x -> x value of grid cell
                    y -> y value of grid cell

                Returns:
                    Average value of grid cell (x, y)
        """

        countUnExplored = 0
        countFree = 0
        countTotal = 0

        # Loop through surrounding grid cells
        for xSearch in range(-radius, radius):
            for ySearch in range(-radius, radius):
                # Calculate surrounding grid cell (x, y)
                newX = xSearch*self.resolution + x
                newY = ySearch*self.resolution + y

                # get map index that corresponds to occupancy grid cartesian co-ordinates 
                index = self.cartesian_to_array_index(newX, newY)

                if index != None:
                    # check if value of grid cell in map at this index is -1 
                    if self.mapData[index] == -1:
                        # update countUnExplored variable when grid cell value is -1 
                        countUnExplored += 1
                    #check if value of grid cell in map at this index is 0
                    elif self.mapData[index] == 0:
                        #update countFree variable when grid cell value is 0 
                        countFree += 1

                    countTotal += 1
        percentUnExplored, perecentFree = countFree/countTotal, countUnExplored/countTotal
        onFrontier = (abs(percentUnExplored - perecentFree) < 0.02) and (abs(percentUnExplored - 0.5) < 0.02)

        return onFrontier
    
    def search_around_grid_cell(self, x, y, radius):
        """
            Searches for grid cells surrounding a specified (x, y) position within a given radius.

            Params:
                x (float): The x-coordinate of the grid cell around which the search is performed.
                y (float): The y-coordinate of the grid cell around which the search is performed.
                radius (int): The search radius in grid cells, defining the area to be explored around (x, y).

            Returns:
                list[int]: A list of indices corresponding to the neighboring grid cells within the radius.
        """
        neighbouringGridCells = []

        for xSearch in range(-radius, radius):
            for ySearch in range(-radius, radius):
                # Calculate surrounding grid cell (x, y)
                newX = xSearch*self.resolution + x
                newY = ySearch*self.resolution + y

                # get map index that corresponds to occupancy grid cartesian co-ordinates 
                index = self.cartesian_to_array_index(newX, newY)

                if index != None:
                    neighbouringGridCells.append(index)

        return neighbouringGridCells



    
class CostMap_Subscriber(Subscriber_Map):
    """
    Subscription to the /global_costmap/costmap topic
    """

    def __init__(self, node: Node) -> None:
        """
            Sets up a subscription to the /scan topic

            params:
                node -> Node that is subscribing to the topic
        """

        super().__init__(node, "/global_costmap/costmap") 
        

class ArucoMarker_Subscriber(object):
    """
    Subscription to /aruco_markers
    """
    
    subscription: rclpy.subscription = None
    node: Node = None
    arucoMarkersPoses: list[tuple[int, np.ndarray]] = []
    totalArucoMarkers = 0

    def __init__(self, node: Node, odom: Odom_Subscriber) -> None:
        """
            Sets up a subscription to the /aruco_markers topic

            params:
                node -> Node that is subscribing to the topic
        """
        # Wait until there is a publisher to /aruco_markers
        self.node = node
        self.wait_aruco_node_running(node)
        self.S_Odom = odom
    
        self.subscription = node.create_subscription(
            ArucoMarkers, 
            "aruco_markers",
            self.callback,
            10
        )

    def wait_aruco_node_running(self, node: Node):
        """
            Blocks until a aruco_node is publishing to /aruco_markers

            params:
                node -> Node that is subscribing to the topic
        """

        while True:
            # Get publishers of the /aruco_markers topic
            aruco_marker_publishers = node.get_publishers_info_by_topic("/aruco_markers")

            # Check who is currently publishing to this topic
            if not aruco_marker_publishers:
                # If no publishers exist, block for another 5 seconds and try again
                log("INFO", node, "Aruco Node not running - Trying again in 5 seconds", True)
                time.sleep(5)

            else:
                # A publisher exists
                publisher = aruco_marker_publishers[0]._node_name

                if publisher == "aruco_node":
                    # If publish is aruco_node, then stop blocking and continue the program
                    log("INFO", node, "Aruco Detection is Running, starting map_exporler node", True)
                    break

    def callback(self, msg: ArucoMarkers) -> None:
        """
            Call back for aruco_markers Subscription

            Params:
                msg -> conatins data of the current poses and IDs of the detected aruco_marketrs
        """
        #Loop over detected Aruco Markers by index and ID
        for index, currentID in enumerate(msg.marker_ids):
            used = False

            #Check if the current detected marker ID is already saved
            for savedArucoMarker in self.arucoMarkersPoses:
                savedID = savedArucoMarker[0] #Saved marker ID
                savedX = savedArucoMarker[1][0] #Saved marker X-coordinate
                savedY = savedArucoMarker[1][1] #Saved marker Y-coordinate
                
                #If the current detected marker ID matches the saved ID, mark it as 'used'
                if currentID == savedID:
                    used = True
                    break

            arucoXtoRobot = msg.poses[index].position.x
            arucoYtoRobot = msg.poses[index].position.y

            #If the marker is not already saved, calculate its global position and save it
            if not used:
                #Calculate actual position by adding robot's odometry position
                actualArucoX = arucoXtoRobot + self.S_Odom.get_X()
                actualArucoY = arucoYtoRobot + self.S_Odom.get_Y()

                #Save the marker ID and its global position along with a format string
                self.arucoMarkersPoses.append([currentID, [actualArucoX, actualArucoY], "Aruco Marker -> ID: {ID} x: {X:.4f}, y: {Y:.4f}"])
                self.totalArucoMarkers += 1 #Increment total aruco markers detected

                log("INFO", self.node, f"New Aruco Marker -> ID: {currentID}", True) #Log new marker detected

            else:
                #If the marker is already saved, update its position by averaging the new and old positions
                savedArucoMarker[1][0] = ((arucoXtoRobot + self.S_Odom.get_X()) + savedX)/2
                savedArucoMarker[1][1] = ((arucoYtoRobot + self.S_Odom.get_Y()) + savedY)/2

                #Log the updated position of the existing marker
                log("INFO", self.node, f"New Aruco Marker -> ID: {currentID}, x: {savedArucoMarker[1][0]:.4f}, y: {savedArucoMarker[1][1]:.4f}", True)

    def print_aruco_poses(self):
        #Loop through all saved Aruco markers and print their positions
        for marker in self.arucoMarkersPoses:
            formatString = marker[2]
            x = marker[1][0]
            y = marker[1][1]
            id = marker[0]

            log("INFO", self.node, formatString.format(X = x, Y = y, ID = id), True)
            

class Explorer(Node):
    """
    How the robot chooses waypoints

    This node is setup to be subscribed to the following topics:
        /odom
        /scan
        /behaviour_tree_log
        /map
        /global_costmap/costmap
    
    and to publish to the follwoing topics:
        /goal_pose

    """

    SEARCH_RADIUS: int = 5 # search for waypoints within this radius 

    S_Odom: Odom_Subscriber = None # Subscription to /odom
    S_Scan: LaserScan_Subscriber = None # Subscription to /scan
    S_BehaviourTree: BehaviourTreeLog_Handler = None # Subscription to /behaviour_tree_log
    S_Map_Occupancy: OccupancyGrid_Subscriber = None # Subscription to /map
    S_Map_Global_Cost: CostMap_Subscriber = None # Subscription to /map
    S_Aruco: ArucoMarker_Subscriber = None # Subscription to /aruco_marker

    waypointCounter: int = 0 # Total waypoints published

    GLOBAL_completedWaypointVectors: list[np.ndarray] = [] # Global Position vectors of the completed waypoints
    LOCAL_completedWaypointVectors: list[np.ndarray] = [] # Local Position vectors of the completed waypoints
    failedWaypointsVectors: list[np.ndarray] = [] # Position vectors of the failed waypoints
    openFrontiers: list[np.ndarray] = []

    totalArucoMarkers = None

    robotPoseVector = EMPTY_WAYPOINT

    currentWaypoint: np.ndarray = EMPTY_WAYPOINT #set current waypoint to be PoseStamped message type 

    debug: bool = None  # initialise debug variable as boolean 

    def __init__(self):
        """
            Initialises the Explorer Node
        """

        super().__init__("map_explorer")

        # Declaring parameters loaded from params.yaml
        self.declare_parameters(
            namespace="",
            parameters=[
                ("aruco_detect", rclpy.Parameter.Type.BOOL), # set up aruco_detect variable as boolean 
                ("num_aruco_markers", rclpy.Parameter.Type.INTEGER), # set the number of aruco markers as integer 
                ("debug", rclpy.Parameter.Type.BOOL) # set debug variable as boolean 
            ]
        )

        # get value of aruco_detect
        rosParam_aruco_detect = self.get_parameter('aruco_detect').get_parameter_value().bool_value
        # get value for num_aruco_markers 
        rosParam_num_aruco_markers = self.get_parameter('num_aruco_markers').get_parameter_value().integer_value
        # get value for debug  
        rosParam_debug = self.get_parameter('debug').get_parameter_value().bool_value


        # Function handlers
        FUNCTION_HANDLERS = [
            ("NavigateRecovery", "SUCCESS", self.chooseWaypoint, False), # Call self.chooseWaypoint(False) if waypoint was reached
            ("NavigateRecovery", "FAILURE", self.chooseWaypoint, True) # Call self.chooseWaypoint(True) if waypoint wasn't reached
        ]   

        # Setup subscriptions
        self.S_Odom = Odom_Subscriber(self) # Setup /odom subscription
        self.S_Scan = LaserScan_Subscriber(self) # Setup /scan subscription
        self.S_Map_Occupancy = OccupancyGrid_Subscriber(self) # Subscription to /map
        self.S_BehaviourTree = BehaviourTreeLog_Handler(self, FUNCTION_HANDLERS) # Subscription to /behaviour_tree_log 
        self.S_Map_Global_Cost = CostMap_Subscriber(self) # subscriton to /cost_map 

        log("WARN", self, "VERSION: " + VERSION, True) ##

        # Check if aruco_detect is True
        if rosParam_aruco_detect == True:
            log("WARN", self, "Running map_explorer with Aruco decetion", True)
            self.S_Aruco = ArucoMarker_Subscriber(self, self.S_Odom) # Subscription to /aruco_marker 
        else:
            log("WARN", self, "Running map_explorer without Aruco decetion", True)
    
        # Create publisher to /goal_pose
        self.GoalPose_Publisher = self.create_publisher(
            PoseStamped, # PoseStamped msg type 
            "goal_pose", # publish to /goal_pose topic 
            10
        )

        self.LOCAL_completedWaypointVectors.append(np.array([0, 0])) # add starting waypoint to local_completedWaypointVectors  

        # check if debug is active or not 
        if not rosParam_debug:
            # print ros debug parameter is disabled 
            log("INFO", self, "READY (Debug disabled)", True)
            self.debug = False
        else:
            # print ros debug parameter is active 
            log("INFO", self, "READY", True)
            self.debug = True

        self.totalArucoMarkers = rosParam_num_aruco_markers

    def find_unoccupied_directions(self) -> list:
        """
            Divides the /scan data into ANGLE_WIDTH_RAD segments (e.g 10deg segments would have 36 total segments)
            and gets average scan distance for each segment and checks to see if it can travel in the middle of that segment

            Returns:
                A list of angles (In reference to the point (0, 0, 0)) in which the robot can travel in
        """

        unoccupiedDirections = [] # empty list for directions which robot can travel in 
        subScanSize = int(len(self.S_Scan.get_Data())/NUM_DIRECTIONS) # size of the scan data in each 2 deg segment 

        for i in range(NUM_DIRECTIONS):
            freeAngle = 0 # angle which are not obstructed by an obstacle 
            bot_angle = 0 # angle of robot 

            startIndex = i*subScanSize # start index of subscan data 
            endIndex = startIndex + subScanSize - 1 # end index of subscan data 

            avg_range = self.S_Scan.get_scan_average(startIndex, endIndex) # Get average distance of each scan segment 

            # Convert angle to be within range (-180 <= arg <= 180)
            if i*ANGLE_WIDTH_DEG > 180:
                freeAngle = (-1.)*(LAMBDA_deg_to_rad(360.) - ANGLE_WIDTH_RAD*(i-0.5))
            else:
                freeAngle = ANGLE_WIDTH_RAD*(i-0.5)
            
            # Calculate the angle in reference to the point (0, 0, 0)
            bot_angle = self.S_Odom.get_W() + freeAngle
            
            # Check if angle has no obstacles in front of it
            if avg_range >= LASER_SCAN_MIN_RANGE:
                unoccupiedDirections.append(bot_angle)

        return unoccupiedDirections

    def calculate_unoccupied_directions_vectors(self, unoccupiedDirections: list[float]) -> list[np.ndarray]:
        """
            Calculates position vectors for where the robot will travel based on DISTANCE_STEP and 
            free angles

            Params:
                 unoccupiedDirections -> Angles in which there are no obstacles

            Returns:
                positions of the Vector class where the robot can travel
        """
        
        unoccupiedDistanceVectors = [] # initialise empty list for positions which are unoccupied  

        x = self.S_Odom.get_X() # get robots pose in x 
        y = self.S_Odom.get_Y() # get robots pose in y 

        # Go through each angle and calculate where the robot's new X and Y will be
        for angle in unoccupiedDirections:            
            newX = x + math.cos(angle)*DISTANCE_STEP
            newY = y + math.sin(angle)*DISTANCE_STEP
            # store robots new pose as a vector and add to unoccupiedDistanceVector 
            vector = np.array(([newX, newY]))
            unoccupiedDistanceVectors.append(vector)

        return unoccupiedDistanceVectors
    
    def check_unoccupied_directions_vectors(self, unoccupiedDirectionsVectors: list[np.ndarray]):
        """
            Generate waypoint which is unexplored and further away from previously
            completed waypoints in the unoccupied directions list (validate a waypoint)

            Params: 
                unoccupiedDirectionsVectors -> a list of angles in which there are no obstacles 

            Returns: 
                A PoseStamped waypoint for robot to travel towards  
        """

        lowestCost = None
        waypoint = EMPTY_WAYPOINT
        
        for unoccupiedDirectionVector in unoccupiedDirectionsVectors:
            i = 0
            for i, completedWaypointVector in enumerate(self.LOCAL_completedWaypointVectors):
                robotPoseVector = np.array(([self.S_Odom.get_X(), self.S_Odom.get_Y()]))
                # get distance from robots current pose to a completeed waypoint 
                distanceFromCompletedWaypoint = np.linalg.norm(completedWaypointVector - robotPoseVector)
                # distance from an unoccupied direction to completed waypoint 
                distanceFromNewWaypointVector = np.linalg.norm(completedWaypointVector - unoccupiedDirectionVector)

                # Check if new waypoint will take the robot further away from the previous comleted waypoints
                if (distanceFromCompletedWaypoint - distanceFromNewWaypointVector) > 0:
                    # If this waypoint brings the robot closer to a previous waypoint, do not use this waypoint
                    break
            
            if (i == len(self.LOCAL_completedWaypointVectors)-1):
                # If this waypoint takes the robot further away from every previous waypoint, create a waypoint to publish to goal_pose topic
                waypointX, waypointY = self.get_vector_data(unoccupiedDirectionVector)

                # Check to see if waypoint is in an unexplored area
                waypointValid, cost = self.check_waypoint(waypointX, waypointY, False)

                # Check cost of waypoint 
                if (waypointValid):
                    # if lowest cost of waypoint is 0 create a new waypoint 
                    if lowestCost == None:
                        lowestCost = cost
                        waypoint = unoccupiedDirectionVector
                    else:
                        # if cost of waypoint is less than lowest cost create a new waypoint 
                        if cost < lowestCost:
                            lowestCost = cost
                            waypoint = unoccupiedDirectionVector

        return waypoint

    def create_waypoint_vector(self, x, y):
        return np.array([x, y])
    
    def get_vector_data(self, vector):
        return vector[0], vector[1]
    
    def create_waypoint(self, x: float, y: float, w: float) -> PoseStamped:
        """
            Creates a waypoint that can be puplished to the goal_pose topic
            
            Params:
                x -> x position in reference to initial pose (meters)
                y -> y position in reference to its intial pose (meters)
                w -> angle in reference to its intial pose (rads)

            Returns:
                PoseStamp object waypoint that can be published to goal_pose
        """

        point = PoseStamped() # set up a point with a PoseStamped type 
        point.header.frame_id = "map" #
        point.pose.position.x = x # pose of point in x 
        point.pose.position.y = y # pose of point in y 
        point.pose.orientation.w = w # quaternion angle of the point 

        return point
    
    def find_open_frontiers(self, radius: int) -> list[np.ndarray]:
        """
            Finds open frontiers within a specified radius.

            Params:
                radius (int): The radius within which to search for open frontiers.
                            This defines the range in the occupancy map to search 
                            for valid frontier points.

            Returns:
                list[np.ndarray]: A list of NumPy arrays representing valid frontiers 
                                (waypoints) detected within the specified radius.
        """
        openFrontiers = []

        #Iterate through the occupancy grid data and check for valid waypoints
        for index in range(len(self.S_Map_Occupancy.get_data_array())):
            x, y = self.S_Map_Occupancy.array_index_to_cartesian(index) # convert occupancy grid index to cartesian co-ordinates 
            waypoint = self.create_waypoint_vector(x, y)
            waypointValid, cost = self.check_waypoint(x, y, True)

            if waypointValid:
                openFrontiers.append(waypoint) #Append valid waypoints to open frontiers

        return openFrontiers

    def bubble_sort_frontiers_by_distance_from_robot(self, openFrontiers: list[np.ndarray]) -> list[np.ndarray]:
        """
            Sorts the list of open frontiers based on their distance from the robot's current position using bubble sort.

            Params:
                openFrontiers (list[np.ndarray]): A list of NumPy arrays representing open frontiers (waypoints) 
                                                that need to be sorted based on their distance from the robot.

            Returns:
                list[np.ndarray]: A sorted list of open frontiers by their distance from the robot, 
                                with the closest frontier first.
        """
        #Inner loop to compare adjacent elements
        for n in range(len(openFrontiers) - 1, 0, -1):
            for i in range(n):
                waypoint1Norm = np.linalg.norm(np.abs(openFrontiers[i] - self.robotPoseVector))
                waypoint2Norm = np.linalg.norm(np.abs(openFrontiers[i+1] - self.robotPoseVector))

                if waypoint1Norm > waypoint2Norm:
                    openFrontiers[i], openFrontiers[i+1] = openFrontiers[i+1], openFrontiers[i]

        return openFrontiers
    
    
    def choose_frontier(self):
        """
            Chooses the next frontier waypoint for the robot to move to. If no open frontiers exist, it searches the occupancy grid.
            
            Returns:
                waypoint: The chosen frontier waypoint as a vector.
        """
        waypoint = EMPTY_WAYPOINT

        #If there are no existing frontiers in the list
        if self.openFrontiers == []:
            log("INFO", self, "No existing frontiers: searching Occupncy grid...", self.debug)
            #Find new open frontiers within a specified radius (e.g., 5 units)
            self.openFrontiers = self.find_open_frontiers(5)
            log("INFO", self, "Done!", self.debug)
            waypoint = self.choose_frontier()
        else:
            log("INFO", self, "Searching through existing frontiers", self.debug)
            #Sort the frontiers based on their distance from the robot's current position
            self.openFrontiers = self.bubble_sort_frontiers_by_distance_from_robot(self.openFrontiers)

            frontierUsedArrayIndex = 0
            #Iterate through the sorted open frontiers to find a valid waypoint
            for frontierUsedArrayIndex, frontier in enumerate(self.openFrontiers):
                x, y = self.get_vector_data(frontier) #Get the x and y values of the frontier
                waypointValid, cost = self.check_waypoint(x, y, True) #Check if the waypoint is valid

                if waypointValid:
                    #If valid, set it as the current waypoint and save its cost
                    lowestCost = cost
                    waypoint = frontier

                    #Search around the grid cell for a potentially better waypoint within a 6-cell radius
                    for index in self.S_Map_Occupancy.search_around_grid_cell(x, y, 6):
                        x, y = self.S_Map_Occupancy.array_index_to_cartesian(index)
                        waypointValid, cost = self.check_waypoint(x, y, True)

                        if waypointValid:
                            #If a new valid waypoint has a lower cost, update the current waypoint and cost
                            if cost < lowestCost:
                                lowestCost = cost
                                waypoint = self.create_waypoint_vector(x, y)
                    break
            #After finding a valid frontier, update the list of remaining frontiers
            if frontierUsedArrayIndex < (len(self.openFrontiers) - 1):
                #Remove the used frontier and keep the remaining ones
                self.openFrontiers = self.openFrontiers[(frontierUsedArrayIndex + 1):]
            else:
                #If all frontiers have been used, clear the list and search for new frontiers
                self.openFrontiers = []
                waypoint = self.choose_frontier() #Recursively search for a new frontier if needed

        return waypoint #Return the chosen waypoint
                

    def chooseWaypoint(self, previousPathFailed: bool) -> None:
        """
        Chooses waypoints based on unoccupied directions (free angles and positions) during robots laser scan

        Params:
            previousPathFailed -> was the previous waypoint not successful
        
        Returns: 
            none 
        """
    
        waypoint = EMPTY_WAYPOINT # Chosen Waypoint
        frontierSearch = False # serach for frontier or angle

        if self.S_Aruco.totalArucoMarkers == self.totalArucoMarkers:
            log("INFO", self, "All aruco markers found:", True)
            self.S_Aruco.print_aruco_poses()
            raise SystemExit

        self.robotPoseVector = np.array([self.S_Odom.get_X(), self.S_Odom.get_Y()])

        if (self.waypointCounter != 0):
            # Add robots current position to the local completed waypoints
            self.LOCAL_completedWaypointVectors.append(self.robotPoseVector)

            # If waypoint failed, add this waypoint to failedWaypointsVectors array
            if previousPathFailed == True:
                log("INFO", self, "FAILED", self.debug)
                self.failedWaypointsVectors.append(np.array(self.currentWaypoint))
             
             # If waypoint was a success, add waypoint to global completed waypoints
            else:
                log("INFO", self, "SUCCESS", self.debug)
                self.GLOBAL_completedWaypointVectors.append(np.array(self.currentWaypoint))
        
        # If robot has more than 5 completed waypoints, update completedWaypointVectors list
        if len(self.LOCAL_completedWaypointVectors) > 5:
            self.LOCAL_completedWaypointVectors = self.LOCAL_completedWaypointVectors[1:]

        #check if robot is looking for frontier
        if frontierSearch:
            currentX = self.S_Odom.get_X() 
            currentY = self.S_Odom.get_Y()
            # add robots pose to list of completed waypoints 
            self.LOCAL_completedWaypointVectors = [np.array([currentX, currentY])]
            frontierSearch = False # stop doing a frontier search 

        ### SECTION 1)
            # choosing waypoints based on unoccupied directions and distances from laser scan data 
            # a valid waypoint is chosen if it is unexplored and some distance away from previous 5 completed waypoints 

        # find all free angles
        unoccupiedDirections = self.find_unoccupied_directions()
        # Calculate position for robot to in unoccupied direction
        unoccupiedDirectionsVectors = self.calculate_unoccupied_directions_vectors(unoccupiedDirections)
        # check if position vector is a valid waypoint 
        waypoint = self.check_unoccupied_directions_vectors(unoccupiedDirectionsVectors)

       
        ### SECTION 2)
            # choose waypoints based on unexplored frontiers and cost map data
            # a valid waypoint is chosen if it is at an unexplored frontier with a low cost 

        if (waypoint == EMPTY_WAYPOINT).all():
        # if there are no valid waypoints from laser scan data, send waypoint to unexplored frontier 
            log("INFO", self, "Stuck, finding unexplored frontier... ", self.debug)
            frontierSearch = True
            waypoint = self.choose_frontier()


        if (waypoint != EMPTY_WAYPOINT).all():
            # if waypoint is valid, set this waypoint to be current waypoint 
            self.currentWaypoint = waypoint 
            self.send_waypoint(waypoint) # publish current waypoint to goal_pose 
            self.waypointCounter += 1 # update waypoint count by 1 
        else:
            # if there are no more waypoints for robot to travel to, map is fully explored, send robot back to origin
            log("INFO", self, "Map fully explored ", self.debug)

    def check_waypoint(self, x: float, y: float, stuck):
        """
            checks waypoint is valid 

            Params: 
                x -> pose of robot in x 
                y -> pose of robot in y 
                stuck -> robot cannot find an unoccupied direction to head in 
            
            Returns: 
                bool type object for waypoint (i.e. if waypoint is valid or not) 
        """
        waypointValid = False

        closeToFailed = False


        closeToFailed = False #########
        for waypoint in self.failedWaypointsVectors:
            # if distance between failed waypoint and robots current pose is < 0.1, waypoint is too close, waypoint almost failed 
            if abs(x - waypoint[0]) < 0.1 and abs(y - waypoint[1]) < 0.1:
                closeToFailed = True
                break
            
        closeToCompleted = False  
        for waypoint in self.GLOBAL_completedWaypointVectors:
            # if distance between completed waypoint and robots current pose is < 0.1, waypoint was almost reached
            if abs(x - waypoint[0]) < 0.1 and abs(y - waypoint[1]) < 0.1:
                closeToCompleted = True
                break

        cost = self.S_Map_Global_Cost.average(x, y, 5) # get average cost of robots pose and 5 nearest points 
        
        if not stuck:
            notExplored = (self.S_Map_Occupancy.average(x, y, self.SEARCH_RADIUS) == -1)
            # a waypoint is valid if it is unexplored and not close to failing and is almost reached 
            waypointValid = (notExplored == True) and (closeToFailed != True) and (closeToCompleted != True)
        
        else: 
            # get percentage of grid cell which is free and percent that is unexplored 
            onFrontier = self.S_Map_Occupancy.is_point_on_frontier(x, y, self.SEARCH_RADIUS)
            # checks if waypoint is at a frontier
            # checks if waypoint is valid 
            waypointValid = (closeToFailed != True) and (closeToCompleted != True) and (onFrontier == True)

        return waypointValid, cost
                

    
    def send_waypoint(self, waypoint: np.ndarray) -> None:
        """
            Publishes a waypoint to the goal_pose topic
            
            Params:
                waypoint: The waypoint in which the robot will travel to
        """

        # log the waypoint that robot will travel to and its status (was it reached or not)
        log("INFO", self, f"Sending Waypoint: x -> {waypoint[0]:.4f}, y -> {waypoint[1]:.4f} -- Status: ", self.debug)
        waypointPoseStamped = self.create_waypoint(waypoint[0], waypoint[1], math.atan2(waypoint[1], waypoint[0]))
        self.currentWaypoint = waypoint # set current waypoint to the waypoint published to goal_pose 
        self.GoalPose_Publisher.publish(waypointPoseStamped) # publish this waypoint to goal_pose topic


def log(level:str, node:Node, message:str, debug: bool) -> None:
    """
        Logs a message to the corresponding ros2 logger (info or warning logger)


        Params:
            level -> which logger to send message to
                    INFO: get_logger().info(...)
                    WARN: get_logger().warning(...)
            node -> node that will send the message
            messgae -> string to be sent to the logger
            debug -> True if messages should be printed

    """
    if debug:
        if level == "WARN":
            node.get_logger().warning(message) # log a message with WARN severity 
        elif level == "INFO":
            node.get_logger().info(message) # log a message with INFO severity 


def main(args=None):
    """
        Entry Point
    """
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = "[{severity}] [{time}]: {message}"
    
    try:
        rclpy.init(args=args) 
        map_explorer = Explorer() 
        rclpy.spin(map_explorer) 
       
    except (KeyboardInterrupt):
        log("WARN", map_explorer, "Exiting (Ctrl^c)", True) ##
        
    except (SystemExit):
        log("WARN", map_explorer, "Exiting (Finished)", True)
    

if __name__ == "__main__":
    main()
