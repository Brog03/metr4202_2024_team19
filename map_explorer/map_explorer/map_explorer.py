import rclpy
import rclpy.logging
from rclpy.node import Node


from geometry_msgs.msg import PoseStamped
from nav2_msgs.msg import BehaviorTreeLog
import rclpy.subscription
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid

#from ros2_aruco_interfaces.msg import ArucoMarkers

import time
import numpy as np
import math
import os

FUNCTION_HANDLE_INDEX_ARGUMENT = 3
FUNCTION_HANDLE_INDEX = 2
NODE_STATUS_INDEX = 1
NODE_NAME_INDEX = 0

CONSTANT_PI = math.pi

ANGLE_WIDTH_DEG = 2
ANGLE_WIDTH_RAD = ANGLE_WIDTH_DEG*(CONSTANT_PI/180.)

NUM_DIRECTIONS = int(360/ANGLE_WIDTH_DEG)

DISTANCE_STEP = 2
LASER_SCAN_MIN_RANGE = 50

LAMBDA_deg_to_rad = lambda x : x*(CONSTANT_PI/180.)
LAMBDA_rad_to_deg = lambda x : x*(180./CONSTANT_PI)

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
            functionHandlers -> An array of tuples that store what function should run based a node and its status
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
    
    subscription = None

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
            LaserScan, #Message Type 
            "scan", #topic name 
            self.callback, #callback function 
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
        return self.data
        
    

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
            Odometry,
            "odom",
            self.callback, 
            10
        )
    
    def __repr__(self) -> str:
        return f"x: {self.x} y: {self.y} w: {self.w}"

    def callback(self, msg: Odometry) -> None:
        """
            Callback function for the topic - reads odom

            Params: 
                msg -> contains data of the robots odometry
        
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        self.w = self.euler_from_quaternion(msg.pose.pose.orientation)[2]
    
    def euler_from_quaternion(self, quaterion: list[float, float, float, float]) -> tuple[float, float, float]:
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)

            Params: 
                quaterion -> the robots orientation in the form of a quaternion
            
            Returns:
                roll, pitch and yaw (rads)
        
        """
        x = quaterion.x
        y = quaterion.y
        z = quaterion.z
        w = quaterion.w
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

    def get_X(self) -> float:
        return self.x
    
    def get_Y(self) -> float:
        return self.y
    
    def get_W(self) -> float:
        return self.w
    
class Subscriber_Map(object):
    subscription: rclpy.subscription = None

    mapData: list[float] = [] # Map data (int)
    resolution: float = None # Map resolution (meters)
    dimX: float = None # Map width (units)
    dimY: float = None # Map height (units)

    minX: float = None # Map lower x bound (meters)
    minY: float = None # Map lower y bound (meters)
    maxX: float = None # Map upper x bound (meters)
    maxY: float = None # Map upper y bound (meters)

    def __init__(self, node: Node, subsctription_: str) -> None:
        """
            Sets up a subscription to the topic

            params:
                node -> Node that is subscribing to the topic
        """

        self.subscription = node.create_subscription(
            OccupancyGrid, 
            subsctription_,
            self.callback,
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
            Converts an array index from the 1D array (map data) to cartesian coordinates

            Params:
                index -> index to be converted to cartesian coordiantes

            Returns:
                x and y value of cvartesian coordinate
        """
        cartesianX = self.minX + (index % self.dimX)*self.resolution # x coordinate from index
        cartesianY = self.minY + (index // self.dimX)*self.resolution # y coordinate from index

        return cartesianX, cartesianY
    
    def cartesian_to_array_index(self, x: float, y: float) -> int:
        """
            Converts cartesian coordinates to the corresponding cell inedx in the map data array

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
                x -> x value
                y -> y value

            Returns:
                True if point is contained otherwise false
        """
        return ((x > self.minX) and (x < self.maxX) and (y > self.minY) and (y < self.maxY))
    
    def get_data_array(self) -> list[float]:
        return self.mapData
    
    def get_data_with_cartesian(self, x: float, y: float) -> float:
        index = self.cartesian_to_array_index(x, y)
        if (index != None):
            return self.mapData[index]
        else:
            return 100

    def get_data_with_index(self, index: int) -> float:
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
            return average/count
        else:
            return None


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


    def get_surrounding_percentages(self, x: float, y: float, radius) -> tuple[float, float]:
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

                index = self.cartesian_to_array_index(newX, newY)

                if index != None:
                    if self.mapData[index] == -1:
                        countUnExplored += 1
                    elif self.mapData[index] == 0:
                        countFree += 1

                    countTotal += 1

        return countFree/countTotal, countUnExplored/countTotal
    
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
        

# class ArucoMarker_Subscriber(object):
#     """
#     Subscription to /aruco_markers
#     """
    
#     subscription: rclpy.subscription = None

#     def __init__(self, node: Node) -> None:
#         """
#             Sets up a subscription to the /aruco_markers topic

#             params:
#                 node -> Node that is subscribing to the topic
#         """
#         # Wait until there is a publisher to /aruco_markers
#         self.wait_aruco_node_running(node)
    
#         self.subscription = node.create_subscription(
#             ArucoMarkers, 
#             "aruco_markers",
#             self.callback,
#             10
#         )

#     def wait_aruco_node_running(self, node: Node):
#         """
#             Blocks until a aruco_node is publishing to /aruco_markers

#             params:
#                 node -> Node that is subscribing to the topic
#         """

#         while True:
#             # Get publishers of the /aruco_markers topic
#             aruco_marker_publishers = node.get_publishers_info_by_topic("/aruco_markers")

#             # Check who is currently publishing to this topic
#             if not aruco_marker_publishers:
#                 # If no publishers exist, block for another 5 seconds and try again
#                 log("INFO", node, "Aruco Node not running - Trying again in 5 seconds", True)
#                 time.sleep(5)

#             else:
#                 # A publisher exists
#                 publisher = aruco_marker_publishers[0]._node_name

#                 if publisher == "aruco_node":
#                     # If publish is aruco_node, then stop blocking and continue the program
#                     log("INFO", node, "Aruco Detection is Running, starting map_exporler node", True)
#                     break

#     def callback(self, msg: ArucoMarkers) -> None:
#         """
#             Call back for aruco_markers Subscription

#             Params:
#                 msg -> conatins data of the current poses and IDs of the detected aruco_marketrs
#         """
        
#         print(msg)



class Explorer(Node):
    """
    How the robot chooses waypoints

    This node is setup to be subscribed to the following topics:
        /odom
        /scan
        /behaviour_tree_log
        /map
        /aruco_markers
    
    and to publish to the follwoing topics:
        /goal_pose

    """

    SEARCH_RADIUS = 5

    S_Odom: Odom_Subscriber = None # Subscription to /odom
    S_Scan: LaserScan_Subscriber = None # Subscription to /scan
    S_BehaviourTree: BehaviourTreeLog_Handler = None # Subscription to /behaviour_tree_log
    S_Map_Occupancy: OccupancyGrid_Subscriber = None # Subscription to /map
    S_Map_Global_Cost: CostMap_Subscriber = None # Subscription to /map
    #S_Aruco: ArucoMarker_Subscriber = None # Subscription to /aruco_marker

    waypointCounter: int = 0 # Total waypoints published

    GLOBAL_completedWaypointVectors: list[np.ndarray] = [] # Position vectors of the completed waypoints
    LOCAL_completedWaypointVectors: list[np.ndarray] = [] # Position vectors of the completed waypoints
    failedWaypointsVectors: list[np.ndarray] = [] # Position vectors of the failed waypoints

    currentWaypoint: PoseStamped = None

    debug: bool = None

    def __init__(self):
        """
            Initialises the Explorer Node
        """

        super().__init__("map_explorer")

        # Declaring parameters loaded from params.yaml
        self.declare_parameters(
            namespace="",
            parameters=[
                ("aruco_detect", rclpy.Parameter.Type.BOOL),
                ("num_aruco_markers", rclpy.Parameter.Type.INTEGER), 
                ("debug", rclpy.Parameter.Type.BOOL)
            ]
        )


        rosParam_aruco_detect = self.get_parameter('aruco_detect').get_parameter_value().bool_value
        rosParam_num_aruco_markers = self.get_parameter('num_aruco_markers').get_parameter_value().integer_value
        rosParam_debug = self.get_parameter('debug').get_parameter_value().bool_value


        # Function handlers
        FUNCTION_HANDLERS = [
            ("NavigateRecovery", "SUCCESS", self.chooseWaypoint, False), # Call self.chooseWaypoint(False) if waypoint was reached
            ("NavigateRecovery", "FAILURE", self.chooseWaypoint, True) # Call self.chooseWaypoint(True) if waypoint wasn't reached
        ]   
        # Setup subscription

        self.S_Odom = Odom_Subscriber(self) # Setup /odom subscription
        self.S_Scan = LaserScan_Subscriber(self) # Setup /scan subscription
        self.S_Map_Occupancy = OccupancyGrid_Subscriber(self) # Subscription to /map
        self.S_BehaviourTree = BehaviourTreeLog_Handler(self, FUNCTION_HANDLERS) # Setup /behaviour_tree_log subscription
        self.S_Map_Global_Cost = CostMap_Subscriber(self)

        # Check if aruco_detect is True
        if rosParam_aruco_detect == True:
            log("WARN", self, "Running map_explorer with Aruco decetion", True)
            #self.S_Aruco = ArucoMarker_Subscriber(self) # Subscription to /aruco_marker 
        else:
            log("WARN", self, "Running map_explorer without Aruco decetion", True)
    
        # Create publisher to /goal_pose
        self.GoalPose_Publisher = self.create_publisher(
            PoseStamped,
            "goal_pose",
            10
        )

        self.LOCAL_completedWaypointVectors.append(np.array([0, 0]))

        if not rosParam_debug:
            log("INFO", self, "READY (Debug disabled)", True)
            self.debug = False
        else:
            log("INFO", self, "READY", True)
            self.debug = True

    def get_scan_average(self, startIndex: float, endIndex: float) -> float:
        """
            Gets the average value of the laserScan data between two indexes

            Params:
                startIndex -> first index in the laser_scan_data array
                endIndex -> last index in the laser_scan_data array

            Returns:
                Average value bewteen these two indexes
        """

        dataSum = 0
        for i in range(startIndex, endIndex):
            dataSum += self.S_Scan.data[i]

        return dataSum/(endIndex-startIndex)

    def find_unoccupied_directions(self) -> list:
        """
            Divides the /scan data up into ANGLE_WIDTH_RAD segments (e.g 10deg segments would have 36 total segments)
            and gets the average scan distance for each segment and checks to see if it can travel in the middle of that segment

            Returns:
                A list of angles (In reference to the point (0, 0, 0)) in which the robot can travel in
        """

        unoccupiedDirections = []
        subScanSize = int(len(self.S_Scan.get_Data())/NUM_DIRECTIONS)

        for i in range(NUM_DIRECTIONS):
            freeAngle = 0
            bot_angle = 0

            startIndex = i*subScanSize 
            endIndex = startIndex + subScanSize - 1

            avg_range = self.get_scan_average(startIndex, endIndex) # Get average distance

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
            Calculates the position vectors for where the robot will tarvel two based on DISTANCE_STEP

            Params:
                 unoccupiedDirections -> Angles in which there are no obstacles

            Returns:
                A list of the Vector class where the robot can travel
        """
        
        unoccupiedDistanceVectors = []

        x = self.S_Odom.get_X()
        y = self.S_Odom.get_Y()

        # Go through each angle and calculate where the robot's new X and Y will be
        for angle in unoccupiedDirections:            
            newX = x + math.cos(angle)*DISTANCE_STEP
            newY = y + math.sin(angle)*DISTANCE_STEP

            vector = np.array(([newX, newY]))
            unoccupiedDistanceVectors.append(vector)

        return unoccupiedDistanceVectors
    
    def check_unoccupied_directions_vectors(self, unoccupiedDirectionsVectors: list[np.ndarray]):
        # Go thorugh each of these position vectors
        lowestCost = None
        waypoint = None

        for unoccupiedDirectionVector in unoccupiedDirectionsVectors:
            i = 0
            for i, completedWaypointVector in enumerate(self.LOCAL_completedWaypointVectors):
                robotPoseVector = np.array(([self.S_Odom.get_X(), self.S_Odom.get_Y()]))
                distanceFromCompletedWaypoint = np.linalg.norm(completedWaypointVector - robotPoseVector)
                distanceFromNewWaypointVector = np.linalg.norm(completedWaypointVector - unoccupiedDirectionVector)

                # Check to see if the new position vector will take the robot further away from the previous comleted waypoints
                if (distanceFromCompletedWaypoint - distanceFromNewWaypointVector) > 0:
                    # If this waypoint brings the robot closer to a previous waypoint, do not use this waypoint
                    break
            
            if (i == len(self.LOCAL_completedWaypointVectors)-1):
                # If this waypoint takes the robot further away from every previous waypoint, create a waypoint to publish to goal_pose topic
                waypointX = unoccupiedDirectionVector[0]
                waypointY = unoccupiedDirectionVector[1]

                # Check to see if waypoint is in an unexplored area
                waypointValid, cost = self.check_waypoint(waypointX, waypointY, False)

                if (waypointValid):
                    if lowestCost == None:
                        lowestCost = cost
                        waypoint = self.create_waypoint(waypointX, waypointY, self.S_Odom.get_W())
                    else:
                        if cost < lowestCost:
                            lowestCost = cost
                            waypoint = self.create_waypoint(waypointX, waypointY, self.S_Odom.get_W())

        return waypoint

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

        point = PoseStamped()
        point.header.frame_id = "map"
        point.pose.position.x = x
        point.pose.position.y = y
        point.pose.orientation.w = w

        return point

    def chooseWaypoint(self, previousPathFailed: bool) -> None:
        """
            Description...

            Params:
                previousPathFailed -> was the previous waypoint not successful
        """
    
        waypoint = None
        stuck = False

        if (self.waypointCounter != 0):
            robotPoseVector = np.array(([self.S_Odom.get_X(), self.S_Odom.get_Y()]))
            self.LOCAL_completedWaypointVectors.append(robotPoseVector)

            if previousPathFailed == True:
                log("INFO", self, "FAILED", self.debug)
                currentX = self.currentWaypoint.pose.position.x
                currentY = self.currentWaypoint.pose.position.y
                self.failedWaypointsVectors.append(np.array([currentX, currentY]))
            else:
                log("INFO", self, "SUCCESS", self.debug)
                currentX = self.currentWaypoint.pose.position.x
                currentY = self.currentWaypoint.pose.position.y
                self.GLOBAL_completedWaypointVectors.append(np.array([currentX, currentY]))
        
        if len(self.LOCAL_completedWaypointVectors) > 5:
            self.LOCAL_completedWaypointVectors = self.LOCAL_completedWaypointVectors[1:]

        if stuck:
            currentX = self.S_Odom.get_X()
            currentY = self.S_Odom.get_Y()
            self.LOCAL_completedWaypointVectors = [np.array([currentX, currentY])]

            stuck = False

        # Calculate free angles
        unoccupiedDirections = self.find_unoccupied_directions()
        # Calculate position vectors
        unoccupiedDirectionsVectors = self.calculate_unoccupied_directions_vectors(unoccupiedDirections)

        waypoint = self.check_unoccupied_directions_vectors(unoccupiedDirectionsVectors)

        if (waypoint == None):
            # If the robot could not find any waypoints
            log("INFO", self, "Stuck, finding unexplored frontier... ", self.debug)

            stuck = True
            lowestCost = None

            for index in range(len(self.S_Map_Occupancy.get_data_array())):
                x, y = self.S_Map_Occupancy.array_index_to_cartesian(index)
                    
                waypointValid, cost = self.check_waypoint(x, y, True)

                # Check to see if point is on a frontier

                if waypointValid:
                    if abs(cost + 1) < 0.01:
                        waypoint = self.create_waypoint(x, y, self.S_Odom.get_W())
                        break
                    elif lowestCost == None:
                        lowestCost = cost
                        waypoint = self.create_waypoint(x, y, self.S_Odom.get_W())
                    elif (cost < lowestCost):
                        lowestCost = cost
                        waypoint = self.create_waypoint(x, y, self.S_Odom.get_W())

        
        if waypoint != None:
            self.currentWaypoint = waypoint
            self.send_waypoint(waypoint)
            self.waypointCounter += 1
        else:
            log("INFO", self, "Map fully explored ", self.debug)
            waypoint = self.create_waypoint(0., 0., 0.)
            self.send_waypoint(waypoint)

    def check_waypoint(self, x: float, y: float, frontier: bool):
        """
        Checks if waypoint chosen is valid and if it is a frontier 

        params: 
            x -> pose of robot x co-ordinate 
            y -> pose of robot y co-ordinate 
            frontier -> is chosen waypoint a frontier 

        Returns:
                whether or not waypoint is valid and the cost of the robots average pose 
        
        """
        waypointValid = False

        closeToFailed = False

        for waypoint in self.failedWaypointsVectors:
            if abs(x - waypoint[0]) < 0.1 and abs(y - waypoint[1]) < 0.1:
                closeToFailed = True
                break
            
        closeToCompleted = False
        for waypoint in self.GLOBAL_completedWaypointVectors:
            if abs(x - waypoint[0]) < 0.1 and abs(y - waypoint[1]) < 0.1:
                closeToCompleted = True
                break

        cost = self.S_Map_Global_Cost.average(x, y, 3)
        
        if not frontier:
            notExplored = (self.S_Map_Occupancy.average(x, y, self.SEARCH_RADIUS) == -1)

            waypointValid = (notExplored == True) and (closeToFailed != True) and (closeToCompleted != True)
        
        else:
            perecentFree, percentUnExplored = self.S_Map_Occupancy.get_surrounding_percentages(x, y, self.SEARCH_RADIUS)

            onFrontier = (abs(percentUnExplored - perecentFree) < 0.02) and (abs(percentUnExplored - 0.5) < 0.02)

            waypointValid = (closeToFailed != True) and (closeToCompleted != True) and (onFrontier == True)

        return waypointValid, cost
                

    
    def send_waypoint(self, waypoint: PoseStamped) -> None:
        """
            Publishes a waypoint to the goal_pose topic
            
            Params:
                waypoint: The waypoint in which the robot will travel to
        """


        log("INFO", self, f"Sending Waypoint: x -> {waypoint.pose.position.x:.4f}, y -> {waypoint.pose.position.y:.4f} -- Status: ", self.debug)
        self.currentWaypoint = waypoint
        self.GoalPose_Publisher.publish(waypoint)


def log(level:str, node:Node, message:str, debug: bool) -> None:
    """
        Logs a message to the corresponding ros2 logger (info or warning logger)


        Params:
            level -> which logger to send message to
                INFO: get_logger().info(...)
                WARN: get_logger().warning(...)
            node -> node that will send the message
            messgae -> string to be sent to the logger

    """
    if debug:
        if level == "WARN":
            node.get_logger().warning(message)
        elif level == "INFO":
            node.get_logger().info(message)

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
        log("WARN", map_explorer, "Exiting", True)
    

if __name__ == "__main__":
    main()