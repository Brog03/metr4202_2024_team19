import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from nav2_msgs.msg import BehaviorTreeLog
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid

import numpy as np
import math

FUNCTION_HANDLE_INDEX_ARGUMENT = 3
FUNCTION_HANDLE_INDEX = 2
NODE_STATUS_INDEX = 1
NODE_NAME_INDEX = 0

CONSTANT_PI = math.pi

ANGLE_WIDTH_DEG = 10
ANGLE_WIDTH_RAD = ANGLE_WIDTH_DEG*(CONSTANT_PI/180.)

NUM_DIRECTIONS = int(360/ANGLE_WIDTH_DEG)

DISTANCE_STEP = 2
LASER_SCAN_MIN_RANGE = 50

deg_to_rad = lambda x : x*(CONSTANT_PI/180.)
rad_to_deg = lambda x : x*(180./CONSTANT_PI)

        

class BehaviourTreeLog_Handler(object):
    """
    Subscription to the Behaviour Tree Log
    """
    subscription = None
    functionHandlers = None

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

        self.subscription
    
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
    


class LaserScan_Reader(object):
    """
    Subscription to the LaserScan topic
    """
    
    subscription = None

    data = 0
    min_angle = 0 
    max_angle = 0
    angle_inc = 0

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

        self.subscription

    def callback(self, msg: LaserScan) -> None:
        """
            Call back for laserScan Subscription - reads the current scan data

            Params:
                msg -> conatins data of the laser scan
        """
        self.data = msg.ranges
        self.min_angle = msg.angle_min
        self.max_angle = msg.angle_max
        self.angle_inc = msg.angle_increment

    def get_Data(self) -> list[float]:
        return self.data
        
    
class Odom_Reader(object):
    """
    Subscription to the Odom topic
    """
    
    subscription = None

    w = 0
    x = 0
    y = 0

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

        self.subscription
    
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
    
class OccupancyGrid_Reader(object):
    """
    Subscription to the LaserScan topic
    """
    
    subscription = None

    mapData = []

    resolution = None
    startX = None
    startY = None
    dimX = None
    dimY = None

    averageSquareSize = 5
    endX = None
    endY = None

    def __init__(self, node: Node) -> None:
        """
            Sets up a subscription to the /scan topic

            params:
                node -> Node that is subscribing to the topic
        """
        self.subscription = node.create_subscription(
            OccupancyGrid, 
            "map",
            self.callback,
            10
        )

        self.subscription

    def callback(self, msg: OccupancyGrid) -> None:
        """
            Call back for laserScan Subscription - reads the current scan data

            Params:
                msg -> conatins data of the laser scan
        """

        self.dimX = msg.info.width
        self.dimY = msg.info.height
        self.mapData = msg.data
        self.resolution = msg.info.resolution

        self.startX = msg.info.origin.position.x
        self.startY = msg.info.origin.position.y

        self.endX = self.startX + (self.dimX*self.resolution)
        self.endY = self.startY + (self.dimY*self.resolution)

    def array_index_to_cartesian(self, index: int) -> tuple[int, int]:
        """
            Converts an array index from the 1D array (map data) to cartesian coordinates

            Params:
                index -> index to be converted to cartesian coordiantes

            Returns:
                x and y value of cvartesian coordinate
        """
        cartesianX = self.startX + (index % self.dimX)*self.resolution
        cartesianY = self.startY + (index // self.dimX)*self.resolution

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
            index += abs(((x - self.startX)//self.resolution))
            index += abs(((y - self.startY)//self.resolution)*self.dimX)
            index = int(index)

        return index

    def get_Data(self) -> list[float]:
        return self.mapData
    
    def cartesian_coordinate_in_occupancy_grid(self, x, y):
        return (x > self.startX and x < self.endX and y > self.startY and y < self.endY)

    
    def average(self, x: float, y: float) -> float:
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
        for xSearch in range(-self.averageSquareSize, self.averageSquareSize):
            for ySearch in range(-self.averageSquareSize, self.averageSquareSize):
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

        return average/count


class Explorer(Node):
    """
    How the robot chooses waypoints

    This node is setup to be subscribed to the following topics:
        /odom
        /scan
        /behaviour_tree_log
    
    and to publish to the follwoing topics:
        /goal_pose

    """

    S_Odom = None
    S_Scan = None
    S_BehaviourTree = None

    waypointCounter = 0

    completedWaypointVectors = []
    failedWaypoints = []

    currentWaypoint = None

    def __init__(self):
        """
            Initialises the Explorer Node
        """

        super().__init__("map_explorer")

        # Function handlers
        FUNCTION_HANDLERS = [
            ("NavigateRecovery", "SUCCESS", self.chooseWaypoint, False), 
            ("NavigateRecovery", "FAILURE", self.chooseWaypoint, True) 
        ]   

        # Setup subscription
        self.S_Odom = Odom_Reader(self) # Setup /odom subscription
        self.S_Scan = LaserScan_Reader(self) # Setup /scan subscription
        self.S_Map = OccupancyGrid_Reader(self)
        self.S_BehaviourTree = BehaviourTreeLog_Handler(self, FUNCTION_HANDLERS) # Setup /behaviour_tree_log subscription

        # Setup publishers
        self.publisher = self.create_publisher(
            PoseStamped,
            "goal_pose",
            10
        )

        self.completedWaypointVectors.append(np.array([0, 0]))

    def get_scan_average(self, startIndex: int, endIndex: int) -> float:
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
                freeAngle = (-1.)*(deg_to_rad(360.) - ANGLE_WIDTH_RAD*(i-0.5))
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

        # Go through each angle and calculate where the robot's new X and Y will be
        for angle in unoccupiedDirections:
            x = self.S_Odom.get_X()
            y = self.S_Odom.get_Y()
            
            newX = x + math.sin(angle+(CONSTANT_PI/2.))*DISTANCE_STEP
            newY = y + math.sin(angle)*DISTANCE_STEP

            vector = np.array(([newX, newY]))
            unoccupiedDistanceVectors.append(vector)

        return unoccupiedDistanceVectors
    

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
            Chooses the robots next waypoint based on these rules:

            1) If this is the robots first waypoint, chose a random waypoint that is unobstructed
            2) If this is not the robots first waypoint, the next waypoint must take it further away from the previous 5 waypoints
            3) If the robot cannot chose a point that satisfys 3, back track to the previous waypoint and continue 3
        """
        waypoint = None
        stuck = False

        if previousPathFailed == True:
            print("FAILED")
            currentX = self.currentWaypoint.pose.position.x
            currentY = self.currentWaypoint.pose.position.y
            self.failedWaypoints.append((currentX, currentY))
        else:
            print("SUCCESS")

        # Robots first waypoint
        if (self.waypointCounter != 0):
            robotPoseVector = np.array(([self.S_Odom.get_X(), self.S_Odom.get_Y()]))
            self.completedWaypointVectors.append(robotPoseVector)
        
        if len(self.completedWaypointVectors) > 5:
            self.completedWaypointVectors = self.completedWaypointVectors[1:]

        if stuck:
            currentX = self.S_Odom.get_X()
            currentY = self.S_Odom.get_Y()
            self.completedWaypointVectors = [np.array([currentX, currentY])]

            stuck = False

        # Calculate free angles
        unoccupiedDirections = self.find_unoccupied_directions()
        # Calculate position vectors
        unoccupiedDirectionVectors = self.calculate_unoccupied_directions_vectors(unoccupiedDirections)

        # Go thorugh each of these position vectors
        for unoccupiedDirectionVector in unoccupiedDirectionVectors:
            i = 0
            for i, completedWaypointVector in enumerate(self.completedWaypointVectors):
                robotPoseVector = np.array(([self.S_Odom.get_X(), self.S_Odom.get_Y()]))
                distanceFromCompletedWaypoint = np.linalg.norm(completedWaypointVector - robotPoseVector)
                distanceFromNewWaypointVector = np.linalg.norm(completedWaypointVector - unoccupiedDirectionVector)

                # Check to see if the new position vector will take the robot further away from the previous comleted waypoints
                if (distanceFromCompletedWaypoint - distanceFromNewWaypointVector) > 0:
                    # If this waypoint brings the robot closer to a previous waypoint, do not use this waypoint
                    break
            
            if (i == len(self.completedWaypointVectors)-1):
                # If this waypoint takes the robot further away from every previous waypoint, create a waypoint to publish to goal_pose topic
                waypointX = unoccupiedDirectionVector[0]
                waypointY = unoccupiedDirections[1]

                if (self.S_Map.cartesian_coordinate_in_occupancy_grid(waypointX, waypointY)):
                    if (self.S_Map.average(waypointX, waypointY) == -1):
                        waypoint = self.create_waypoint(unoccupiedDirectionVector[0], unoccupiedDirectionVector[1], self.S_Odom.get_W())
                        self.waypointCounter += 1
                        break

        if (waypoint == None):
            # If the robot could not find any waypoints
            print("Stuck, finding unexplored area... ", end="")
            stuck = True

            for index in range(len(self.S_Map.get_Data())):
                x, y = self.S_Map.array_index_to_cartesian(index)

                if self.S_Map.average(x, y) == -1 and self.check_waypoint(x, y):
                    currentW = self.S_Odom.get_W()
                    waypoint = self.create_waypoint(x, y, currentW)

                    print("Done!\n")
                    
                    break

        self.send_waypoint(waypoint)

    def check_waypoint(self, x, y):
        waypointValid = True
        for waypoint in self.failedWaypoints:
            if abs(x - waypoint[0]) < 0.1 and abs(y - waypoint[1]) < 0.1:
                waypointValid = False
                break

        return waypointValid
                

    
    def send_waypoint(self, waypoint: PoseStamped) -> None:
        """
            Publishes a waypoint to the goal_pose topic
            
            Params:
                waypoint: The waypoint in which the robot will travel to
        """

        print(f"Sending Waypoint: x -> {waypoint.pose.position.x:.4f}, y -> {waypoint.pose.position.y:.4f} -- Status: ", end="", flush=True)
        self.currentWaypoint = waypoint
        self.publisher.publish(waypoint)

        
        


def main(args=None):
    """
        Entry Point
    """
    rclpy.init(args=args)

    map_explorer = Explorer()

    rclpy.spin(map_explorer)

if __name__ == "__main__":
    main()