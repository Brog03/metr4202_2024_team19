import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from map_explorer.submodules.Odom_Reader import Odom_Reader
from map_explorer.submodules.LaserScan_Reader import LaserScan_Reader
from map_explorer.submodules.BehaviourTreeLog_Handle import BehaviourTreeLog_Handler

from map_explorer.submodules.Vector import *

import math

ANGLE_WIDTH_DEG = 10
ANGLE_WIDTH_RAD = ANGLE_WIDTH_DEG*(CONSTANT_PI/180.)

NUM_DIRECTIONS = int(360/ANGLE_WIDTH_DEG)

DISTANCE_STEP = 2
LASER_SCAN_MIN_RANGE = 50

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

    backTrack = False
    backTrack_index = -2

    completedWaypointVectors = []

    def __init__(self):
        """
            Initialises the Explorer Node
        """

        super().__init__("map_explorer")

        # Function handlers
        FUNCTION_HANDLERS = [
            ("NavigateRecovery", "IDLE", self.chooseWaypoint) # If NavigateRecovery is in IDLE state, run self.chooseWaypoint
        ]

        # Setup subscription
        self.S_Odom = Odom_Reader(self) # Setup /odom subscription
        self.S_Scan = LaserScan_Reader(self) # Setup /scan subscription
        self.S_BehaviourTree = BehaviourTreeLog_Handler(self, FUNCTION_HANDLERS) # Setup /behaviour_tree_log subscription

        # Setup publishers
        self.publisher = self.create_publisher(
            PoseStamped,
            "goal_pose",
            10
        )

        self.completedWaypointVectors.append(Vector(0, 0))

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

    def calculate_unoccupied_directions_vectors(self, unoccupiedDirections: list[float]) -> list[Vector]:
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

            vector = Vector(newX, newY)
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

    def chooseWaypoint(self) -> None:
        """
            Chooses the robots next waypoint based on these rules:

            1) If this is the robots first waypoint, chose a random waypoint that is unobstructed
            2) If this is not the robots first waypoint, the next waypoint must take it further away from the previous 5 waypoints
            3) If the robot cannot chose a point that satisfys 3, back track to the previous waypoint and continue 3
        """
        waypoint = None

        # Robots first waypoint
        if (self.waypointCounter != 0):
            robotPoseVector = Vector(self.S_Odom.get_X(), self.S_Odom.get_Y())
            self.completedWaypointVectors.append(robotPoseVector)
        
        if len(self.completedWaypointVectors) > 5:
            self.completedWaypointVectors = self.completedWaypointVectors[1:]

        # Calculate free angles
        unoccupiedDirections = self.find_unoccupied_directions()
        # Calculate position vectors
        unoccupiedDirectionVectors = self.calculate_unoccupied_directions_vectors(unoccupiedDirections)

        # Go thorugh each of these position vectors
        for unoccupiedDirectionVector in unoccupiedDirectionVectors:
            i = 0
            for i, completedWaypointVector in enumerate(self.completedWaypointVectors):
                robotPoseVector = Vector(self.S_Odom.get_X(), self.S_Odom.get_Y())
                distanceFromCompletedWaypoint = (completedWaypointVector - robotPoseVector).get_Magnitude()
                distanceFromNewWaypointVector = (completedWaypointVector - unoccupiedDirectionVector).get_Magnitude()

                # Check to see if the new position vector will take the robot further away from the previous comleted waypoints
                if (distanceFromCompletedWaypoint - distanceFromNewWaypointVector) > 0:
                    # If this waypoint brings the robot closer to a previous waypoint, do not use this waypoint
                    break
            
            if (i == len(self.completedWaypointVectors)-1):
                # If this waypoint takes the robot further away from every previous waypoint, create a waypoint to publish to goal_pose topic
                waypoint = self.create_waypoint(unoccupiedDirectionVector.get_X(), unoccupiedDirectionVector.get_Y(), self.S_Odom.get_W())
                self.waypointCounter += 1
                break

        if (waypoint != None):
            self.send_waypoint(waypoint)
        else:
            # If the robot could not find any waypoints
            print("Stuck")
    
    def send_waypoint(self, waypoint: PoseStamped) -> None:
        """
            Publishes a waypoint to the goal_pose topic
            
            Params:
                waypoint: The waypoint in which the robot will travel to
        """
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