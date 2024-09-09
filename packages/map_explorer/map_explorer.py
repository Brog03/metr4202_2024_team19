import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from map_explorer.submodules.Odom_Reader import Odom_Reader
from map_explorer.submodules.LaserScan_Reader import LaserScan_Reader
from map_explorer.submodules.BehaviourTreeLog_Handle import BehaviourTreeLog_Handler

import math

CONSTANT_PI = math.pi

ANGLE_WIDTH_DEG = 10
ANGLE_WIDTH_RAD = ANGLE_WIDTH_DEG*(CONSTANT_PI/180.)

NUM_DIRECTIONS = int(360/ANGLE_WIDTH_DEG)

DISTANCE_STEP = 2
LASER_SCAN_MIN_RANGE = 25

deg_to_rad = lambda x : x*(CONSTANT_PI/180.)
rad_to_deg = lambda x : x*(180./CONSTANT_PI)

"""
    How the robot chooses waypoints

    This node is setup to be subscribed to the following topics:
        /odom
        /scan
        /behaviour_tree_log
    
    and to publish to the follwoing topics:
        /goal_pose

"""
class Explorer(Node):
    odom_data = None
    laser_data = None
    behaviourTree = None

    completed_waypoints = []

    def __init__(self):
        """
            Initialises the Explorer Node
        """

        super().__init__("map_explorer")

        # Function handlers
        FUNCTION_HANDLERS = [
            ("NavigateRecovery", "IDLE", self.chooseWaypoint) # If NavigateRecovery is in IDLE state, run self.chooseWaypoint
        ]

        self.odom_data = Odom_Reader(self) # Setup /odom subscription
        self.laser_data = LaserScan_Reader(self) # Setup /scan subscription
        self.behaviourTree = BehaviourTreeLog_Handler(self, FUNCTION_HANDLERS) # Setup /behaviour_tree_log subscription

        self.publisher = self.create_publisher(
            PoseStamped,
            "goal_pose",
            10
        )

        self.completed_waypoints.append(self.create_waypoint(0., 0., 0.))

    def getScanAverage(self, startIndex: int, endIndex: int) -> float:
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
            dataSum += self.laser_data.data[i]

        return dataSum/(endIndex-startIndex)

    def find_empty_space_angles(self) -> list:
        emptySpaceAngles = []
        subScanSize = int(len(self.laser_data.data)/NUM_DIRECTIONS)
        for i in range(NUM_DIRECTIONS):
            freeAngle = 0
            bot_angle = 0

            startIndex = i*subScanSize 
            endIndex = startIndex + subScanSize - 1

            avg_range = self.getScanAverage(startIndex, endIndex)

            if i*ANGLE_WIDTH_DEG > 180:
                freeAngle = (-1.)*(deg_to_rad(360.) - ANGLE_WIDTH_RAD*(i-0.5))
            else:
                freeAngle = ANGLE_WIDTH_RAD*(i-0.5)
            
            bot_angle = self.odom_data.w + freeAngle
            
            if avg_range >= LASER_SCAN_MIN_RANGE:
                emptySpaceAngles.append(bot_angle)

        return emptySpaceAngles

    def callculate_empty_waypoints(self, emptySpaceAngles):
        emptyWaypoints = []

        for angle in emptySpaceAngles:
            x = self.odom_data.x
            y = self.odom_data.y
            
            newX = x + math.sin(angle+(CONSTANT_PI/2.))*DISTANCE_STEP
            newY = y + math.sin(angle)*DISTANCE_STEP

            waypoint = self.create_waypoint(newX, newY, angle)
            emptyWaypoints.append(waypoint)

        return emptyWaypoints

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

    def chooseWaypoint(self) -> PoseStamped:
        """
            
        """
        completedWaypoint = self.create_waypoint(self.odom_data.x, self.odom_data.y, self.odom_data.w)
        self.completed_waypoints.append(completedWaypoint)

        emptySpaceAngles = self.find_empty_space_angles()
        emptyWaypoints = self.callculate_empty_waypoints(emptySpaceAngles)
        waypoints = []

        for emptWaypoint in emptyWaypoints:
            count = 0
            for completedWaypoint in self.completed_waypoints:
                previousX = completedWaypoint.pose.position.x
                previousY = completedWaypoint.pose.position.y

                nextX = emptWaypoint.pose.position.x
                nextY = emptWaypoint.pose.position.y
                nextW = emptWaypoint.pose.orientation.w

                vectorX = previousX - nextX
                vectorY = previousY - nextY
                vectorW = math.atan((vectorY)/(vectorX))

                if vectorX < 0 and vectorY < 0:
                    vectorW = deg_to_rad(90) + vectorW
                elif vectorX < 0 and vectorY > 0:
                    vectorW = (-1.)*(deg_to_rad(90) + vectorW)
                
                if abs(nextW - vectorW) > 0.45:
                    count += 1
                else:
                    break

            if count == len(self.completed_waypoints):
                self.send_waypoint(emptWaypoint)
                break

    
    def send_waypoint(self, waypoint):
        self.publisher.publish(waypoint)
    


    



def main(args=None):
    rclpy.init(args=args)

    map_explorer = Explorer()

    rclpy.spin(map_explorer)

if __name__ == "__main__":
    main()