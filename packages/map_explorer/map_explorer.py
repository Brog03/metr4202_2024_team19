import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from map_explorer.submodules.Odom_Reader import Odom_Reader
from map_explorer.submodules.LaserScan_Reader import LaserScan_Reader
from map_explorer.submodules.BehaviourTreeLog_Handle import BehaviourTreeLog_Handler

ANGLE_WIDTH_RAD = 0.174533
ANGLE_WIDTH = 10
NUM_DIRECTIONS = int(360/ANGLE_WIDTH)

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
        subScanSize = int(len(self.laser_data.data)/NUM_DIRECTIONS)
        for i in range(NUM_DIRECTIONS):
            startIndex = i*subScanSize 
            endIndex = startIndex + subScanSize - 1

            avg = self.getScanAverage(startIndex, endIndex)
            
            if avg >= 10:
                print("Can go in direction: ", i*ANGLE_WIDTH)
        print(self.odom_data)


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



def main(args=None):
    rclpy.init(args=args)

    map_explorer = Explorer()

    rclpy.spin(map_explorer)

if __name__ == "__main__":
    main()