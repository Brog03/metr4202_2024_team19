from sensor_msgs.msg import LaserScan
from rclpy.node import Node


"""
    Subscription to the LaserScan topic
"""
class LaserScan_Reader(object):
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
        self.subscription_laser = node.create_subscription(
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
    
    
    


