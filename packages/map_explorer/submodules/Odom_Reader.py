from nav_msgs.msg import Odometry
import math

"""
    Subscription to the Odom topic
"""
class Odom_Reader(object):
    subscription = None

    w = 0
    x = 0
    y = 0

    def __init__(self, node):
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
    
    def euler_from_quaternion(self, quaterion: list[float, float, float, float]):
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


