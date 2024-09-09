from nav2_msgs.msg import BehaviorTreeLog
from rclpy.node import Node

FUNCTION_HANDLE = 2
NODE_STATUS = 1
NODE_NAME = 0

"""
    Subscription to the Behaviour Tree Log
"""
class BehaviourTreeLog_Handler(object):
    subscription = None
    function_handlers = None

    def __init__(self, node: Node, function_handlers: list[tuple[str, str]]):
        """
        Sets up a subscription to the /behavior_tree_log topic

        params:
            node -> Node that is subscribing to the topic
            function_handlers -> An array of tuples that store what function should run based a node and its status
        """
        self.function_handlers = function_handlers

        self.subscription_bt = node.create_subscription(
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
            for handler in self.function_handlers:
                if event.node_name == handler[NODE_NAME] and event.current_status == handler[NODE_STATUS]:
                    handler[FUNCTION_HANDLE]()
    


