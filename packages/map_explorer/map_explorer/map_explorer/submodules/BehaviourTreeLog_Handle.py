from nav2_msgs.msg import BehaviorTreeLog
from rclpy.node import Node

FUNCTION_HANDLE_INDEX = 2
NODE_STATUS_INDEX = 1
NODE_NAME_INDEX = 0


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
                    (Node1 Name, Node1 Status, function), 
                    (..., ..., ...), 
                    ...
                ]
        """
        self.functionHandlers = functionHandlers

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
            for handler in self.functionHandlers:
                if event.node_name == handler[NODE_NAME_INDEX] and event.current_status == handler[NODE_STATUS_INDEX]:
                    handler[FUNCTION_HANDLE_INDEX]()
    


