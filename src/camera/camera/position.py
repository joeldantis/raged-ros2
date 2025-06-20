import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class PosiotionCalc(Node):
    """
    ROS Node that takes the position info from the farme and returns 
    1. Offset Angle
    2. Distance from Object once the angle from the is within a threshold
    """

    def __init__(self):
        # Initialize the node with the name 'ml'
        super().__init__('position')

        # Publishers

        # Subscribers
        self.pos_sub = self.create_subscription(Int32MultiArray,'position',self.position_callback,10)