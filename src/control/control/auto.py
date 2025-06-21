import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

global FOV_x
FOV_x = 84.0  # degrees

class Auto(Node):
    """
    ROS Node that takes the position info from the farme and returns 
    1. Offset Angle
    2. Distance from Object once the angle from the is within a threshold
    """

    def __init__(self):
        # Initialize the node with the name 'ml'
        super().__init__('auto')

        # Publishers

        # Subscribers
        self.pos_sub = self.create_subscription(String,'position',self.position_callback,10)

    def position_callback(self, msg):
        global FOV_x
        dat = json.loads(msg.data)
        print(dat)
        angle = ((dat['obj_width'] - dat['frame_width'] / 2) / (dat['frame_width'] / 2)) * (FOV_x / 2)

        if angle < 1:
            if dat['frame%'] >= 70:
                #collect
                pass

        pass

def main(args=None):
    """
    Main function for the position node.
    """
    rclpy.init(args=args) # Initialize ROS2 client library
    auto = Auto() # Create an instance of the position node
    try:
        rclpy.spin(auto) # Keep the node alive until it's explicitly shut down
    except KeyboardInterrupt:
        pass # Handle Ctrl+C gracefully
    finally:
        auto.destroy_node() # Clean up resources
        rclpy.shutdown() # Shut down ROS2 client library

if __name__ == '__main__':
    main()
