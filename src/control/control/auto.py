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
        # Initialize the node with the name 'auto'
        super().__init__('auto')

        # Variables
        self.prev_detection = ""

        # Publishers
        self.control_pub = self.create_publisher(String, 'control',10)

        # Subscribers
        self.pos_sub = self.create_subscription(String,'position',self.position_callback,10)

    def position_callback(self, msg):
        global FOV_x
        dat = json.loads(msg.data)
        msg = String()
        
#        if self.prev_detection != dat['name']:
        angle = ((dat['obj_center'] - (dat['frame_width']/2)) / (dat['frame_width'] / 2)) * (FOV_x / 2)
        self.prev_detection = dat['name']
        msg.data = json.dumps([2,angle])
        
 #       else: angle = 0

        # Calculate pixel width
        # 7cm
        print(angle)

        if abs(angle) < 5:
            if dat['obj_width'] > 0:
                distance = (6.5 * 527.5) / dat['obj_width']
                #distance_text = f"{distance:.2f} cm"
                # delay = distance * 0.045
                msg.data = json.dumps([1, distance])

            else:
                #distance_text = "N/A"
                pass

        self.control_pub.publish(msg)
        print(msg.data)
        
'''        
# Frame % Distance Calculation

            if angle < 1:
            if dat['frame%'] >= 70:
                #collect
                msg.data = json.dumps([0,int(dat['bin_section'])])
            
            else:
                dist = 10 - dat['frame%']/10
                msg.data = json.dumps([1,dist])

        msg.data = json.dumps([0,int(dat['bin_section'])])
'''


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
