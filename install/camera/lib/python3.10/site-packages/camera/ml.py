import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraSubscriber(Node):
    """
    A ROS2 node that subscribes to sensor_msgs/msg/Image messages and displays them.
    """
    def __init__(self):
        # Initialize the node with the name 'camera_subscriber'
        super().__init__('ml_node')
        # Create a subscriber to the 'image_raw' topic
        # The callback function 'listener_callback' will be called when a message is received
        # The queue size of 10
        self.subscription = self.create_subscription(Image,'image_raw',self.listener_callback,10)
        
        self.subscription # Prevent unused variable warning
        # Initialize CvBridge for converting between ROS and OpenCV images
        self.bridge = CvBridge()
        self.get_logger().info('Camera subscriber node started.')

    def listener_callback(self, msg):
        """
        Callback function executed when an Image message is received.
        """
        self.get_logger().info('Receiving image frame')
        try:
            # Convert the ROS Image message back to an OpenCV image (NumPy array)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # Display the image in a window named 'Camera Feed'
        cv2.imshow("Camera Feed", cv_image)
        # Wait for 1 millisecond for a key event (necessary for imshow to work)
        cv2.waitKey(1)

    def destroy_node(self):
        """
        Destroy the OpenCV window when the node is destroyed.
        """
        cv2.destroyAllWindows() # Close all OpenCV windows
        super().destroy_node()

def main(args=None):
    """
    Main function for the camera subscriber node.
    """
    rclpy.init(args=args) # Initialize ROS2 client library
    camera_subscriber = CameraSubscriber() # Create an instance of the CameraSubscriber node
    try:
        rclpy.spin(camera_subscriber) # Keep the node alive until it's explicitly shut down
    except KeyboardInterrupt:
        pass # Handle Ctrl+C gracefully
    finally:
        camera_subscriber.destroy_node() # Clean up resources
        rclpy.shutdown() # Shut down ROS2 client library

if __name__ == '__main__':
    main()