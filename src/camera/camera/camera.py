import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    """
    A ROS2 node that publishes camera frames as sensor_msgs/msg/Image messages.
    """
    def __init__(self):
        # Initialize the node with the name 'camera_publisher'
        super().__init__('camera_publisher')
        # Create a publisher for 'image_raw' topic, with a queue size of 10
        self.image_pub = self.create_publisher(Image, 'image', 10)
        # Set up a timer to publish images at approximately 30 Hz
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        # Initialize CvBridge for converting between OpenCV and ROS images
        self.bridge = CvBridge()
        # Open the default camera (index 0)
        self.cap = cv2.VideoCapture(2)

        # Check if the camera opened successfully
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera. Exiting.')
            rclpy.shutdown() # Shutdown ROS2 if camera fails to open
            exit()

        self.get_logger().info('Camera publisher node started and camera opened.')

    def timer_callback(self):
        """
        Callback function executed by the timer to capture and publish camera frames.
        """
        ret, frame = self.cap.read() # Read a frame from the camera

        if ret:
            # Convert the OpenCV frame (NumPy array) to a ROS Image message
            # The encoding 'bgr8' means 8-bit color image with Blue, Green, Red channels
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            # Publish the ROS Image message
            self.image_pub.publish(ros_image)
            self.get_logger().info('Publishing image frame')
        else:
            self.get_logger().warn('Failed to capture image frame.')

    def destroy_node(self):
        """
        Release the camera resource when the node is destroyed.
        """
        self.cap.release() # Release the camera
        super().destroy_node()

def main(args=None):
    """
    Main function for the camera publisher node.
    """
    rclpy.init(args=args) # Initialize ROS2 client library
    camera_publisher = CameraPublisher() # Create an instance of the CameraPublisher node
    try:
        rclpy.spin(camera_publisher) # Keep the node alive until it's explicitly shut down
    except KeyboardInterrupt:
        pass # Handle Ctrl+C gracefully
    finally:
        camera_publisher.destroy_node() # Clean up resources
        rclpy.shutdown() # Shut down ROS2 client library

if __name__ == '__main__':
    main()