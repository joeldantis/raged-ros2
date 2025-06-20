import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Int32MultiArray, String
from cv_bridge import CvBridge
import json
import cv2
from ultralytics import YOLO

# Load the YOLO instance segmentation model
global model
model = YOLO("/mnt/Storage/Hackathons/IIITM/raged-ros2/src/camera/camera/best.pt")  # Replace with your model path if custom

class ML(Node):
    """
    A ROS2 node that subscribes to sensor_msgs/msg/Image messages and displays them.
    """
    def __init__(self):

        # variables
        self.detect = 1

        # Initialize the node with the name 'camera_subscriber'
        super().__init__('ml')

        # Subscribers
        self.image_sub = self.create_subscription(Image,'image',self.image_callback,10)
        self.detect_sub = self.create_subscription(Int32, 'detect', self.detect_callback, 10)

        # Publishers
        self.pos_pub = self.create_publisher(Int32MultiArray, 'position', 10)
        self.class_pub = self.create_publisher(String, 'class_info', 10)    


        # Initialize CvBridge for converting between ROS and OpenCV images
        self.bridge = CvBridge()
#        self.get_logger().info('Camera subscriber node started.')
    
    def image_callback(self, msg):
        global model

        position = Int32MultiArray()
        class_info = String()

        """
        Callback function executed when an Image message is received.
        """
#        self.get_logger().info('Receiving image frame')
        try:
            # Convert the ROS Image message back to an OpenCV image (NumPy array)
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # Check detect flag
        if self.detect:
            # Detect in frame
            self.results = model.predict(source=self.cv_image, show=False, stream=True)
            info = self.get_largest()

            if info:
                self.detect = 0

                # position = [width, height, distance from center]
                position.data = [info['width'], info['height'], info['center_dist']]
                class_info.data = info['name']

                # publishing
                self.pos_pub.publish(position)
                self.class_pub.publish(class_info)

            # Display the image in a window named 'Camera Feed'
            cv2.imshow("Camera Feed", self.cv_image)
            # Wait for 1 millisecond for a key event (necessary for imshow to work)
            cv2.waitKey(1)


    def detect_callback(self,msg):
        """
        Sets the global detect variable
        1 -> Continue with detection
        0 -> Pause
        """
        self.detect = msg.data


    def get_largest(self):
        """
        Finds the bounding box with the largest area from the prediction results of a single frame.
        """
        largest_box = None
        max_area = 0
        for i in self.results:
            if i.boxes:
                for box in i.boxes:
                    # box.xyxy returns the box coordinates in [x1, y1, x2, y2] format
                    x1, y1, x2, y2 = box.xyxy[0].tolist() # Convert tensor to list
                    #conf = box.conf[0].item() # Confidence score
                    # confidence based cut off
                    # if confidence below threshold remove
                    
                    width = x2 - x1
                    height = y2 - y1
                    area = width * height

                    if area > max_area:
                        max_area = area
                        big = box
            else:
                return False

        x1, y1, x2, y2 = big.xyxy[0].tolist() # Convert tensor to list

        cls = int(big.cls[0].item()) # Class ID
        width = x2 - x1
        height = y2 - y1

        _, img_width, _ = self.cv_image.shape
        frame_center_x = img_width / 2
        obj_x = width/2
        center_dist = obj_x - frame_center_x

        #class_name = big.class_names.get(cls, f"Unknown_{cls}") # Get class name

        largest_box = {
            'width': width,
            'height': height,
            #'area': area,
            #'name' : class_name,
            'center_dist': center_dist,
            'class': cls
            }
        print(largest_box)
        self.get_logger().info(str(largest_box))
        return largest_box

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
    ml = ML() # Create an instance of the ML node
    try:
        rclpy.spin(ml) # Keep the node alive until it's explicitly shut down
    except KeyboardInterrupt:
        pass # Handle Ctrl+C gracefully
    finally:
        ml.destroy_node() # Clean up resources
        rclpy.shutdown() # Shut down ROS2 client library

if __name__ == '__main__':
    main()
