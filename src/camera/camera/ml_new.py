import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Int32MultiArray, String
from cv_bridge import CvBridge
import json
import cv2
from ultralytics import RTDETR, YOLO

# Load the YOLO instance segmentation model
global model
model = RTDETR("/mnt/Storage/Hackathons/IIITM/raged-ros2/src/camera/camera/rtdetr.pt")  # Replace with your model path if custom

category_mapping = {
    'Cigarette': 4,
    'Plastic bottle': 1,
    'Plastic wrapper': 1,
    'Plastic container': 1,
    'Plastic cup': 1,
    'Plastic lid/cap': 1,
    'Plastic utensil': 1,
    'Plastic straw': 1,
    'Plastic bag': 1,
    'Plastic gloves': 1,
    'Foam container': 1,
    'Paper straw': 2,
    'Paper': 2,
    'Paper container': 2,
    'Carton': 2,
    'Metal can/lid': 3,
    'Scrap metal': 3,
    'Glass': 3,
    'Aluminium/foil': 3,
    'Rope/string': 4,
    'Aerosol': 3,
    'Tissues': 4,
    'Unlabeled litter': 4,
    'Food waste': 4,
    'Other': 4
}

class ML(Node):
    """
    A ROS2 node that subscribes to sensor_msgs/msg/Image messages and displays them.
    """
    def __init__(self):

        # variables
        self.detect = 1
        self.mode = 1

        # Initialize the node with the name 'ml'
        super().__init__('ml')

        # Subscribers
        self.image_sub = self.create_subscription(Image,'image',self.image_callback,10)
        self.detect_sub = self.create_subscription(Int32, 'detect', self.detect_callback, 10)
        self.mode_sub = self.create_subscription(Int32, 'mode', self.mode_callback,10)

        # Publishers
        self.pos_pub = self.create_publisher(String, 'position', 10)
        self.class_pub = self.create_publisher(String, 'class_info', 10)


        # Initialize CvBridge for converting between ROS and OpenCV images
        self.bridge = CvBridge()
#        self.get_logger().info('Camera subscriber node started.')
    
    def image_callback(self, msg):
        global model

        position = String()
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
                position.data = json.dumps({
                    'obj_width': info['obj_width'],
                    'obj_center': info['obj_center'],
                    'frame_width': info['frame_width'],
                    'frame%': info['frame%'],
                    'bin_section': info['bin_section'],
                    'name': info['name']
                })
                class_info.data = info['name']

                # publishing
                if self.mode:
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

    def mode_callback(self,msg):
        self.mode = msg.data

    def get_largest(self):
        global model

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
        obj_height = y2-y1
        obj_width = x2 - x1
        obj_area = obj_height*obj_width
        obj_centre = (x2+x1)/2
        #height = y2 - y1

        frame_height, frame_width, _ = self.cv_image.shape
        frame_area = frame_height*frame_width

        frame_percent = (obj_area/frame_area)*100

        #frame_center_x = img_width / 2
        #obj_x = width/2
        #center_dist = obj_x - frame_center_x

        class_name = model.names[cls]

        # add if blocks to separate the classes into 4 sections
        #bin_section = category_mapping[f'{class_name}']
        bin_section = 1

        largest_box = {
            'obj_width': obj_width,
            'obj_center':obj_centre,
            'frame_width': frame_width,
            'frame%': frame_percent,
            #'height': height,
            #'area': area,
            'name' : class_name,
            #'center_dist': center_dist,
            'class': cls,
            'bin_section': bin_section
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
    Main function for the ml node.
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
