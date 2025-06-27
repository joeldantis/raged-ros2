import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
from cv_bridge import CvBridge
import json
import cv2
from ultralytics import YOLO

# Load YOLO model
model = YOLO("/mnt/Storage/Hackathons/IIITM/raged-ros2/src/camera/camera/yolov8n.pt")

class ML(Node):
    def __init__(self):
        super().__init__('ml')
        self.detect = 1
        self.mode = 1
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(Image, 'image', self.image_callback, 10)
        self.detect_sub = self.create_subscription(Int32, 'detect', self.detect_callback, 10)
        self.mode_sub = self.create_subscription(Int32, 'mode', self.mode_callback, 10)

        self.pos_pub = self.create_publisher(String, 'position', 10)
        self.class_pub = self.create_publisher(String, 'class_info', 10)

    def image_callback(self, msg):
        position = String()
        class_info = String()

        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        if not self.detect:
            return

        self.results = []

        # Run YOLO and filter to class 39 (bottle)
        try:
            predictions = model.predict(source=self.cv_image, show=False, stream=True)

            for r in predictions:
                r.boxes = r.boxes[r.boxes.cls == 39]
                if r.boxes:
                    self.results.append(r)
        except Exception as e:
            self.get_logger().error(f"Error during prediction: {e}")
            return

        if not self.results:
            return

        info = self.get_largest()
        if info:
            self.detect = 0

            position.data = json.dumps({
                'obj_width': info['obj_width'],
                'obj_center': info['obj_center'],
                'frame_width': info['frame_width'],
                'frame%': info['frame%'],
                'bin_section': info['bin_section'],
                'name': info['name']
            })

            class_info.data = info['name']

            if self.mode:
                self.pos_pub.publish(position)
            self.class_pub.publish(class_info)

        cv2.imshow("Camera Feed", self.cv_image)
        cv2.waitKey(1)

    def detect_callback(self, msg):
        self.detect = msg.data

    def mode_callback(self, msg):
        self.mode = msg.data

    def get_largest(self):
        largest_box = None
        max_area = 0
        big = None

        for r in self.results:
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                width = x2 - x1
                height = y2 - y1
                area = width * height
                if area > max_area:
                    max_area = area
                    big = box

        if big is None:
            return False

        x1, y1, x2, y2 = big.xyxy[0].tolist()
        cls = int(big.cls[0].item())
        obj_width = x2 - x1
        obj_height = y2 - y1
        obj_area = obj_width * obj_height
        obj_centre = (x2+x1)/2

        frame_height, frame_width, _ = self.cv_image.shape
        frame_area = frame_height * frame_width
        frame_percent = (obj_area / frame_area) * 100

        class_name = model.names[cls]
        bin_section = 1  # hardcoded as per your logic

        info = {
            'obj_width': obj_width,
            'obj_center':obj_centre,
            'frame_width': frame_width,
            'frame%': frame_percent,
            'name': class_name,
            'class': cls,
            'bin_section': bin_section
        }

        self.get_logger().info(str(info))
        return info

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    ml = ML()
    try:
        rclpy.spin(ml)
    except KeyboardInterrupt:
        pass
    finally:
        ml.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
