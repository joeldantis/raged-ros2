import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import json
import serial

global ser

port = '/dev/ttyACM0'
baud_rate = 9600
ser = serial.Serial(port, baud_rate, timeout=1)

class Communication(Node):
    """
    ROS Node that is responsible for taking values form various nodes and 
    communicating with the microcontroller
    """

    def __init__(self):
        # Initialize the node with the name 'manual'
        super().__init__('communication')

        # Variables
        self.mode = 1

        # Publishers
        self.detect_pub = self.create_publisher(Int32, 'detect', 10)
        self.serial = self.create_publisher(String, 'serial', 10)

        #Subcribers
        self.mode_sub = self.create_subscription(Int32, 'mode', self.mode_callback, 10)
        self.control_sub = self.create_subscription(String, 'control', self.control_callback, 10)
        self.action_sub = self.create_subscription(String, 'action', self.action_callback, 10)

        # Timers
        self.timer = self.create_timer(0.1, self.timer_callback)


    def mode_callback(self, msg):
        self.mode = msg.data
    
    def control_callback(self, msg):
        dat = json.loads(msg.data)
        command = f'{self.mode}, {dat[0]}, {dat[1]}\n'
        ser.write(command.encode('utf-8'))
        print(command)

    def action_callback(self,msg):
        global ser
        dat = json.loads(msg.data)
        command = f'{self.mode},{dat}\n'
        ser.write(command.encode('utf-8'))
        print(command)

    def timer_callback(self):
        global ser
        response = ser.readline().decode('utf-8', errors='ignore').strip()
        if response == "r":
            det = Int32()
            det.data = 1
            self.detect_pub.publish(det)
        pass

def main(args=None):
    """
    Main function for the position node.
    """
    rclpy.init(args=args) # Initialize ROS2 client library
    communication = Communication() # Create an instance of the position node
    try:
        rclpy.spin(communication) # Keep the node alive until it's explicitly shut down
    except KeyboardInterrupt:   
        pass # Handle Ctrl+C gracefully
    finally:
        communication.destroy_node() # Clean up resources
        rclpy.shutdown() # Shut down ROS2 client library

if __name__ == '__main__':
    main()
