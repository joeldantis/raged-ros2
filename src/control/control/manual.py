import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import json
import keyboard

class Manual(Node):
    """
    ROS Node that is responsible for taking keypress used for manual control of bot
    """

    def __init__(self):
        # Initialize the node with the name 'manual'
        super().__init__('manual')

        # Variables
        self.prev_state = ''

        # Publishers
        self.mode = self.create_publisher(Int32, 'mode', 10)
        self.action = self.create_publisher(String, 'action', 10)

        # Subscribers

        # Timers
        # Set up a timer to publish at approximately 30 Hz
        self.mode_timer = self.create_timer(1.0 / 30.0, self.mode_timer_callback)
        self.action_timer = self.create_timer(1.0 / 30.0, self.action_timer_callback)

    def mode_timer_callback(self):
        msg = Int32()
        
        if keyboard.is_pressed('m'):
            msg.data = 0
            self.mode.publish(msg)
        
        elif keyboard.is_pressed('n'):
            msg.data = 1
            self.mode.publish(msg)
    
    def action_timer_callback(self):
        msg = String()

        if keyboard.is_pressed('w'):
            if self.prev_state == 'w':
                return
            msg.data = 'w'
            self.action.publish(msg)
            self.prev_state = 'w'

        elif keyboard.is_pressed('a'):
            if self.prev_state == 'a':
                return
            msg.data = 'a'
            self.action.publish(msg)
            self.prev_state = 'a'

        elif keyboard.is_pressed('s'):
            if self.prev_state == 's':
                return
            msg.data = 's'
            self.action.publish(msg)
            self.prev_state = 's'

        elif keyboard.is_pressed('d'):
            if self.prev_state == 'd':
                return
            msg.data = 'd'
            self.action.publish(msg)
            self.prev_state = 'd'

        elif keyboard.is_pressed('1'):
            if self.prev_state == '1':
                return
            msg.data = '1'
            self.action.publish(msg)
            self.prev_state = '1'

        elif keyboard.is_pressed('2'):
            if self.prev_state == '2':
                return
            msg.data = '2'
            self.action.publish(msg)
            self.prev_state = '2'
        
        elif keyboard.is_pressed('3'):
            if self.prev_state == '3':
                return
            msg.data = '3'
            self.action.publish(msg)
            self.prev_state = '3'
        
        elif keyboard.is_pressed('4'):
            if self.prev_state == '4':
                return
            msg.data = '4'
            self.action.publish(msg)
            self.prev_state = '4'
        
        elif keyboard.is_pressed('l'):
            if self.prev_state == 'l':
                return
            msg.data = 'l'
            self.action.publish(msg)
            self.prev_state = 'l'
        
        elif keyboard.is_pressed('k'):
            if self.prev_state == 'k':
                return
            msg.data = 'k'
            self.action.publish(msg)
            self.prev_state = 'k'
        
        elif keyboard.is_pressed('z'):
            if self.prev_state == 'z':
                return
            msg.data = 'z'
            self.action.publish(msg)
            self.prev_state = 'z'
        
        elif keyboard.is_pressed('x'):
            if self.prev_state == 'x':
                return
            msg.data = 'x'
            self.action.publish(msg)
            self.prev_state = 'x'
        
        elif keyboard.is_pressed('c'):
            if self.prev_state == 'c':
                return
            msg.data = 'c'
            self.action.publish(msg)
            self.prev_state = 'c'

        elif keyboard.is_pressed('v'):
            if self.prev_state == 'v':
                return
            msg.data = 'v'
            self.action.publish(msg)
            self.prev_state = 'v'
        
        else:

            msg.data = 'b'
            self.action.publish(msg)
            self.prev_state = 'b'
