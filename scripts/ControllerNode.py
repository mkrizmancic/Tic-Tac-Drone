#!/usr/bin/env python

""" 
ROS node: Microcontroller
This node takes controller data either from GUI or regulator
and communicates with Arduino type microcontroller.
"""

#Must import rospy and msgs
import rospy
from sensor_msgs.msg import Joy


class ControllerNode():
    # Callback for msgs
    def joystick_callback(self,data):
        if (data.GUMB_ZA_PROMJENU_MODA):
            self.mode = not self.mode

        self.IME_PORUKE.pitch = data.axes[] / self.max_axis_value * (1-0.6*self.mode)
        self.IME_PORUKE.roll = data.axes[] / self.max_axis_value * (1-0.6*self.mode)
        self.IME_PORUKE.throttle = data.axes[] / self.max_axis_value * (1-0.6*self.mode)
        self.IME_PORUKE.yaw = data.axes[] / self.max_axis_value * (1-0.6*self.mode)
        #ili
        self.IME_PORUKE.yaw = data.buttons[]*self.yaw_rate*(-1) + data.buttons[]*self.yaw_rate

            
    # Must have __init__(self) function for a class
    def __init__(self):
        # Create a publisher for commands
        pub = rospy.Publisher('manual_output',TIP_PORUKE, queue_size=1)

        # Local helper variables
        self.mode = 0 # Modifier for controls sensitivity 0 -> 100%, 1 -> 40%
        self.max_axis_value =
        self.yaw_rate =

        # Set the message to publish as command.
        self.IME_PORUKE = TIP_PORUKE()
        
        # Initialize message variables.
        self.IME_PORUKE.pitch = 0
        self.IME_PORUKE.roll = 0
        self.IME_PORUKE.yaw = 0
        self.IME_PORUKE.throttle = 0
        
        # Create a subscriber for color msg
        rospy.Subscriber("joystick_input", Joy, self.joystick_callback)
        
        # Main while loop.
        while not rospy.is_shutdown():
            # Publish our command.
            pub.publish(self.IME_PORUKE)
            rospy.sleep(1.0)

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('Controller')
    
    # Go to class functions that do all the heavy lifting.
    # Do error checking.
    try:
        co = ControllerNode()
    except rospy.ROSInterruptException:
        pass