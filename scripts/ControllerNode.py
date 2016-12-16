#!/usr/bin/env python

""" 
ROS node: Microcontroller
This node takes controller data either from GUI or regulator
and communicates with Arduino type microcontroller.
"""

#Must import rospy and msgs
import rospy
from sensor_msgs.msg import Joy
from tic_tac_drone.msg import CtrlValue


class ControllerNode():
    # Callback for msgs
    def joystick_callback(self,data):
        if (data.buttons[1]):
            self.mode = not self.mode

        if (data.buttons[0]):
            self.kill = 0
        elif (data.buttons[7] and data.buttons[8] and data.axes[2] == -1):
            self.kill = 1

        modifier = self.sensitivity if self.mode else 1

        rospy.loginfo ("Mode = {0}".format(self.mode))
        self.control.pitch = (data.axes[1] * modifier + 1) * 0.5
        self.control.roll = (-1 * data.axes[0] * modifier + 1) * 0.5
        self.control.throttle = (data.axes[2] + 1) * 0.5 * modifier * self.kill
        self.control.yaw = 0.5 + data.buttons[3]*self.yaw_rate*(-1) + data.buttons[4]*self.yaw_rate

            
    # Must have __init__(self) function for a class
    def __init__(self):
        # Create a publisher for commands
        pub = rospy.Publisher('manual_output',CtrlValue, queue_size=1)

        # Local helper variables
        self.mode = 0 # Modifier for controls sensitivity 0 -> 100%, 1 -> definied by self.sensitivity
        self.sensitivity = rospy.get_param('~sensitivity', 0.4)
        self.yaw_rate = rospy.get_param('~yaw_rate', 0.2)
        self.kill = 1 # Flag for a panic button

        # Set the message to publish as command.
        self.control = CtrlValue()
        
        # Initialize message variables.
        self.control.pitch = 0
        self.control.roll = 0
        self.control.yaw = 0
        self.control.throttle = 0
        
        # Create a subscriber for color msg
        rospy.Subscriber("joystick_input", Joy, self.joystick_callback)
        
        # Main while loop.
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            # Publish our command.
            pub.publish(self.control)
            rate.sleep()

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('Controller')
    
    # Go to class functions that do all the heavy lifting.
    # Do error checking.
    try:
        co = ControllerNode()
    except rospy.ROSInterruptException:
        pass
