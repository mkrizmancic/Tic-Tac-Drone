#!/usr/bin/env python

""" 
ROS node: Microcontroller
This node takes controller data either from GUI or regulator
and communicates with Arduino type microcontroller.
"""

#Must import rospy and msgs
import rospy
from sensor_msgs.msg import Joy


class NodeExample():
    # Callback for msgs
    def callback(self,data):
        
            
    # Must have __init__(self) function for a class
    def __init__(self):
        # Create a publisher for commands
        pub = rospy.Publisher('',,queue_size=1)
        
        # Set the message to publish as command.
        # self.variable means you can access it from class fnc
        self. = 
        
        # Initialize message variables.

        
        # Create a subscriber for color msg
        rospy.Subscriber("", Joy, self.callback)
        
        # Main while loop.
        while not rospy.is_shutdown():
            # Publish our command.
            pub.publish(self.)
            rospy.sleep(1.0)

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('pyclass')
    
    # Go to class functions that do all the heavy lifting.
    # Do error checking.
    try:
        ne = NodeExample()
    except rospy.ROSInterruptException:
        pass