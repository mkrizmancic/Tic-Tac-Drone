#!/usr/bin/env python 

""" 
ROS node: OptiTrack
This node takes position data from OptiTrack system, 
performs necessary axis adjustments 
and distributes that information to other nodes.
"""

#Import necessary dependencies
import rospy
from geometry_msgs.msg import PoseStamped, Pose2D
from tic_tac_drone.msg import CustomPose
from tf.transformations import euler_from_quaternion
from math import degrees

class OptitrackNode():

    def quat_to_eul (self, data):
        """ 
        This function takes data in quaternion form, exchanges y and z axis and 
        transforms the data to euler form
        """
        quaternion = (
            data.pose.orientation.x,
            data.pose.orientation.z,
            data.pose.orientation.y,
            data.pose.orientation.w)

        return euler_from_quaternion(quaternion)

    def adjust_axis_1 (self,data):
        """ 
        Callback function that adjust axis data for our UAV.
        Data is stored in class variables.
        """
        self.uav1.x = data.pose.position.x
        self.uav1.y = -data.pose.position.z
        self.uav1.z = data.pose.position.y

        euler = self.quat_to_eul(data)
        self.uav1.yaw = degrees(euler[2])

    def adjust_axis_2 (self,data):
        """ 
        Callback function that adjust axis data for opponent UAV.
        Data is stored in class variables.
        """
        self.uav2.x = data.pose.position.x
        self.uav2.y = -data.pose.position.z
        self.uav2.z = data.pose.position.y

    def adjust_field (self, data):
        """
        Callback function that adjusts axis data for playing field and publishes its position.
        """
        self.field.x = data.pose.position.x
        self.field.y = data.pose.position.z
        euler = self.quat_to_eul(data)
        self.field.theta = degrees(euler[2])

        pub_f.publish(self.field)

    # Must have __init__(self) function for a class
    def __init__(self):
        # Create a publisher
        pub_p1 = rospy.Publisher('MyUAV/cpose', CustomPose, queue_size=1)
        pub_p2 = rospy.Publisher('OpUAV/cpose', CustomPose, queue_size=1)
        pub_f = rospy.Publisher('field_pos', Pose2D, queue_size=1)
 
        # Set the message to publish as command.
        self.uav1 = CustomPose()    # Position of our UAV
        self.uav2 = CustomPose()    # Position of opponent UAV
        self.field = Pose2D()       # Position of playing field
        
        # Create subscribers
        rospy.Subscriber("MyUAV/pose", PoseStamped, self.adjust_axis_1)
        rospy.Subscriber("OpUAV/pose", PoseStamped, self.adjust_axis_2)
        rospy.Subscriber("Field/pose", PoseStamped, self.adjust_field)
        
        # Main while loop.
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            # Publish our command.
            pub_p1.publish(self.uav1)
            pub_p2.publish(self.uav2)
            pub_f.publish(self.field)
            rate.sleep()

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('Optitrack')
    
    # Go to class functions that do all the heavy lifting.
    # Do error checking.
    try:
        on = OptitrackNode()
    except rospy.ROSInterruptException:
        pass
