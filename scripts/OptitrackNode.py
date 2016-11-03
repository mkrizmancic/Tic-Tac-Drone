#!/usr/bin/env python 

""" 
ROS node: OptiTrack
This node takes position data from OptiTrack system, 
performs necessary axis adjustments 
and distributes that information to other nodes.
"""

#Import necessary dependencies
import rospy
from geometry_msgs.msg import PoseStamped
from tic_tac_drone.msg import CustomPose
from tf.transformations import euler_from_quaternion
from math import degrees

class OptitrackNode():

    def quat_to_eul (self, data):
        # Funcion that transforms quaternions to euler
        quaternion = (
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w)

        return euler_from_quaternion(quaternion)

    def adjust_axis_1 (self,data):
        # Function that adjusts position info for our UAV
        self.uav1_pos.x = data.pose.position.x
        self.uav1_pos.y = data.pose.position.z
        self.uav1_pos.z = data.pose.position.y

        euler = self.quat_to_eul(data)
        self.uav1_rot.roll = degrees(euler)[0]
        self.uav1_rot.pitch = degrees(euler)[2]
        self.uav1_rot.yaw = degrees(euler)[1]

    def adjust_axis_2 (self,data):
        # Function that adjusts position info for opponent UAV
        self.uav2_pos.x = data.pose.position.x
        self.uav2_pos.y = data.pose.position.z
        self.uav2_pos.z = data.pose.position.y

        euler = self.quat_to_eul(data)
        self.uav2_rot.roll = degrees(euler[0])
        self.uav2_rot.pitch = degrees(euler[2])
        self.uav2_rot.yaw = degrees(euler[1])

    # Must have __init__(self) function for a class
    def __init__(self):
        # Create a publisher
        pub_p1 = rospy.Publisher('MyUAV/pose', CustomPose, queue_size=1)
        pub_p2 = rospy.Publisher('OpUAV/pose', CustomPose, queue_size=1)
 
        # Set the message to publish as command.
        self.uav1_pos = CustomPose()    # Position of our UAV
        self.uav2_pos = CustomPose()    # Position of opponent UAV
        
        # Create subscribers
        rospy.Subscriber("/vrpn_client_node/kruna/pose", PoseStamped, self.adjust_axis_1)
        rospy.Subscriber("/vrpn_client_node/UAV2/pose", PoseStamped, self.adjust_axis_2)
        
        # Main while loop.
        while not rospy.is_shutdown():
            # Publish our command.
            pub_p1.publish(self.uav1_pos)
            pub_p2.publish(self.uav2_pos)
            rospy.sleep(1.0)

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('Optitrack')
    
    # Go to class functions that do all the heavy lifting.
    # Do error checking.
    try:
        on = OptitrackNode()
    except rospy.ROSInterruptException:
        pass
