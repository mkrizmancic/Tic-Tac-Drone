#!/usr/bin/env python 

""" 
ROS node: OptiTrack
This node takes position data from OptiTrack system, 
performs necessary axis adjustments 
and distributes that information to other nodes.
"""

#Import necessary dependencies
import rospy
from geometry_msgs.msg import Accel, Pose, Twist, PoseStamped
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

        euler = euler_from_quaternion(quaternion)
        # self.uav1_pos.roll = euler[0]
        # self.uav1_pos.pitch = euler[1]
        # self.uav1_pos.yaw = euler[2]
        rospy.loginfo("Roll: %f  | Pitch: %f  | Yaw: %f", degrees(euler[0]) , degrees(euler[2]) , degrees(euler[1]) )

    def adjust_axis_pos(self,data):
        # Tu sad treba prebaciti dobivene koordinate u potrebne koordinate
        # self.uav1_pos.x = data.position.x
        # self.uav1_pos.y = data.position.z
        # self.uav1_pos.z = data.position.y
        rospy.loginfo("X: %f  | Y: %f  | Z: %f", data.pose.position.x, data.pose.position.z, data.pose.position.y)
        self.quat_to_eul(data)

    #def adjust_axis_vel (self, data):
            
    # Must have __init__(self) function for a class
    # similar to a C++ class constructor.
    def __init__(self):
        # Create a publisher
        #pub_p1 = rospy.Publisher('MyUAV/pose', MojaPoruka, queue_size=1)
        #pub_v1 = rospy.Publisher('MyUAV/vel', Twist, queue_size=1)
        #pub_p2 = rospy.Publisher('OpUAV/pose', Pose, queue_size=1)

        
        # Set the message to publish as command.
        #self.uav1_pos = MojaPoruka()    # Position of our UAV
        #self.uav1_vel = Twist()         # Velocity of our UAV
        #self.uav2_pos = Pose()          # Position of opponent UAV
        
        # Create subscribers
        rospy.Subscriber("/vrpn_client_node/kruna/pose", PoseStamped, self.adjust_axis_pos)
        #rospy.Subscriber("UAV1/twist", Twist, adjust_axis_vel)
        #rospy.Subscriber("UAV2/pose", Pose, adjust_axis_pos)
        
        # Main while loop.
        while not rospy.is_shutdown():
            # Publish our command.
            #pub_p1.publish(self.uav1_pos)
            #pub_v1.publish(self.uav1_vel)
            #pub_p2.publish(self.uav2_pos)
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
