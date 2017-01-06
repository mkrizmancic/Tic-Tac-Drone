#!/usr/bin/env python

"""
ROS node: Game Detection
This node translates relative information about next move into absolute coordinates,
monitors drone's position to determine if a move has been played and
plans a course from starting point to the goal.
"""

#Import necessary dependencies
import rospy
from geometry_msgs.msg import Pose2D, Point
from tic_tac_drone.msg import MakeMove, CustomPose
from math import *

class GameDetectionNode():

    def in_sphere (current, target, radius = 0.1):
        """ Check is if drone is within 'radius' distance from target """
        if (abs (target.x - current.x) <= radius and abs (target.y - current.y) <= radius and abs (target.z - current.z) <= radius):
            return True
        else:
            return False


    def check_leg (position):
        """ Function that checks if part of the course is done """
        if self.in_sphere(position, self.course[self.next_step]):
            if (self.timer == 0):
                self.start_time = rospy.get_rostime()
                self.timer = 1
            else:
                if (rospy.get_rostime() - start_time >= self.duration):
                    if (self.next_step == 2):
                        rospy.loginfo (rospy.get_caller_id() + "Potez odigran")
                        #return potez odigran
                        self.next_step = 0
                        self.course_set = 0
                    else:
                        self.next_step += 1
                        rospy.loginfo (rospy.get_caller_id() + "Checkpoint")
                    self.publish_once = 1
        else:
            self.timer = 0


    def vector_rotation (old, base):
        """ Calculates exact coordinates for a target """
        new = Point()
        fi = radians(base.theta if (base.theta >= 0) else base.theta + 360)
        new.x = old.x*cos(fi) - old.y*sin(fi) + base.x - base.x*cos(fi) + base.y*sin(fi)
        new.y = old.x*sin(fi) + old.y*cos(fi) + base.y - base.x*sin(fi) - base.y*cos(fi)
        new.z = 0
        return new


    def set_origin (self, data):
        """ Saves location of the playing field """
        self.origin.x = data.x
        self.origin.y = data.y
        self.origin.theta = data.theta

       
    def next_move (self, data):
        """ Defines target coordinates for next move """
        relative = Pose2D()
        relative.x = data.col * self.cell_size + self.cell_size / 2
        relative.y = -(data.row * self.cell_size + self.cell_size / 2)  
        self.goal = self.vector_rotation(relative, self.origin)


        self.course[0] = self.pos
        self.course[0].z = self.max_height
        self.course[1] = self.goal
        self.course[1].z = self.max_height
        self.course[2] = self.goal
        self.course_set = 1
     
    def uav_tracking (self, data):
        self.pos.x = data.x
        self.pos.y = data.y
        self.pos.z = data.z

        if self.course_set:
            if self.publish_once:
                pub_reg.publish (self.course[self.next_step]) # Publishes coordinates for next step to the regulator
                self.publish_once = 0
            self.check_leg(self.pos)

    # Must have __init__(self) function for a class
    def __init__(self):
        # Create a publisher
        # pub_game = rospy.Publisher('', tic_tac_toe.msg, queue_size=1)
        pub_reg = rospy.Publisher('reference', Point, queue_size=1)

        # Define flags
        self.timer = 0          # Starting and stopping the timer
        self.next_step = 0      # Which waypoint is next
        self.course_set = 0     # Course is set and can be used
        self.publish_once = 1   # Make sure we publish only once

        # Create variables
        self.origin = Pose2D()  # Upper left corner of the playing field
        self.goal = Point()     # Next move cell location
        self.pos = Point()      # Drone position
        self.course = []        # Set of waypoints for drone navigation
        self.start_time = 0

        # Define parameters
        self.cell_size = rospy.get_param("~cell_size", 0.5)
        self.max_height = rospy.get_param("~max_height", 1)
        self.duration = rospy.Duration.from_sec(3)

        # Create subscribers
        rospy.Subscriber("/Optitrack/field_pos", Pose2D, self.set_origin) # NEPOTREBNO SE ZOVE PUNO PUTA
        rospy.Subscriber("/TicTacToe/Make_move", MakeMove, self.next_move)
        rospy.Subscriber("/Optitrack/MyUAV/cpose",CustomPose, self.uav_tracking)
 
        # Main while loop
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('GameDetection')
    
 
    # Go to class functions that do all the heavy lifting.
    # Do error checking.
    try:
        gdn = GameDetectionNode()
    except rospy.ROSInterruptException:
        pass
