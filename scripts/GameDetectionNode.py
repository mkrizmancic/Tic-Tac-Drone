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

    def in_sphere (self, radius = 0.1):
        """ Check is if drone is within 'radius' distance from target """
        #rospy.loginfo ("s: x=%f,  y=%f,  z=%f", self.course[0].x, self.course[0].y, self.course[0].z)

        if (abs (self.course[self.next_step].x - self.pos.x) <= radius and abs (self.course[self.next_step].y - self.pos.y) <= radius and abs (self.course[self.next_step].z - self.pos.z) <= radius):
            return True
        else:
            return False


    def check_leg (self):
        """ Function that checks if part of the course is done """
        if self.in_sphere(radius=0.1):
            if (self.timer == 0):
                self.start_time = rospy.get_rostime()
                self.timer = 1
            else:
                if (rospy.get_rostime() - self.start_time >= self.duration):
                    if (self.next_step == 3):
                        rospy.loginfo (rospy.get_caller_id() + "Potez odigran")
                        self.pub_game.publish(self.move)
                        self.next_step = 0
                        self.course_set = 0
                    else:
                        self.next_step += 1
                        rospy.loginfo (rospy.get_caller_id() + "Checkpoint")
                    self.publish_once = 1
        else:
            self.timer = 0


    def vector_rotation (self, old, base):
        """ Calculates exact coordinates for a target """
        fi = radians(base.theta)
        self.goal.x = old.x*cos(fi) - old.y*sin(fi) + base.x - base.x*cos(fi) + base.y*sin(fi)
        self.goal.y = old.x*sin(fi) + old.y*cos(fi) + base.y - base.x*sin(fi) - base.y*cos(fi)
        self.goal.z = 0


    def set_origin (self, data):
        """ Saves location of the playing field """
        self.origin.x = data.x
        self.origin.y = data.y
        self.origin.theta = data.theta

       
    def next_move (self, data):
        """ Defines target coordinates for next move """
        relative = Pose2D()
        self.move.row = data.row
        self.move.col = data.col
        self.move.player = data.player
        relative.x = self.origin.x + data.col * self.cell_size + self.cell_size / 2
        relative.y = self.origin.y -(data.row * self.cell_size + self.cell_size / 2)  
        self.vector_rotation(relative, self.origin)
        #rospy.loginfo ("g: x=%f,  y=%f,  z=%f", self.goal.x, self.goal.y, self.goal.z)

        print self.goal
        # self.course[0].x = self.pos.x
        # self.course[0].y = self.pos.y
        # self.course[0].z = 0
        #rospy.loginfo ("1: x=%f,  y=%f,  z=%f", self.pos.x, self.pos.y, self.max_height)
        
        self.course[0].x = self.pos.x
        self.course[0].y = self.pos.y
        self.course[0].z = self.max_height

        self.course[1].x = self.goal.x
        self.course[1].y = self.pos.y
        self.course[1].z = self.max_height

        self.course[2].x = self.goal.x
        self.course[2].y = self.goal.y   
        self.course[2].z = self.max_height
        #rospy.loginfo ("2: x=%f,  y=%f,  z=%f", self.goal.x, self.goal.y, self.max_height)

        self.course[3].x = self.goal.x
        self.course[3].y = self.goal.y
        self.course[3].z = 0
        #rospy.loginfo ("3: x=%f,  y=%f,  z=%f", self.goal.x, self.goal.y, 0)

        # rospy.loginfo ("*g: x=%f,  y=%f,  z=%f", self.goal.x, self.goal.y, self.goal.z)
        # rospy.loginfo ("*1: x=%f,  y=%f,  z=%f", self.pos.x, self.pos.y, self.max_height)
        # rospy.loginfo ("*2: x=%f,  y=%f,  z=%f", self.goal.x, self.goal.y, self.max_height)
        # rospy.loginfo ("*3: x=%f,  y=%f,  z=%f", self.goal.x, self.goal.y, 0)


        # rospy.loginfo ("-0: x=%f,  y=%f,  z=%f", self.course[0].x, self.course[0].y, self.course[0].z)
        # rospy.loginfo ("-1: x=%f,  y=%f,  z=%f", self.course[1].x, self.course[1].y, self.course[1].z)
        # rospy.loginfo ("-2: x=%f,  y=%f,  z=%f", self.course[2].x, self.course[2].y, self.course[2].z)

        self.course_set = 1
     
    def uav_tracking (self, data):
        self.pos.x = data.x
        self.pos.y = data.y
        self.pos.z = data.z

        if self.course_set:
            if self.publish_once:
                #rospy.loginfo ("reg_0: x=%f,  y=%f,  z=%f", self.course[0].x, self.course[0].y, self.course[0].z)

                #rospy.loginfo ("reg: x=%f,  y=%f,  z=%f", self.course[self.next_step].x, self.course[self.next_step].y, self.course[self.next_step].z)

                self.pub_reg.publish (self.course[self.next_step]) # Publishes coordinates for next step to the regulator
                self.publish_once = 0
            self.check_leg()

    # Must have __init__(self) function for a class
    def __init__(self):
        # Create a publisher
        self.pub_game = rospy.Publisher('Get_move', MakeMove, queue_size=1)
        self.pub_reg = rospy.Publisher('reference', Point, queue_size=1)

        # Define flags
        self.timer = 0          # Starting and stopping the timer
        self.next_step = 0      # Which waypoint is next
        self.course_set = 0     # Course is set and can be used
        self.publish_once = 1   # Make sure we publish only once

        # Create variables
        self.origin = Pose2D()  # Upper left corner of the playing field
        self.goal = Point()     # Next move cell location
        self.pos = Point()      # Drone position
        self.course = [ Point() for i in range(5)]   # Set of waypoints for drone navigation
        self.move = MakeMove()
        
        self.start_time = 0

        #rospy.loginfo ("*!: x=%f,  y=%f,  z=%f", self.pos.x, self.pos.y, self.pos.z)

        # Define parameters
        self.cell_size = rospy.get_param("~cell_size", 0.3)
        self.max_height = rospy.get_param("~max_height", 1)
        self.duration = rospy.Duration.from_sec(3)

        # Create subscribers
        rospy.Subscriber("/field_pos", Pose2D, self.set_origin) # NEPOTREBNO SE ZOVE PUNO PUTA
        rospy.Subscriber("/Make_move", MakeMove, self.next_move)
        rospy.Subscriber("/MyUAV/cpose",CustomPose, self.uav_tracking)
 
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
