#!/usr/bin/env python

__author__ = 'Branko'

import rospy
from tic_tac_drone.msg import CustomPose
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from math import *
import time

class Messages:
    """ Treba napisati docstring """
    def __init__(self):
        # Create publishers
        self.pub = rospy.Publisher("signal", CustomPose, queue_size=1)
        self.pub1 = rospy.Publisher("current", CustomPose, queue_size=1)

        # Create variables
        self.signal = CustomPose()
        self.feedback = CustomPose()
        self.reference_pod = CustomPose()
        self.running = 0

        # Create subscribers
        rospy.Subscriber("MyUAV/cpose", CustomPose, self.feedback_callback, queue_size=1)
        rospy.Subscriber("reference", Point, self.reference_callback, queue_size=1)

        # Initialize reference
        self.reference_pod.z = 0
        self.reference_pod.x = 0
        self.reference_pod.y = 0
        self.reference_pod.yaw = 0


    def reference_callback(self, data):
        """ Treba napisati docstring """
        self.running = 1
        self.reference_pod.x = data.x
        self.reference_pod.y = data.y
        self.reference_pod.z = data.z


    def feedback_callback(self, data):
        self.feedback.x = data.x
        self.feedback.y = data.y
        self.feedback.z = data.z
        self.feedback.yaw = data.yaw

        if (not self.running):
            self.reference_pod.x = self.feedback.x
            self.reference_pod.y = self.feedback.y
            self.reference_pod.z = self.feedback.z

        # IZBACITI NAKON SPREMANJA ODZIVA
        self.pub1.publish(self.feedback)


    def send_signals(self, signal_x=0.5, signal_y=0.5, signal_z=0, signal_yaw=0.5):
        """ Treba napisati docstring """
        self.signal.x = signal_x
        self.signal.y = signal_y
        self.signal.z = signal_z
        self.signal.yaw = signal_yaw

        self.pub.publish(self.signal)


    def getX_reference(self):
        return self.reference_pod.x 

    def getY_reference(self):
        return self.reference_pod.y

    def getZ_reference(self):
        return self.reference_pod.z

    def getYAW_reference(self):
        return self.reference_pod.yaw
    
    def getX_feedback(self):
        return self.feedback.x

    def getY_feedback(self):
        return self.feedback.y

    def getZ_feedback(self):
        return self.feedback.z

    def getYAW_feedback(self):
        return self.feedback.yaw


class PID:
    """ Treba napisati docstring """
    def __init__(self, Kr=1.0, Integrator_max=100.0, Integrator_min=0.0, Kr_i = 1.0, Kr_d= 1.0):
        # Set variables
        self.Kr = Kr
        self.Integrator_max = Integrator_max
        self.Integrator_min = Integrator_min
        self.Kr_i = Kr_i
        self.Kr_d = Kr_d

        # Initialize variables
        self.error_prefilter = [0.0, 0.0]
        self.error = [0.0, 0.0, 0.0]
        self.u_p = 0.0
        self.u_i = [0.0, 0.0, 0.0]
        self.u_d = [0.0, 0.0]
        self.error_d = [0.0, 0.0]
        self.set_point = 0.0
        self.u_sum = [0.0, 0.0] 

    def update(self, current_value):
        """ Napisati docstring """

        # Calculate regulation error
        self.error[0] = self.set_point - current_value

        # Calculate P, I and D components
        self.u_p = self.Kr * self.error[0]
        self.u_i[0] = self.Kr_i * self.error[0] + self.u_i[1]
        self.u_d[0] = self.Kr_d * (self.error[0] - self.error[1])

        # Integral part saturation
        if self.u_i[0] > self.Integrator_max :
            self.u_i[0] = self.Integrator_max
        elif self.u_i[0] < self.Integrator_min:
            self.u_i[0] = self.Integrator_min

        # Store current values for future uses as history
        self.u_i[2] =  self.u_i[1]
        self.u_i[1] =  self.u_i[0]
        self.u_d[1] =  self.u_d[0]
        self.error[2] = self.error[1]
        self.error[1] = self.error[0]
        self.error_d[1] = self.error_d[0]

        # Sum P, I and D parts to make a complete signal
        u_sum = self.u_p + self.u_d[0] + self.u_i[0]
        return u_sum

    def setPoint(self, set_point):
            self.set_point = set_point


if __name__ == '__main__':

    rospy.init_node('PID')
    try:
        Mes = Messages()
    except rospy.ROSInterruptException:
        pass

    radius = 0.1
    pub_p = rospy.Publisher("setpoint", Float32, queue_size=1)

    signal_x_main = 0.0
    signal_y_main = 0.0
    signal_z_main = 0.0
    signal_yaw_main = 0.0

    time.sleep(3)

    PID_x = PID(Kr=0.7, Integrator_max=100.0, Integrator_min=-100.0, Kr_i=0, Kr_d=10)

    PID_y = PID(Kr=0.7, Integrator_max=100.0, Integrator_min=-100.0, Kr_i = 0, Kr_d= 10)

    PID_z = PID(Kr=40.0, Integrator_max=50.0, Integrator_min=-50.0, Kr_i = 0.8, Kr_d= 1000.0)

    PID_yaw = PID(Kr= -1.0/500, Integrator_max=50.0, Integrator_min=-50.0, Kr_i = 0, Kr_d= 0)


    rate = rospy.Rate (20)
    while not rospy.is_shutdown():
        # Set reference
        PID_x.setPoint(Mes.getX_reference())
        PID_y.setPoint(Mes.getY_reference())
        PID_z.setPoint(Mes.getZ_reference())
        pub_p.publish(PID_z.set_point)
        PID_yaw.setPoint(Mes.getYAW_reference())

        # Set current values
        current_x = Mes.getX_feedback()
        current_y = Mes.getY_feedback()
        current_z = Mes.getZ_feedback()
        current_yaw = Mes.getYAW_feedback()

        # Calculate regulator output
        signal_x_main = PID_x.update(current_x)
        signal_y_main = PID_y.update(current_y)
        signal_z_main = PID_z.update(current_z)
        signal_yaw_main = PID_yaw.update(current_yaw)

        # Scale regulator output to use as control value for UAV
        if signal_z_main > 50.0:
            signal_z_main = 50.0
        elif signal_z_main < -50.0:
            signal_z_main = -50.0        
        signal_z_main = (0.25/50.0) * signal_z_main + 0.7 * Mes.running # ????? .25 i .7

        if signal_x_main > 0.2:
            signal_x_main = 0.2
        elif signal_x_main < -0.2:
            signal_x_main = -0.2
        signal_x_main = signal_x_main + 0.5

        if signal_y_main > 0.2:
            signal_y_main = 0.2
        elif signal_y_main < -0.2:
            signal_y_main = -0.2
        signal_y_main = -(signal_y_main + 0.5)

        signal_yaw_main = signal_yaw_main + 0.5


        # Publish control values for UAV
        Mes.send_signals(signal_x=signal_x_main,signal_y=signal_y_main,signal_z=signal_z_main,signal_yaw=signal_yaw_main)

        condition = ((abs(Mes.getX_reference() - current_x) < radius) and 
                    (abs(Mes.getY_reference() - current_y) < radius) and 
                    (abs(Mes.getZ_reference() - current_z) < radius))

        if(Mes.getZ_reference() == 0 and condition):
            Mes.running = 0
            PID_z.u_i=[0.0, 0.0, 0.0]

        rate.sleep()
