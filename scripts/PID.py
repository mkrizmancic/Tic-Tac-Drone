#!/usr/bin/env python

__author__ = 'Branko'

import rospy
from tic_tac_drone.msg import CustomPose
from geometry_msgs.msg import Point

class Poruke:

    def __init__(self):

        self.pub = rospy.Publisher("signal", CustomPose, queue_size=1)

        self.signal = CustomPose()
        self.feedback = CustomPose()
        self.reference_pod = CustomPose()
        self.getNextMove_value = False


        rospy.Subscriber("MyUAV/cpose", CustomPose, self.feedback_fun)
        rospy.Subscriber("reference", Point, self.reference_fun)


    def reference_fun(self, data):
        self.reference_pod.x = data.x
        self.reference_pod.y = data.y
        self.reference_pod.z = data.z
        self.reference_pod.yaw = 0.0


        #rospy.loginfo ("++++++++++++++callback: %f", self.reference_pod.z)
        #self.setNextMove(True)
#
#

    def send_signals(self, signal_x=0, signal_y=0, signal_z=0, signal_yaw=0):
        self.signal.x = signal_x
        self.signal.y = signal_y
        self.signal.z = signal_z
        self.signal.yaw = signal_yaw

        self.pub.publish(self.signal)

    def feedback_fun(self, data):

        self.feedback.x = data.x
        self.feedback.y = data.y
        self.feedback.z = data.z
        self.feedback.yaw = data.yaw


    def getX_reference(self):
        return self.reference_pod.x 

    def getY_reference(self):
        return self.reference_pod.y

    def getZ_reference(self):
        return self.reference_pod.z

    def getYAW_reference(self):
        return self.reference_pod.yaw

    def getNextMove(self):
        return self.getNextMove_value

    def setNextMove(self, vrijednost):
        self.getNextMove_value = vrijednost
    
    def getX_feedback(self):
        return self.feedback.x

    def getY_feedback(self):
        return self.feedback.y

    def getZ_feedback(self):
        return self.feedback.z

    def getYaw_feedback(self):
        return self.feedback.yaw


class PID:
    
    def __init__(self,K_pf_num,K_pf_den ,Kr=1.0, Ti=1.0, Td=1.0, v=10.0, Integrator_max=100.0, Integrator_min=0.0, T_dis = 0.01, Kr_i = 1.0, Kr_d= 1.0):

        #print("K_pf_num = %r,K_pf_den = %r ,Kr = %r, Ti = %r, Td = %r, v = %r, Integrator_max = %r, Integrator_min = %r, T_dis = %r"%(K_pf_num,K_pf_den ,Kr, Ti, Td, v, Integrator_max, Integrator_min, T_dis))

        self.K_pf_num = K_pf_num
        self.K_pf_den = K_pf_den
        self.Kr = Kr
        self.Ti = Ti
        self.Td = Td
        self.v = v
        self.Integrator_max = Integrator_max
        self.Integrator_min = Integrator_min
        self.T_dis = T_dis
        self.Kr_i = Kr_i
        self.Kr_d = Kr_d

        
        self.error_prefilter = [0.0 ,0.0]
        self.error = [0.0 ,0.0, 0.0]
        self.u_p = 0.0
        self.u_i = [0.0 ,0.0, 0.0]
        self.u_d = [0.0 ,0.0]
        self.error_d = [0.0 ,0.0]
        self.set_point = 0.0
        self.u_sum = [0.0, 0.0]



    def update(self, current_value):



        self.error[0] = self.set_point - current_value

        #print "errrrrrrrrrrrororororororor" , self.error[0]
        #print "++++++++++++++++++++++++++++++++++++", current_value
        self.u_p = self.Kr * self.error[0]
        #Euler unazadna
        self.u_i[0]= self.Kr_i*self.error[0] + self.u_i[1]
        self.u_d[0]=self.Kr_d*(self.error[0]-self.error[1])

        #Tustin
        #self.u_i[0] = self.u_i[1] + ((self.Kr*self.T_dis)/(self.Ti))*(self.error[0]+self.error[1])/2.0

        if self.u_i[0]> self.Integrator_max :
            self.u_i[0]= self.Integrator_max
        elif self.u_i[0]< self.Integrator_min:
            self.u_i[0]= self.Integrator_min
        #Tustin
        #self.u_d[0] = ((2.0* self.Td - self.v * self.T_dis )/(2.0* self.Td + self.v * self.T_dis ))*self.u_d[1] + ((2.0*self.Kr*self.v*self.Td)/(2.0*self.Td + self.v*self.T_dis))*(self.error[0]-self.error[1])


        #rospy.loginfo("U_P= %f *******U_I= %f*******U_D= %f"%(self.u_p, self.u_i[0],self.u_d[0]) )
        self.u_i[2] =  self.u_i[1] #proba koda-----------------------
        self.u_i[1] =  self.u_i[0]
        self.u_d[1] =  self.u_d[0]
        self.error[2] = self.error[1]#proba koda-----------------------
        self.error[1] = self.error[0]
        self.error_d[1] = self.error_d[0]

        u_sum = self.u_p + self.u_d[0] + self.u_i[0]
        
        return u_sum

    def setPoint(self, set_point):

            self.set_point = set_point
            #self.error = [0.0 ,0.0, 0.0]




    def getSetPoint(self):
        return self.set_point

    def getUi(self):
        return self.u_i[0]
    def setKr(self, Kr):
            self.Kr = Kr

    def setTi(self, Ti):
            self.Ti = Ti

    def setTd(self, Td):
            self.Td = Td


    def setV(self, v):
            self.v = v

    def setT_dis(self, T_dis):
            self.T_dis = T_dis

    def setIntegrator_max(self, Integrator_max):
            self.Integrator_max = Integrator_max

    def setIntegrator_min(self, Integrator_min):
            self.Integrator_min = Integrator_min


# class Rad:
    
#     def run(self):

#         PID_x = PID(K_pf_num=0.7148,K_pf_den=0.2852 ,Kr=7.0, Ti=35.0, Td=0.4, v=10.0, Integrator_max=1.0, Integrator_min=-1.0, T_dis = 0.084, Kr_i = 0.0168, Kr_d= 33.333333)
#         PID_y = PID(K_pf_num=0.7148,K_pf_den=0.2852,Kr=7.0, Ti=35.0, Td=0.4, v=10.0, Integrator_max=1.0, Integrator_min=-1.0, T_dis = 0.084, Kr_i = 0.0168, Kr_d= 33.333333)
#         PID_z = PID(K_pf_num=0.7198,K_pf_den=0.2802,Kr=10.0, Ti=30.0, Td=0.4, v=20.0, Integrator_max=25.0, Integrator_min=-100.0, T_dis = 0.2439, Kr_i = 0.0813, Kr_d= 16.4002)
#         PID_yaw = PID(K_pf_num=0.7123,K_pf_den=0.2877 ,Kr=10.0, Ti=20.0, Td=0.1, v=10.0, Integrator_max=5.0, Integrator_min=0.0, T_dis = 0.084, Kr_i = 0.042, Kr_d= 11.904762)


#         rospy.init_node('PID', anonymous=True)
#         try:
#             Mjerenja = Poruke()
#         except rospy.ROSInterruptException:
#             pass


#         signal_x_main = 0.0
#         signal_y_main =0.0
#         signal_z_main =0.0
#         signal_yaw_main =0.0
#         i = 0
#         kraj = False
#         while True:
            
#             PID_x.setPoint(getX_reference())
#             PID_y.setPoint(getY_reference())
#             PID_z.setPoint(getZ_reference())
#             PID_yaw.setPoint(getYAW_reference())


#             current_x = Mjerenja.getx_feedback()
#             current_y = Mjerenja.gety_feedback()
#             current_z = Mjerenja.getZ_feedback()
#             current_yaw = Mjerenja.getYaw_feedback()


#             signal_x_main = PID_x.update(current_x)
#             signal_y_main = PID_y.update(current_y)
#             signal_z_main = PID_z.update(current_z)
#             signal_yaw_main = PID_yaw.update(current_yaw)

#             if signal_z_main > 25:
#                 signal_z_main = 25
#             elif signal_z_main < -100:
#                 signal_z_main = -100
            
#             signal_z_main = signal_z_main*(1/125.0) +0.8
#             if signal_x_main > 1:
#                 signal_x_main = 1
#             elif signal_x_main < -1:
#                 signal_x_main = -1
#             signal_x_main = signal_x_main*0.5 +0.5
#             if signal_y_main > 1:
#                 signal_y_main = 1
#             elif signal_y_main < -1:
#                 signal_y_main = -1
#             signal_y_main = signal_y_main*0.5 +0.5

#             if (i> 0):

#                 kraj = True

#             if(getZ_reference() == 0 and abs(getZ_reference()-current_z) <0.3 and kraj ):
#                 rospy.sleep(50.0)
#                 Mjerenja.send_signals(signal_x=0,signal_y=0,signal_z=0,signal_yaw=0)
#                 Mjerenja.setNextMove(False)
#                 break;
#             i = i+1




#             rospy.sleep(50.0)
#             Mjerenja.send_signals(signal_x=signal_x_main,signal_y=signal_y_main,signal_z=signal_z_main,signal_yaw=signal_yaw_main)






if __name__ == '__main__':

    PID_x = PID(K_pf_num=0.7148, K_pf_den=0.2852, Kr=15.0, Ti=35.0, Td=0.4, v=10.0, Integrator_max=1.0, Integrator_min=-1.0, T_dis =0.084, Kr_i=0.00168, Kr_d=33.333333)

    PID_y = PID(K_pf_num=0.7148, K_pf_den=0.2852, Kr=7.0, Ti=35.0, Td=0.4, v=10.0, Integrator_max=1.0, Integrator_min=-1.0, T_dis = 0.084, Kr_i = 0.0168, Kr_d= 3.333333)

    PID_z = PID(K_pf_num=0.7198, K_pf_den=0.2802, Kr=5.0, Ti=30.0, Td=0.4, v=20.0, Integrator_max=50.0, Integrator_min=-50.0, T_dis = 0.2439, Kr_i = 0.0020, Kr_d= 0)
    # dobri paramtri 10.0 , 0.005 60% hover
    PID_yaw = PID(K_pf_num=0.7123, K_pf_den=0.2877, Kr=10.0, Ti=20.0, Td=0.1, v=10.0, Integrator_max=5.0, Integrator_min=0.0, T_dis = 0.084, Kr_i = 0.042, Kr_d= 11.904762)


    error_x_file = open("error_x.txt", "w+")

    rospy.init_node('PID')
    try:
        Mjerenja = Poruke()
    except rospy.ROSInterruptException:
        pass


    signal_x_main = 0.0
    signal_y_main =0.0
    signal_z_main = 0.0
    signal_yaw_main =0.0
    kraj = False
    while not rospy.is_shutdown():
        
        #rospy.loginfo ("**********%f", Mjerenja.getZ_reference())
        PID_x.setPoint(Mjerenja.getX_reference())
        PID_y.setPoint(Mjerenja.getY_reference())
        PID_z.setPoint(Mjerenja.getZ_reference())
        PID_yaw.setPoint(Mjerenja.getYAW_reference())


        current_x = Mjerenja.getX_feedback()
        current_y = Mjerenja.getY_feedback()
        current_z = Mjerenja.getZ_feedback()
        current_yaw = Mjerenja.getYaw_feedback()


        #signal_x_main = PID_x.update(current_x)
        #signal_y_main = PID_y.update(current_y)
        signal_z_main = PID_z.update(current_z)
        # signal_yaw_main = PID_yaw.update(current_yaw)

        rospy.loginfo("signal: %f", signal_z_main)
        if signal_z_main > 50.0:
            signal_z_main = 50.0
        elif signal_z_main < -50.0:
            signal_z_main = -50.0        
        signal_z_main = signal_z_main *(0.6/100.0) + 50*0.6/100.0 - 0.3 + 0.7 #signal_z_main * (0.8/100.0) +50.0*(0.8/100.0)- 0.4 +0.6  #signal_z_main*(0.3/100) +50*(0.3/100)-0.15 + 0.85 
        print "&&&&&&&&&&&&&&&&&&&&", signal_z_main
        if signal_x_main > 1:
            signal_x_main = 1
        elif signal_x_main < -1:
            signal_x_main = -1
        signal_x_main = signal_x_main * 0.5 + 0.5

        if signal_y_main > 1:
            signal_y_main = 1.0
        elif signal_y_main < -1:
            signal_y_main = -1.0
        signal_y_main = signal_y_main * 0.5 + 0.5

        #print "current_x"
        error_x_file.write(str(current_x))

        if(Mjerenja.getZ_reference() == 0 and abs(Mjerenja.getZ_reference()-current_z) <0.3 and kraj ):
            rospy.sleep(0.05)
            Mjerenja.send_signals(signal_x=0,signal_y=0,signal_z=0,signal_yaw=0)
            Mjerenja.setNextMove(False)
            break;




        rate = rospy.Rate (20)
        rate.sleep
        Mjerenja.send_signals(signal_x=signal_x_main,signal_y=signal_y_main,signal_z=signal_z_main,signal_yaw=signal_yaw_main)
    error_x_file.close()

