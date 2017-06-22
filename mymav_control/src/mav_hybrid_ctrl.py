#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from pid import PID
from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
#from std_msgs.msg import Float32, Float64

from morus_msgs.msg import PIDController
#from dynamic_reconfigure.server import Server   
#from morus_msgs.cfg import MavZCtlParams

import math
from datetime import datetime
from rosgraph_msgs.msg import Clock
from control_msgs.msg import JointControllerState
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators

class HybridControl(object):
    
    '''
    Constructor for height control. Initializes parameters for PID controller.
    '''
    def __init__(self):
        
        self.clock = Clock()
        self.start_flag1 = False
        self.start_flag2 = False
        self.start_force = False 

        #two var for initial u_i passing between controllers
        self.passH = 0               
        self.passF = 0   #initially on ForceCTRL because of the inicial contact with the surface


        # Initialize controller for height
        self.pid_h = PID()

        #Initalize controller parameters.
        #----parameters empirically obtained----
        self.pid_h.set_kp(100)  #matlab 24.05873
        self.pid_h.set_ki(1)   #matlab 0.696
        self.pid_h.set_kd(100)   #matlab 116.1412
        self.pid_h.set_Tv(0.025)  #matlab filter coeff 0.2414)
        
        # Initialize controller for force
        self.pid_f = PID()

        #Initalize controller parameters.
        #----parameters empirically obtained----
        self.pid_f.set_kp(50)
        self.pid_f.set_ki(150)
        self.pid_f.set_kd(10) 
        self.pid_f.set_Tv(1) 
        #Seamless hybrid ctrl
        #self.pid_f.get_seamless(ui_from_height_ctrl)

        self.pid_f.set_lim_high(823)
        self.pid_f.set_lim_low(-823)
        
        #Seamless hybrid ctrl
        #self.pid_f.get_seamless(ui_height_ctrl)

        #self.pid_h.set_lim_high(838)
        #self.pid_h.set_lim_low(0)
        
        # Initialize controller frequency
        self.rate = 50
        self.ros_rate = rospy.Rate(self.rate)

        # Initialize subscribers
        rospy.Subscriber('/clock', Clock, self.clock_cb)
        rospy.Subscriber('/firefly/command/pose_ref', PoseStamped, self.pose_cb)
        rospy.Subscriber('/firefly/odometry_sensor1/odometry', Odometry, self.h_mv_cb)
        rospy.Subscriber('/firefly/ft_sensor_topic', WrenchStamped, self.f_mv_cb)
        rospy.Subscriber('/firefly/command/ft_ref', WrenchStamped, self.f_ref_cb) 
        self.mot_ref_pub = rospy.Publisher('/firefly/mot_vel_ref', Float32, queue_size=1)

    def run(self):

        while not self.start_flag1: #and not self.start_flag2 :
            print 'Waiting for the first measurement or reference'
            rospy.sleep(0.5)

        print 'Starting hybrid control'
        
        # Starting reference
        self.h_ref = 1
        self.f_ref = 5

        clock_old = self.clock

        while not rospy.is_shutdown():
            
            self.ros_rate.sleep()

            # Calculate dt
            clock_now = self.clock
            dt_clk = (clock_now.clock - clock_old.clock).to_sec()
            clock_old = clock_now

            if dt_clk < 10e-10:
                dt_clk = 0.05
            #dt_clk = 0.11
     
            if (self.start_force == False):       
                print 'HeightCTRL'
                domega = self.pid_h.compute(self.h_ref, self.h_mv, dt_clk)
                print 'h_ref: ', self.h_ref
                print 'h_mv: ', self.h_mv

            elif(self.start_force == True):
                print 'ForceCTRL'
                domega = -self.pid_f.compute(self.f_ref, self.f_mv, dt_clk)
                print 'f_ref: ', self.f_ref
                print 'f_mv: ', self.f_mv

            

            omega= 540.783 + domega
            #saturation
            if omega < 0:
                omega = 0
            if omega > 838 :
                omega = 838     
            print 'omega', omega
            print dt_clk


            #publish omega to a topic for attitude control
            omegaMsg=Float32(omega)     
            self.mot_ref_pub.publish(omegaMsg)

    def clock_cb(self, msg):
        self.clock = msg

    def h_mv_cb(self, msg):
        self.start_flag1 = True
        self.h_mv = msg.pose.pose.position.z

    def f_mv_cb(self, msg):
        #Seamless switch#
        self.f_mv = msg.wrench.force.z

        if (self.f_mv > 0.1):
            if (self.passH == True):
                self.passH = False
                self.passF = True
                self.u_height_ctrl=self.pid_h.get_pid_values()
                self.ui_seamless=self.u_height_ctrl[1]
                self.pid_f.set_seamless(self.ui_seamless)
            self.start_force = True    

        elif(self.f_mv < 0):
            if (self.passF == True):
                self.passF = False
                self.passH = True
                self.u_force_ctrl=self.pid_f.get_pid_values()
                self.ui_seamless=self.u_force_ctrl[1]
                self.pid_h.set_seamless(self.ui_seamless)
            self.start_force = False

    def pose_cb(self, msg):
        self.h_ref = msg.pose.position.z

    def f_ref_cb(self, msg):
        self.f_ref = msg.wrench.force.z 

if __name__ == '__main__':  

    rospy.init_node('mav_hybrid_ctrl')
    hybrid_ctrl = HybridControl()
    hybrid_ctrl.run()
