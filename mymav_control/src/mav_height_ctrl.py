#!/usr/bin/python
# -*- coding: utf-8 -*-

from matplotlib import pyplot as plt
import rospy
from pid import PID
from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import Imu

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

class HeightControl(object):
    
    '''
    Constructor for height control. Initializes parameters for PID controller.
    '''
    def __init__(self):
        
        self.clock = Clock()
        self.start_flag1 = False
        self.start_flag2 = False

        # Initialize controllers for height
        self.pid_h = PID()

        #Initalize controller parameters.
        self.pid_h.set_kp(14.823)
        self.pid_h.set_ki(0.461)
        self.pid_h.set_kd(45.22)
        self.pid_h.set_Tv(0.153)  #filter coeff

        #Seamless hybrid ctrl
        #self.pid_f.get_seamless(ui_from_height_ctrl)

        #self.pid_h.set_lim_high(838)
        #self.pid_h.set_lim_low(0)
        
        # Initialize controller frequency
        self.rate = 50
        self.ros_rate = rospy.Rate(self.rate)

        # Initialize subscribers
        rospy.Subscriber('/clock', Clock, self.clock_cb)
        rospy.Subscriber('/fiefly/command/pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/firefly/odometry_sensor1/odometry', Odometry, self.h_mv_cb)
        #rospy.Subscriber('/firefly/gazebo/command/motor_speed', Actuators, self.w_mv_cb)
        self.omega_pub = rospy.Publisher('/firefly/command/motor_speed', Actuators, queue_size=1)

    def run(self):

        while not self.start_flag1 and not self.start_flag2:
            print 'Waiting for the first measurement or reference'
            rospy.sleep(0.5)

        print 'Starting height control'
        
        # Starting reference
        self.h_ref = 1


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
     
            
            domega = self.pid_h.compute(self.h_ref, self.h_mv, dt_clk)
            omega= 547.59 + domega
            print 'omega', omega
            print dt_clk
            print 'h_ref: ', self.h_ref
            print 'h_mv: ', self.h_mv
            #saturation
            if omega < 0:
                omega = 0
            if omega > 838 :
                omega = 838
            #publish omega!!!
            omegaMsg=Actuators()   
            omegaMsg.angular_velocities=[omega, omega, omega, omega, omega, omega]      
            self.omega_pub.publish(omegaMsg)

    def clock_cb(self, msg):
        self.clock = msg

    def h_mv_cb(self, msg):
        self.start_flag1 = True
        self.h_mv = msg.pose.pose.position.z
    def pose_cb(self, msg):
        '''
        Pose (6DOF - position and orientation) callback.
        :param msg: Type PoseStamped
        '''
        self.z_mv = msg.pose.pose.position.z
    #def w_mv_cb(self,msg)
        #self.start_flag1 = True


if __name__ == '__main__':  

    rospy.init_node('mav_height_ctrl')
    height_ctrl = HeightControl()
    height_ctrl.run()
