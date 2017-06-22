#!/usr/bin/python
# -*- coding: utf-8 -*-

from matplotlib import pyplot as plt
import rospy
from pid import PID
from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import Imu

from std_msgs.msg import Float32

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
        #----parameters empirically obtained----
        self.pid_h.set_kp(100)  #matlab 24.05873
        self.pid_h.set_ki(1)   #matlab 0.696
        self.pid_h.set_kd(100)   #matlab 116.1412
        self.pid_h.set_Tv(0.025)  #matlab filter coeff 0.2414)

        #Seamless hybrid ctrl
        #self.pid_f.get_seamless(ui_from_height_ctrl)

        #self.pid_h.set_lim_high(0.05)
        #self.pid_h.set_lim_low(-0.05)
        self.val=self.pid_h.get_pid_values()
        print 'val' , self.val[1]
        # Initialize controller frequency
        self.rate = 50
        self.ros_rate = rospy.Rate(self.rate)

        # Initialize subscribers
        rospy.Subscriber('/clock', Clock, self.clock_cb)
        rospy.Subscriber('/firefly/command/pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/firefly/odometry_sensor1/odometry', Odometry, self.h_mv_cb)
        #self.omega_pub = rospy.Publisher('/firefly/command/motor_speed', Actuators, queue_size=1)
        self.mot_ref_pub = rospy.Publisher('/firefly/mot_vel_ref', Float32, queue_size=1)

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
        
            self.domega = self.pid_h.compute(self.h_ref, self.h_mv, dt_clk)
            self.omega= 547.59 + self.domega
            print 'omega', self.omega
            print dt_clk
            print 'h_ref: ', self.h_ref
            print 'h_mv: ', self.h_mv
            #saturation
            if self.omega < 0:
                self.omega = 0
            if self.omega > 838 :
                self.omega = 838
            #publish omega!!!

            self.attitude_ctl = 1  # don't forget to set me to 1 when you implement attitude ctl

            ########################################################
            ########################################################

            if self.attitude_ctl == 0:
                # Publish motor velocities
                omegaMsg=Actuators()   
                omegaMsg.angular_velocities=[self.omega, self.omega, self.omega, self.omega, self.omega, self.omega]      
                self.omega_pub.publish(omegaMsg)


            else:
                # publish referent motor velocity to attitude controller
                omegaMsg = Float32(self.omega)
                self.mot_ref_pub.publish(omegaMsg)
        
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
        self.h_ref = msg.pose.position.z


if __name__ == '__main__':  

    rospy.init_node('mav_height_ctrl')
    height_ctrl = HeightControl()
    height_ctrl.run()
