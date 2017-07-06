#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from pid import PID
from pid2 import PID2
from geometry_msgs.msg import Vector3, Vector3Stamped, PoseStamped, WrenchStamped
from std_msgs.msg import Float32
import math
from morus_msgs.msg import PIDController
from datetime import datetime
from rosgraph_msgs.msg import Clock
from control_msgs.msg import JointControllerState
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators

class AttitudeControl:
    '''
    Class implements MAV attitude control (roll, pitch, yaw). Two PIDs in cascade are
    used for each degree of freedom.
    Subscribes to:
        /firefly/odometry_sensor1/odometry               - used to extract attitude and attitude rate of the vehicle
        /firefly/mot_vel_ref        - used to receive referent motor velocity from the height controller
        /firefly/euler_ref          - used to set the attitude referent (useful for testing controllers)
    Publishes:
        /firefly/command/motor_speed     - referent motor velocities sent to each motor controller
    
    '''

    def __init__(self):
        '''
        Initialization of the class.
        '''

        self.start_flag = False             # flag indicates if the first measurement is received
        self.config_start = False           # flag indicates if the config callback is called for the first time

        self.w_sp = 0
        self.euler_sp = Vector3(0, 0, 0)
        # init euler angles referent values            
        self.euler_mv=Vector3()
        # init measured angular velocities
        self.euler_rate_mv=Vector3()

        self.pid_roll = PID2()                           # roll controller
        self.pid_roll_rate  = PID2()                     # roll rate (wx) controller

        self.pid_pitch = PID2()                          # pitch controller
        self.pid_pitch_rate = PID2()                     # pitch rate (wy) controller

        ##################################################################
        ##################################################################
        # Add your PID params here

        self.pid_roll.set_kp(14.5)
        self.pid_roll.set_ki(2.4)
        self.pid_roll.set_kd(2.4)

        self.pid_roll_rate.set_kp(17.5)
        self.pid_roll_rate.set_ki(7.1)
        self.pid_roll_rate.set_kd(1)

        self.pid_pitch.set_kp(14.5)
        self.pid_pitch.set_ki(2.4)
        self.pid_pitch.set_kd(2.4)

        self.pid_pitch_rate.set_kp(17.5)
        self.pid_pitch_rate.set_ki(7.1)
        self.pid_pitch_rate.set_kd(1)

        ##################################################################
        ##################################################################

        self.ros_rate = rospy.Rate(100)                 # attitude control at 100 Hz

        rospy.Subscriber('/firefly/odometry_sensor1/odometry', Odometry,self.ahrs_cb)
        rospy.Subscriber('/firefly/mot_vel_ref', Float32, self.mot_vel_ref_cb)
        rospy.Subscriber('/firefly/euler_ref', Vector3, self.euler_ref_cb)
        self.pub_mot = rospy.Publisher('/firefly/command/motor_speed', Actuators, queue_size=1)
        self.pub_euler = rospy.Publisher('/firefly/euler_mv', Vector3, queue_size=1)
        self.pub_euler_rate = rospy.Publisher('/firefly/euler_rate_mv', Vector3, queue_size=1)
    def run(self):
        '''
        Runs ROS node - computes PID algorithms for cascade attitude control.
        '''

        while not self.start_flag:
            print "Waiting for the first measurement."
            rospy.sleep(0.5)
        print "Starting attitude control."

        while not rospy.is_shutdown():
            self.ros_rate.sleep()

            ####################################################################
            ####################################################################
            # Add your code for cascade control for roll, pitch, yaw.
            # Referent attitude values are stored in self.euler_sp
            # (self.euler_sp.x - roll, self.euler_sp.y - pitch, self.euler_sp.z - yaw)
            # Measured attitude values are stored in self.euler_mv (x,y,z - roll, pitch, yaw)
            # Measured attitude rate values are store in self.euler_rate_mv (self.euler_rate_mv.x, y, z)
            # Your result should be referent velocity value for each motor.
            # Store them in variables mot_sp1, mot_sp2, mot_sp3, mot_sp4, mot_sp5, mot_sp6

            #FIREFLY ROTOR CONFIG  <color>.<number>
            #######################
            #     R.1     R.6     #
            # B.2             B.5 #
            #     B.3     B.4     #
            ####################### 

            #roll
            u=self.pid_roll.compute(self.euler_sp.x,self.euler_mv.x)
            mot_speedr=self.pid_roll_rate.compute(u,self.euler_rate_mv.x)

            mot_sp1_r=mot_speedr
            mot_sp2_r=mot_speedr
            mot_sp3_r=mot_speedr
            mot_sp4_r=-mot_speedr
            mot_sp5_r=-mot_speedr
            mot_sp6_r=-mot_speedr

            #pitch
            u=self.pid_pitch.compute(self.euler_sp.y,self.euler_mv.y)
            mot_speedp=self.pid_pitch_rate.compute(u,self.euler_rate_mv.y)
            mot_sp1_p=-mot_speedp
            mot_sp2_p=0
            mot_sp3_p=mot_speedp
            mot_sp4_p=mot_speedp
            mot_sp5_p=0
            mot_sp6_p=-mot_speedp

            #bez yaw 

            mot_sp1_y=0
            mot_sp2_y=0
            mot_sp3_y=0
            mot_sp4_y=0
            mot_sp5_y=0
            mot_sp6_y=0


            
            mot_sp1=self.w_sp+mot_sp1_r+mot_sp1_p+mot_sp1_y
            mot_sp2=self.w_sp+mot_sp2_r+mot_sp2_p+mot_sp2_y
            mot_sp3=self.w_sp+mot_sp3_r+mot_sp3_p+mot_sp3_y
            mot_sp4=self.w_sp+mot_sp4_r+mot_sp4_p+mot_sp4_y
            mot_sp5=self.w_sp+mot_sp5_r+mot_sp5_p+mot_sp5_y
            mot_sp6=self.w_sp+mot_sp6_r+mot_sp6_p+mot_sp6_y

            ####################################################################
            ####################################################################

            # Publish motor velocities
            mot_speed_msg = Actuators()
            mot_speed_msg.angular_velocities = [mot_sp1,mot_sp2,mot_sp3,mot_sp4,mot_sp5,mot_sp6]
            self.pub_mot.publish(mot_speed_msg)
            # measured euler angle
            
            self.pub_euler.publish(self.euler_mv)
            # measured angular velocities
        
            self.pub_euler_rate.publish(self.euler_rate_mv)

    def mot_vel_ref_cb(self, msg):
        '''
        Referent motor velocity callback. (This should be published by height controller).
        :param msg: Type Float32
        '''
        self.w_sp = msg.data

    def ahrs_cb(self, msg):
        '''
        AHRS callback. Used to extract roll, pitch, yaw and their rates.
        We used the following order of rotation - 1)yaw, 2) pitch, 3) roll
        :param msg: nav_msgs.msg/Odometry
        '''
        if not self.start_flag:
            self.start_flag = True

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # conversion quaternion to euler (yaw - pitch - roll)
        self.euler_mv.x = math.atan2(2 * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz)
        self.euler_mv.y = math.asin(2 * (qw * qy - qx * qz))
        self.euler_mv.z = math.atan2(2 * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz)

        # gyro measurements (p,q,r)
        p = msg.twist.twist.angular.x
        q = msg.twist.twist.angular.y
        r = msg.twist.twist.angular.z

        sx = math.sin(self.euler_mv.x)   # sin(roll)
        cx = math.cos(self.euler_mv.x)   # cos(roll)
        cy = math.cos(self.euler_mv.y)   # cos(pitch)
        ty = math.tan(self.euler_mv.y)   # cos(pitch)

        # conversion gyro measurements to roll_rate, pitch_rate, yaw_rate
        self.euler_rate_mv.x = p + sx * ty * q + cx * ty * r
        self.euler_rate_mv.y = cx * q - sx * r
        self.euler_rate_mv.z = sx / cy * q + cx / cy * r



    def euler_ref_cb(self, msg):
        ''' 
        Euler ref values callback.
        :param msg: Type Vector3 (x-roll, y-pitch, z-yaw)
        '''
        self.euler_sp = msg

if __name__ == '__main__':

    rospy.init_node('mav_attitude_ctl')
    attitude_ctl = AttitudeControl()
    attitude_ctl.run()
