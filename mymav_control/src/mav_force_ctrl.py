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
#from morus_msgs.cfg import MavAttitudeCtlParamsConfig 
import math
from datetime import datetime
from rosgraph_msgs.msg import Clock
from control_msgs.msg import JointControllerState
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators


class ForceControl(object):
    
    '''
    Constructor for force control. Initializes parameters for PID controller.
    '''
    def __init__(self):
        
        self.clock = Clock()
        self.start_flag1 = False
        self.start_flag2 = False

        # Initialize controllers for force
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
        
        # Initialize controller frequency
        self.rate = 50
        self.ros_rate = rospy.Rate(self.rate)

        # Initialize subscribers
        rospy.Subscriber('/clock', Clock, self.clock_cb)
        rospy.Subscriber('/firefly/ft_sensor_topic', WrenchStamped, self.f_mv_cb)
        #rospy.Subscriber('/firefly/command/pose', PoseStamped, self.pose_cb) 
        self.omega_pub = rospy.Publisher('/firefly/command/motor_speed', Actuators, queue_size=1)

    def run(self):

        while not self.start_flag1 and not self.start_flag2:
            print 'Waiting for the first measurement or reference'
            rospy.sleep(0.5)

        print 'Starting force control'
        
        # Starting reference
        self.f_ref = 10
    

        clock_old = self.clock

        while not rospy.is_shutdown():
            
            self.ros_rate.sleep()

            # Calculate dt
            clock_now = self.clock
            dt_clk = (clock_now.clock - clock_old.clock).to_sec()
            clock_old = clock_now

            if dt_clk < 10e-10:
                dt_clk = 0.05

            # Calculate new omega value 
            
            domega = self.pid_f.compute(self.f_ref, self.f_mv, dt_clk)
            omega = 540.783 - domega
            print 'domega', domega
            print 'omega', omega
            print dt_clk
            print 'f_ref: ', self.f_ref
            print 'f_mv: ', self.f_mv

            #publish omega!!!
            omegaMsg=Actuators()   
            omegaMsg.angular_velocities=[omega, omega, omega, omega, omega, omega]      
            self.omega_pub.publish(omegaMsg)

    def clock_cb(self, msg):
        self.clock = msg

    def f_mv_cb(self, msg):
        self.start_flag1 = True
        self.f_mv = msg.wrench.force.z


if __name__ == '__main__':  

    rospy.init_node('mav_force_ctrl')
    force_ctrl = ForceControl()
    force_ctrl.run()