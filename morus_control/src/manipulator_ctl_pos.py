#!/usr/bin/python
# -*- coding: utf-8 -*-


import rospy
from pid import PID
from geometry_msgs.msg import Vector3, Vector3Stamped, \
    PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Float64
from morus_msgs.msg import PIDController
from dynamic_reconfigure.server import Server
from morus_msgs.cfg import MavAttitudeCtlParamsConfig
import math
from datetime import datetime
from rosgraph_msgs.msg import Clock
from control_msgs.msg import JointControllerState
from nav_msgs.msg import Odometry

class PositionControl(object):
    
    '''
    Constructor for position control. Initializes parameters for PID controller.
    '''
    def __init__(self):
        
        self.clock = Clock()
        self.start_flag1 = False
        self.start_flag2 = False

        # Initialize controllers for position and velocity
        self.pid_x = PID()
        self.pid_vx = PID()

        self.pid_y = PID()
        self.pid_vy = PID()

        #Initalize controller parameters.
        self.pid_x.set_kp(0.25)
        self.pid_x.set_kd(0.008)
        self.pid_x.set_ki(0)

        self.pid_vx.set_kp(0.25)
        self.pid_vx.set_kd(0)
        self.pid_vx.set_ki(0)
        self.pid_vx.set_lim_high(0.05)
        self.pid_vx.set_lim_low(-0.05)

        self.pid_y.set_kp(0.25)
        self.pid_y.set_kd(0.008)
        self.pid_y.set_ki(0)

        self.pid_vy.set_kp(0.25)
        self.pid_vy.set_kd(0)
        self.pid_vy.set_ki(0)
        self.pid_vy.set_lim_high(0.05)
        self.pid_vy.set_lim_low(-0.05)
        
        # Initialize controller frequency
        self.rate = 50
        self.ros_rate = rospy.Rate(self.rate)

        # Initialize subscribers
        rospy.Subscriber('/clock', Clock, self.clock_cb)
        rospy.Subscriber('/morus/odometry', Odometry, self.pose_cb)
        rospy.Subscriber('/morus/pos_ref', Vector3, self.pos_ref_cb)

        self.euler_pub = rospy.Publisher('/morus/euler_ref', Vector3, queue_size=1)

    def run(self):

        while not self.start_flag1 and not self.start_flag2:
            print 'Waiting for the first measurement'
            rospy.sleep(0.5)

        print 'Starting position control.'

        clock_old = self.clock

        while not rospy.is_shutdown():
            
            self.ros_rate.sleep()

            # Calculate dt
            clock_now = self.clock
            dt_clk = (clock_now.clock - clock_old.clock).to_sec()
            clock_old = clock_now

            if dt_clk < 10e-10:
                dt_clk = 0.05

            # Calculate new roll value
            vx_sp = self.pid_x.compute(self.x_ref, self.x_mv, dt_clk)
            roll = self.pid_vx.compute(vx_sp, self.vx_mv, dt_clk)

            # Calculate new pitch value
            vy_sp = self.pid_y.compute(self.y_ref, self.y_mv, dt_clk)
            pitch = self.pid_vy.compute(vy_sp, self.vy_mv, dt_clk) 

            print ''
            print dt_clk
            print 'x_ref: ', self.x_ref, ' y_ref: ', self.y_ref
            print 'vx_sp: ', vx_sp, ' vy_sp: ', vy_sp
            print 'vx_mv: ', self.vx_mv, ' vy_mv: ', self.vy_mv
            print 'roll: ', roll, ' pitch: ', pitch
            
            newMsg = Vector3()
            newMsg.x = -pitch
            newMsg.y = roll
            newMsg.z = 0

            self.euler_pub.publish(newMsg)


    def clock_cb(self, msg):
        self.clock = msg

    def pose_cb(self, msg):
        self.start_flag1 = True
        self.x_mv = msg.pose.pose.position.x
        self.y_mv = msg.pose.pose.position.y
        self.vx_mv = msg.twist.twist.linear.x
        self.vy_mv = msg.twist.twist.linear.y

    def pos_ref_cb(self, msg):
        self.start_flag2 = True
        self.x_ref = msg.x
        self.y_ref = msg.y

if __name__ == '__main__':  

    rospy.init_node('mav_position_ctl')
    position_ctl = PositionControl()
    position_ctl.run()