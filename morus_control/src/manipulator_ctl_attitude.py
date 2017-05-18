#!/usr/bin/python
# -*- coding: utf-8 -*-

__author__ = 'thaus'

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


class AttitudeControl:

    '''
    Class implements MAV attitude control (roll, pitch, yaw). Two PIDs in cascade are
    used for each degree of freedom.
    Subscribes to:
        /morus/imu                - used to extract attitude and attitude rate of the vehicle
        /morus/mot_vel_ref        - used to receive referent motor velocity from the height controller
        /morus/euler_ref          - used to set the attitude referent (useful for testing controllers)
    Publishes:
        /morus/command/motors     - referent motor velocities sent to each motor controller
        /morus/pid_roll           - publishes PID-roll data - referent value, measured value, P, I, D and total component (useful for tuning params)
        /morus/pid_roll_rate      - publishes PID-roll_rate data - referent value, measured value, P, I, D and total component (useful for tuning params)
        /morus/pid_pitch          - publishes PID-pitch data - referent value, measured value, P, I, D and total component (useful for tuning params)
        /morus/pid_pitch_rate     - publishes PID-pitch_rate data - referent value, measured value, P, I, D and total component (useful for tuning params)
        /morus/pid_yaw            - publishes PID-yaw data - referent value, measured value, P, I, D and total component (useful for tuning params)
        /morus/pid_yaw_rate       - publishes PID-yaw_rate data - referent value, measured value, P, I, D and total component (useful for tuning params)

    Dynamic reconfigure is used to set controllers param online.
    '''

    def __init__(self):
        '''
        Initialization of the class.
        '''

        self.start_flag = False  # flag indicates if the first measurement is received
        self.config_start = False  # flag indicates if the config callback is called for the first time
        self.euler_mv = Vector3()  # measured euler angles
        self.euler_sp = Vector3(0, 0, 0)  # euler angles referent values

        self.w_sp = 0  # referent value for motor velocity - it should be the output of height controller

        self.euler_rate_mv = Vector3()  # measured angular velocities

        self.clock = Clock()

        self.pid_roll = PID()  # roll controller
        self.pid_roll_rate = PID()  # roll rate (wx) controller

        self.pid_pitch = PID()  # pitch controller
        self.pid_pitch_rate = PID()  # pitch rate (wy) controller

        self.pid_yaw = PID()  # yaw controller
        self.pid_yaw_rate = PID()  # yaw rate (wz) controller

        # #################################################################
        # #################################################################
        # Add your PID params here

        self.pid_roll.set_kp(0.05)
        self.pid_roll.set_ki(0.0)
        self.pid_roll.set_kd(0)

        self.pid_roll_rate.set_kp(0.25)
        self.pid_roll_rate.set_ki(0.0)
        self.pid_roll_rate.set_kd(0)
        self.pid_roll_rate.set_lim_high(0.001)
        self.pid_roll_rate.set_lim_low(-0.001)

        self.pid_pitch.set_kp(0.05)
        self.pid_pitch.set_ki(0)
        self.pid_pitch.set_kd(0)

        self.pid_pitch_rate.set_kp(0.25)
        self.pid_pitch_rate.set_ki(0.0)
        self.pid_pitch_rate.set_kd(0)
        self.pid_pitch_rate.set_lim_high(0.001)
        self.pid_pitch_rate.set_lim_low(-0.001)
        
        self.pid_yaw.set_kp(0)
        self.pid_yaw.set_ki(0)
        self.pid_yaw.set_kd(0)

        self.pid_yaw_rate.set_kp(0)
        self.pid_yaw_rate.set_ki(0)
        self.pid_yaw_rate.set_kd(0)

        # #################################################################
        # #################################################################

        self.rate = 100.0
        self.ros_rate = rospy.Rate(self.rate)  # attitude control at 100 Hz

        self.t_old = 0

        rospy.Subscriber('imu', Imu, self.ahrs_cb)
        rospy.Subscriber('mot_vel_ref', Float32, self.mot_vel_ref_cb)
        rospy.Subscriber('euler_ref', Vector3, self.euler_ref_cb)
        rospy.Subscriber('/clock', Clock, self.clock_cb)

        # Joint publishers
        self.pub_joint0_left = \
            rospy.Publisher('joint0_left_controller/command', Float64,
                            queue_size=1)
        self.pub_joint1_left = \
            rospy.Publisher('joint1_left_controller/command', Float64,
                            queue_size=1)
        self.pub_joint0_right = \
            rospy.Publisher('joint0_right_controller/command', Float64,
                            queue_size=1)
        self.pub_joint1_right = \
            rospy.Publisher('joint1_right_controller/command', Float64,
                            queue_size=1)


        self.pub_pid_roll = rospy.Publisher('/pid_roll', PIDController,
                queue_size=1)
        self.pub_pid_roll_rate = rospy.Publisher('pid_roll_rate',
                PIDController, queue_size=1)
        self.pub_pid_pitch = rospy.Publisher('pid_pitch',
                PIDController, queue_size=1)
        self.pub_pid_pitch_rate = rospy.Publisher('pid_pitch_rate',
                PIDController, queue_size=1)
        self.pub_pid_yaw = rospy.Publisher('pid_yaw', PIDController,
                queue_size=1)
        self.pub_pid_yaw_rate = rospy.Publisher('pid_yaw_rate',
                PIDController, queue_size=1)
        self.cfg_server = Server(MavAttitudeCtlParamsConfig,
                                 self.cfg_callback)


    def run(self):
        '''
        Runs ROS node - computes PID algorithms for cascade attitude control.
        '''

        while rospy.get_time() == 0:
            print 'Waiting for clock server to start'

        print 'Received first clock message'

        while not self.start_flag:
            print 'Waiting for the first measurement. (1)'
            rospy.sleep(0.5)
        print 'Starting attitude control.'

        self.t_old = rospy.Time.now()
        clock_old = self.clock

        # self.t_old = datetime.now()

        self.count = 0
        self.loop_count = 0

        while not rospy.is_shutdown():

            #self.ros_rate.sleep()

            rospy.sleep(0.1)
            if not self.start_flag:
                print 'Waiting for the first IMU measurement.'
                rospy.sleep(0.5)
            else:
                clock_now = self.clock
                dt_clk = (clock_now.clock - clock_old.clock).to_sec()

                clock_old = clock_now
                if dt_clk > 1.0 / self.rate + 0.005:
                    self.count += 1
                    print self.count, ' - ', dt_clk

                if dt_clk < 1.0 / self.rate - 0.005:
                    self.count += 1
                    print self.count, ' - ', dt_clk

                roll_rate_sv = self.pid_roll.compute(self.euler_sp.x,
                        self.euler_mv.x, dt_clk)

                print "x_sp ", self.euler_sp.x, " x_mv ", self.euler_mv.x 
                # roll rate pid compute

                dy_roll = self.pid_roll_rate.compute(roll_rate_sv,
                        self.euler_rate_mv.x, dt_clk)

                print "y_sp ", self.euler_sp.y, " y_mv ", self.euler_mv.y 

                pitch_rate_sv = self.pid_pitch.compute(self.euler_sp.y,
                        self.euler_mv.y, dt_clk)

                # pitch rate pid compute

                dx_pitch = self.pid_pitch_rate.compute(pitch_rate_sv,
                        self.euler_rate_mv.y, dt_clk)

                # Calculating angles - inverse Jacobian --------------------------------------------------------------
                l = 0.5

                #saturation = 0.01

                #if dx_pitch > saturation:
                #    dx_pitch = saturation
                #if dx_pitch < -saturation:
                #    dx_pitch = -saturation

                #if dy_roll > saturation:
                #    dy_roll = saturation
                #if dy_roll < -saturation:
                #    dy_roll = -saturation

                q1L = 1.0298
                q2L = -2.0596

                # New angle increments (left)
                try:
                    dq1L = ( math.cos(q1L + q2L) / (l * math.sin(q2L)) ) * (-dx_pitch) + \
                           ( math.sin(q1L + q2L) / (l * math.sin(q2L)) ) * (-dy_roll)
                    dq2L = -( math.cos(q1L + q2L) + math.cos(q1L) ) / (l * math.sin(q2L)) * (-dx_pitch) \
                           - ( math.sin(q1L + q2L) + math.sin(q2L) ) / (l * math.sin(q2L)) * (-dy_roll)
                except ZeroDivisionError:
                    print "ZeroDivisionError"
                    # TODO Handle zero division

                q1R = 1.0298
                q2R = -2.0596

                # New angle increments (right)
                try:
                    dq1R = math.cos(q1R + q2R) / (l * math.sin(q2R)) * (dx_pitch) \
                       + math.sin(q1R + q2R) / (l * math.sin(q2R)) * (dy_roll)
                    dq2R = - (math.cos(q1R + q2R) + math.cos(q1R)) / (l * math.sin(q2R)) * (dx_pitch) \
                       - (math.sin(q1R + q2R) + math.sin(q2R)) / (l * math.sin(q2R)) * (dy_roll)
                except ZeroDivisionError:
                    print "ZeroDivisionError"
                    # TODO Handle zero division
                

                # Make all new messagees
                joint0_left_command_msg = Float64()
                joint0_left_command_msg.data = q1L + dq1L

                joint1_left_command_msg = Float64()
                joint1_left_command_msg.data = q2L + dq2L

                joint0_right_command_msg = Float64()
                joint0_right_command_msg.data = q1R + dq1R

                joint1_right_command_msg = Float64()
                joint1_right_command_msg.data = q2R + dq2R

                # Publish all new messages

                self.pub_joint0_left.publish(joint0_left_command_msg)
                self.pub_joint1_left.publish(joint1_left_command_msg)
                self.pub_joint0_right.publish(joint0_right_command_msg)
                self.pub_joint1_right.publish(joint1_right_command_msg)

                # Publish PID data - could be usefule for tuning
                self.pub_pid_roll.publish(self.pid_roll.create_msg())
                self.pub_pid_roll_rate.publish(self.pid_roll_rate.create_msg())
                self.pub_pid_pitch.publish(self.pid_pitch.create_msg())
                self.pub_pid_pitch_rate.publish(self.pid_pitch_rate.create_msg())
                self.pub_pid_yaw.publish(self.pid_yaw.create_msg())
                self.pub_pid_yaw_rate.publish(self.pid_yaw_rate.create_msg())

                #print ""
                print "dx = ", dx_pitch, " dy = ", dy_roll
                #print 'dq1L = ', dq1L, ' dq2L = ', dq2L
                #print 'dq1R = ', dq1R, ' dq2R = ', dq2R

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
        :param msg: Type sensor_msgs/Imu
        '''

        if not self.start_flag:
            self.start_flag = True

        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        # conversion quaternion to euler (yaw - pitch - roll)

        self.euler_mv.x = math.atan2(2 * (qw * qx + qy * qz), qw * qw
                - qx * qx - qy * qy + qz * qz)
        self.euler_mv.y = math.asin(2 * (qw * qy - qx * qz))
        self.euler_mv.z = math.atan2(2 * (qw * qz + qx * qy), qw * qw
                + qx * qx - qy * qy - qz * qz)

        # gyro measurements (p,q,r)

        p = msg.angular_velocity.x
        q = msg.angular_velocity.y
        r = msg.angular_velocity.z

        sx = math.sin(self.euler_mv.x)  # sin(roll)
        cx = math.cos(self.euler_mv.x)  # cos(roll)
        cy = math.cos(self.euler_mv.y)  # cos(pitch)
        ty = math.tan(self.euler_mv.y)  # cos(pitch)

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

    def clock_cb(self, msg):
        self.clock = msg
        

    def cfg_callback(self, config, level):
        """ Callback for dynamically reconfigurable parameters (P,I,D gains for each controller)
        """

        if not self.config_start:

            # callback is called for the first time. Use this to set the new params to the config server

            config.roll_kp = self.pid_roll.get_kp()
            config.roll_ki = self.pid_roll.get_ki()
            config.roll_kd = self.pid_roll.get_kd()

            config.roll_r_kp = self.pid_roll_rate.get_kp()
            config.roll_r_ki = self.pid_roll_rate.get_ki()
            config.roll_r_kd = self.pid_roll_rate.get_kd()

            config.pitch_kp = self.pid_pitch.get_kp()
            config.pitch_ki = self.pid_pitch.get_ki()
            config.pitch_kd = self.pid_pitch.get_kd()

            config.pitch_r_kp = self.pid_pitch_rate.get_kp()
            config.pitch_r_ki = self.pid_pitch_rate.get_ki()
            config.pitch_r_kd = self.pid_pitch_rate.get_kd()

            config.yaw_kp = self.pid_yaw.get_kp()
            config.yaw_ki = self.pid_yaw.get_ki()
            config.yaw_kd = self.pid_yaw.get_kd()

            config.yaw_r_kp = self.pid_yaw_rate.get_kp()
            config.yaw_r_ki = self.pid_yaw_rate.get_ki()
            config.yaw_r_kd = self.pid_yaw_rate.get_kd()

            self.config_start = True
        else:

            # The following code just sets up the P,I,D gains for all controllers

            self.pid_roll.set_kp(config.roll_kp)
            self.pid_roll.set_ki(config.roll_ki)
            self.pid_roll.set_kd(config.roll_kd)

            self.pid_roll_rate.set_kp(config.roll_r_kp)
            self.pid_roll_rate.set_ki(config.roll_r_ki)
            self.pid_roll_rate.set_kd(config.roll_r_kd)

            self.pid_pitch.set_kp(config.pitch_kp)
            self.pid_pitch.set_ki(config.pitch_ki)
            self.pid_pitch.set_kd(config.pitch_kd)

            self.pid_pitch_rate.set_kp(config.pitch_r_kp)
            self.pid_pitch_rate.set_ki(config.pitch_r_ki)
            self.pid_pitch_rate.set_kd(config.pitch_r_kd)

            self.pid_yaw.set_kp(config.yaw_kp)
            self.pid_yaw.set_kp(config.yaw_kp)
            self.pid_yaw.set_ki(config.yaw_ki)
            self.pid_yaw.set_kd(config.yaw_kd)

            self.pid_yaw_rate.set_kp(config.yaw_r_kp)
            self.pid_yaw_rate.set_ki(config.yaw_r_ki)
            self.pid_yaw_rate.set_kd(config.yaw_r_kd)

        # this callback should return config data back to server

        return config


if __name__ == '__main__':

    rospy.init_node('mav_attitude_ctl')
    rospy.sleep(10)
    attitude_ctl = AttitudeControl()
    attitude_ctl.run()


			