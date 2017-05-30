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

        # Matknuti integrator 
        # P - 7-5, D- 5, P - 5, D - 2.5

        self.pid_roll.set_kp(3.0)
        self.pid_roll.set_ki(0.0)
        self.pid_roll.set_kd(0)

        self.pid_roll_rate.set_kp(2.5)
        self.pid_roll_rate.set_ki(0.0)
        self.pid_roll_rate.set_kd(0)
        self.pid_roll_rate.set_lim_high(0.3)
        self.pid_roll_rate.set_lim_low(-0.3)

        self.pid_pitch.set_kp(3)
        self.pid_pitch.set_ki(0.0)
        self.pid_pitch.set_kd(0)

        self.pid_pitch_rate.set_kp(2.5)
        self.pid_pitch_rate.set_ki(0.0)
        self.pid_pitch_rate.set_kd(0)
        self.pid_pitch_rate.set_lim_high(0.3)
        self.pid_pitch_rate.set_lim_low(-0.3)

        self.pid_yaw.set_kp(0)
        self.pid_yaw.set_ki(0)
        self.pid_yaw.set_kd(0)

        self.pid_yaw_rate.set_kp(0)
        self.pid_yaw_rate.set_ki(0)
        self.pid_yaw_rate.set_kd(0)

        # #################################################################
        # #################################################################

        #self.rate = 100.0
        self.rate = 50
        self.ros_rate = rospy.Rate(self.rate)  # attitude control at 100 Hz

        self.t_old = 0

        rospy.Subscriber('imu', Imu, self.ahrs_cb)
        rospy.Subscriber('mot_vel_ref', Float32, self.mot_vel_ref_cb)
        rospy.Subscriber('euler_ref', Vector3, self.euler_ref_cb)
        rospy.Subscriber('/clock', Clock, self.clock_cb)

        self.euluer_mv_pub = rospy.Publisher('euler_mv', Vector3, queue_size=1)

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


        self.sub_joint0_left = rospy.Subscriber('joint0_left_controller/state', JointControllerState, self.joint0_left_cb)
        self.sub_joint1_left = rospy.Subscriber('joint1_left_controller/state', JointControllerState, self.joint1_left_cb)
        self.sub_joint0_right = rospy.Subscriber('joint0_right_controller/state', JointControllerState, self.joint0_right_cb)
        self.sub_joint1_right = rospy.Subscriber('joint1_right_controller/state', JointControllerState, self.joint1_right_cb)

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

        # Length of a single manipulator arm
        self.l = 0.5

        # Initial offset from the origin of the manipulator arm to the initial position
        self.x_offset = 0.515
        self.y_offset = 0

        # Inizalize refernce filter
        self.euler_x_old = 0
        self.euler_y_old = 0

    def initial_position(self):
        print 'Setting initial position'

        rospy.sleep(5)

        q1_initial = 1.0298
        q2_initial = -2.0596

        initial_joint0_left_msg = Float64()
        initial_joint0_left_msg.data = q1_initial

        initial_joint1_left_msg = Float64()
        initial_joint1_left_msg.data = q2_initial
        
        initial_joint0_right_msg = Float64()
        initial_joint0_right_msg.data = q1_initial

        initial_joint1_right_msg = Float64()
        initial_joint1_right_msg.data = q2_initial

        self.pub_joint0_left.publish(initial_joint0_left_msg)
        self.pub_joint1_left.publish(initial_joint1_left_msg)
        self.pub_joint0_right.publish(initial_joint0_right_msg)
        self.pub_joint1_right.publish(initial_joint1_right_msg)

        rospy.sleep(5)

    def run(self):
        '''
        Runs ROS node - computes PID algorithms for cascade attitude control.
        '''

        while rospy.get_time() == 0:
            print 'Waiting for clock server to start'

        print 'Received first clock message - manipulator control'

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

            self.ros_rate.sleep()

            #rospy.sleep(0.02)
                
            
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

                
                # dt for some reason is 0 sometimes... 
                # Causes zero division error in controllers
                if dt_clk < 10e-10:
                    dt_clk = 0.05

                # First order reference filter - x
                if abs(self.euler_x_old - self.euler_sp.x) > 0.03:
                    a = 0.99
                else:
                    a = 0.9
                ref_x = self.euler_x_old + (1-a) * (self.euler_sp.x - self.euler_x_old)
                self.euler_x_old = ref_x

                roll_rate_sv = self.pid_roll.compute(ref_x,
                        self.euler_mv.x, dt_clk)

                print "x_sp ", ref_x, ' x_mv: ', self.euler_mv.x
                
                # roll rate pid compute
                dy_roll = self.pid_roll_rate.compute(roll_rate_sv,
                        self.euler_rate_mv.x, dt_clk)

                # Publish new euler measured values
                vectorMsg = Vector3()
                vectorMsg.x = self.euler_mv.x
                vectorMsg.y = self.euler_mv.y
                self.euluer_mv_pub.publish(vectorMsg)

                # First order reference filter - y
                if abs(self.euler_y_old - self.euler_sp.y) > 0.03:
                    a = 0.99
                else:
                    a = 0.9
                ref_y = self.euler_y_old + (1-a) * (self.euler_sp.y - self.euler_y_old)
                self.euler_y_old = ref_y

                print "y_sp ", ref_y, ' y_mv: ', self.euler_mv.y

                pitch_rate_sv = self.pid_pitch.compute(ref_y,
                        self.euler_mv.y, dt_clk)

                # pitch rate pid compute

                dx_pitch = self.pid_pitch_rate.compute(pitch_rate_sv,
                        self.euler_rate_mv.y, dt_clk)

                # Calculating angles - inverse Jacobian 
                dqL = self.get_new_dqL(dx_pitch, -dy_roll, 0.15)
                dqR = self.get_new_dqR(dx_pitch, -dy_roll, 0.15)
       
                # Calculate new joint values
                joint0_right_command_msg = Float64()
                joint0_right_command_msg.data = self.q1R + dqR[0]

                joint1_right_command_msg = Float64()
                joint1_right_command_msg.data = self.q2R + dqR[1]

                joint0_left_command_msg = Float64()
                joint0_left_command_msg.data = self.q1L + dqL[0]

                joint1_left_command_msg = Float64()
                joint1_left_command_msg.data = self.q2L + dqL[1]


                # Publish new joint values
                self.pub_joint0_right.publish(joint0_right_command_msg)
                self.pub_joint1_right.publish(joint1_right_command_msg)
                self.pub_joint0_left.publish(joint0_left_command_msg)
                self.pub_joint1_left.publish(joint1_left_command_msg)

                # Publish PID data - could be usefule for tuning
                self.pub_pid_roll.publish(self.pid_roll.create_msg())
                self.pub_pid_roll_rate.publish(self.pid_roll_rate.create_msg())
                self.pub_pid_pitch.publish(self.pid_pitch.create_msg())
                self.pub_pid_pitch_rate.publish(self.pid_pitch_rate.create_msg())
                self.pub_pid_yaw.publish(self.pid_yaw.create_msg())
                self.pub_pid_yaw_rate.publish(self.pid_yaw_rate.create_msg())

    '''
    Calculate new joint increments for right manipulator.

    @param dx_pitch
    @param dy_roll
    @param limit Limit maximum pitch and roll values
    @return Tuple containing (dq1R, dq2R)
    '''
    def get_new_dqR(self, dx_pitch, dy_roll, limit):
        # Add initial offset - right manipulator
        x_right_target = self.x_offset - dx_pitch
        y_right_target = self.y_offset - dy_roll

        #x_right_target = 2*self.x_offset - self.l * (math.cos(self.q1L + self.q2L) + math.cos(self.q1L))
        #y_right_target = - self.l * (math.sin(self.q1L + self.q2L) + math.sin(self.q1L))

        # Get joint offsets of the right manipulator
        q1R = self.q1R
        q2R = self.q2R

        print ''
        print 'q1R: ', q1R, ' q2L: ', q2R

        # Get current position of the right manipulator 
        x_right_curr = self.l * (math.cos(q1R + q2R) + math.cos(q1R))
        y_right_curr = self.l * (math.sin(q1R + q2R) + math.sin(q1R))

        print 'x_right_target: ', x_right_target, ' y_right_target: ', y_right_target
        print 'x_right_curr: ', x_right_curr, ' y_right_curr: ', y_right_curr
        print 'dx: ', - dx_pitch, ' dy: ', - dy_roll

        dx_right_pitch = x_right_target - x_right_curr
        dy_right_roll = y_right_target - y_right_curr

        # Limit dx / dy
        dx_right_pitch = self.saturation(dx_right_pitch, limit)
        dy_right_roll = self.saturation(dy_right_roll, limit)

        print 'dx_new: ', dx_right_pitch, ' dy_new: ', dy_right_roll
        
        # New angle increments (right)
        try:
            dq1R =  math.cos(q1R + q2R) / (self.l * math.sin(q2R)) * (dx_right_pitch) \
               + math.sin(q1R + q2R) / (self.l * math.sin(q2R)) * (dy_right_roll)
            dq2R = - (math.cos(q1R + q2R) + math.cos(q1R)) / (self.l * math.sin(q2R)) * (dx_right_pitch) \
               - (math.sin(q1R + q2R) + math.sin(q1R)) / (self.l * math.sin(q2R)) * (dy_right_roll)
        except ZeroDivisionError:
            print "ZeroDivisionError"
            
            #dq2r - any value
            dq2r = 0.01
            dq1r = dx_right_pitch - math.cos(q2R) / (math.cos(q2R) + 1) * dq2R
        
        print 'dq1R: ', dq1R, ' dq2R: ', dq2R
        print ''

        return (dq1R, dq2R)

    '''
    Calculate new joint increments for the left manipulator.

    @param dx_pitch 
    @param dy_roll
    @return Tuple containing (dq1L, dq2L)
    '''
    def get_new_dqL(self, dx_pitch, dy_roll, limit):
        # Add initial offset - left manipulator
        x_left_target = self.x_offset + dx_pitch
        y_left_target = self.y_offset + dy_roll

        # Get joint offsets of the left manipulator
        q1L = self.q1L
        q2L = self.q2L

        print ''
        print 'q1L: ', q1L, ' q2L: ', q2L

        # Get current (x,y) position of the left manipulator
        x_left_curr = self.l * (math.cos(q1L + q2L) + math.cos(q1L))
        y_left_curr = self.l * (math.sin(q1L + q2L) + math.sin(q1L))

        print 'x_left_target: ', x_left_curr, ' y_left_target: ', y_left_target
        print 'x_left_curr: ', x_left_curr, ' y_left_curr: ', y_left_curr
        print 'dx: ', dx_pitch, ' dy: ', dy_roll

        # Calculate distance to the target position
        dx_left_pitch = x_left_target - x_left_curr
        dy_left_roll = y_left_target - y_left_curr

        # Limit dx / dy
        dx_left_pitch = self.saturation(dx_left_pitch, limit)
        dy_left_roll = self.saturation(dy_left_roll, limit)

        print 'dx_new: ', dx_left_pitch, ' dy_new: ', dy_left_roll

        # New angle increments (left)
        try:
            dq1L = ( math.cos(q1L + q2L) / (self.l * math.sin(q2L)) ) * (dx_left_pitch) + \
                   ( math.sin(q1L + q2L) / (self.l * math.sin(q2L)) ) * (dy_left_roll)
            dq2L = -( math.cos(q1L + q2L) + math.cos(q1L) ) / (self.l * math.sin(q2L)) * (dx_left_pitch) \
                   - ( math.sin(q1L + q2L) + math.sin(q1L) ) / (self.l * math.sin(q2L)) * (dy_left_roll)
        except ZeroDivisionError:
            print "ZeroDivisionError"
            # TODO Handle zero division

            #dq2r - any value
            dq2r = 0.01
            dq1r = dx_right_pitch - math.cos(q2R) / (math.cos(q2R) + 1) * dq2R

        print 'dq1L: ', dq1L, ' dq2L: ', dq2L
        print ''
        
        return (dq1L, dq2L)      

    def saturation(self, x, limit):
        if x > limit:
            return limit

        if x < (- limit):
            return -limit

        return x

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
        
    def joint0_left_cb(self, msg):
        self.q1L = msg.process_value

    def joint1_left_cb(self, msg):
        self.q2L = msg.process_value
        
    def joint0_right_cb(self, msg):
        self.q1R = msg.process_value
        
    def joint1_right_cb(self, msg):
        self.q2R = msg.process_value
        
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
    attitude_ctl = AttitudeControl()
    attitude_ctl.initial_position()
    attitude_ctl.run()


			