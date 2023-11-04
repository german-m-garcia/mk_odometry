#!/usr/bin/env python

"""
mk_odometry: connects to the Auriga Board and reads encoder ticks. Calculates odometry and publishes
it as ROS messages.

"""

import sys
import select
import serial
import threading


import threading
import time
import math
from time import sleep
#from apscheduler.scheduler import Scheduler
from apscheduler.schedulers.background import BackgroundScheduler
import logging

# imports for ROS odometry
import numpy as np
import rospy
import roslib
import tf
import PyKDL as kdl
# Messages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist
# from gamecontrol.msg import Motioncmd
from mk_odometry.msg import Encoder
from sensor_msgs.msg import JointState

from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy

# to register  an exit function
import atexit
import covariances

# to pack floats
import struct


# Mutex
from threading import Thread, Lock


def map(v, in_min, in_max, out_min, out_max):
    # Check that the value is at least in_min
    if v < in_min:
        v = in_min
    # Check that the value is at most in_max
    if v > in_max:
        v = in_max
    return (v - in_min) * (out_max - out_min) // (in_max - in_min) + out_min


def joystickToDiff(x, y, minJoystick, maxJoystick, minSpeed, maxSpeed):
    # If x and y are 0, then there is not much to calculate...
    if x == 0 and y == 0:
        return (0, 0)

    # First Compute the angle in deg
    # First hypotenuse
    z = math.sqrt(x * x + y * y)

    # angle in radians
    rad = math.acos(math.fabs(x) / z)

    # and in degrees
    angle = rad * 180 / math.pi

    # Now angle indicates the measure of turn
    # Along a straight line, with an angle o, the turn co-efficient is same
    # this applies for angles between 0-90, with angle 0 the coeff is -1
    # with angle 45, the co-efficient is 0 and with angle 90, it is 1

    tcoeff = -1 + (angle / 90) * 2
    turn = tcoeff * math.fabs(math.fabs(y) - math.fabs(x))
    turn = round(turn * 100, 0) / 100

    # And max of y or x is the movement
    mov = max(math.fabs(y), math.fabs(x))

    # First and third quadrant
    if (x >= 0 and y >= 0) or (x < 0 and y < 0):
        rawLeft = mov
        rawRight = turn
    else:
        rawRight = mov
        rawLeft = turn

    # Reverse polarity
    if y < 0:
        rawLeft = 0 - rawLeft
        rawRight = 0 - rawRight

    # minJoystick, maxJoystick, minSpeed, maxSpeed
    # Map the values onto the defined rang
    rightOut = map(-rawRight, minJoystick, maxJoystick, minSpeed, maxSpeed)
    leftOut = map(rawLeft, minJoystick, maxJoystick, minSpeed, maxSpeed)

    # return (rightOut, leftOut)
    return (leftOut, rightOut)


class RosOdomPublisher:
    def __init__(self):
        rospy.init_node('odometryPublisher', anonymous=True)

        self.odom_pub = rospy.Publisher(
            'odom', Odometry, queue_size=10)
        #self.imu_pub = rospy.Publisher('/makeblock/imu', Imu, queue_size=10)
        self.imu_pub = rospy.Publisher('/imu', Imu, queue_size=10)
        self.lwheel_pub = rospy.Publisher(
            '/makeblock/lwheel', Encoder, queue_size=10)
        self.rwheel_pub = rospy.Publisher(
            '/makeblock/rwheel', Encoder, queue_size=10)
        self.tf_br = tf.TransformBroadcaster()

        self.publish_odom_tf = rospy.get_param('~publish_odom_tf', False)

        self.frame_id = 'odom'
        self.child_frame_id = 'base_link'

        self.vx = self.vy = self.x = self.y = self.z = 0
        self.w = self.theta = 0

        self.encoder_msg_l = Encoder()
        self.encoder_msg_r = Encoder()
        self.encoder_msg_l.header.frame_id = self.child_frame_id 
        self.encoder_msg_r.header.frame_id = self.child_frame_id 

        self.imu_msg = Imu()
        self.imu_frame_id = "imu"

        # Joint States
        self.joint_state_publisher = rospy.Publisher(
            '/joint_states', JointState)
        self.jsmsg = JointState()

        self.jsmsg.name = ['base_to_lwheel', 'base_to_rwheel']
        self.jsmsg.position = [0, 0]
        self.jsmsg.velocity = [0, 0]
        self.jsmsg.effort = [0, 0]

        print('---RosOdomPublisher---')
        print(rospy.resolve_name('~publish_odom_tf'), self.publish_odom_tf)

    def publish_joint_states(self, rwheelPos, lwheelPos):

        # convert abs ticks to radians
        rRads = rwheelPos * math.pi / 180.
        lRads = lwheelPos * math.pi / 180.

        self.jsmsg.position[0] = lRads
        self.jsmsg.velocity[0] = 0
        self.jsmsg.effort[0] = 0

        self.jsmsg.position[1] = rRads
        self.jsmsg.velocity[1] = 0
        self.jsmsg.effort[1] = 0
        # print('publishing  lRads, rRads =', self.jsmsg.position[0], self.jsmsg.position[1], ' self.jsmsg.name=', self.jsmsg.name )

        self.jsmsg.header.stamp = rospy.Time.now()
        self.jsmsg.header.frame_id = 'base_link'
        self.joint_state_publisher.publish(self.jsmsg)

    def publish_wheel_encoders(self, rwheel, lwheel):

        self.encoder_msg_l.ticks = lwheel
        self.encoder_msg_r.ticks = rwheel
        self.encoder_msg_r.header.stamp = rospy.Time.now()
        self.encoder_msg_l.header.stamp = self.encoder_msg_r.header.stamp
        
        self.lwheel_pub.publish(self.encoder_msg_l)
        self.rwheel_pub.publish(self.encoder_msg_r)
        # print("encoders: ", lwheel, rwheel)

    # Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
    def publish_imu(self, rotX, rotY, rotZ, accX, accY, accZ):
        self.imu_msg.header.stamp = rospy.Time.now()
        self.imu_msg.header.frame_id = self.imu_frame_id  # i.e. '/base_link'


        #imu_msg.orientation = Quaternion(*(kdl.Rotation.RPY(roll, pitch, yaw).GetQuaternion()))
        self.imu_msg.orientation_covariance = covariances.ORIENTATION_COV_IMU

        #print 'imu_msg.orientation_covariance'
        # conversions
        gravity = 9.8
        degreesToRad = 0.01745329252

        self.imu_msg.angular_velocity.x = rotX * degreesToRad
        self.imu_msg.angular_velocity.y = rotY * degreesToRad
        self.imu_msg.angular_velocity.z = rotZ * degreesToRad

        
        
        self.imu_msg.angular_velocity_covariance = covariances.ANGULAR_VEL_COV_IMU


        self.imu_msg.linear_acceleration.x = accX * gravity
        self.imu_msg.linear_acceleration.y = accY * gravity
        self.imu_msg.linear_acceleration.z = accZ * gravity
        self.imu_msg.linear_acceleration_covariance = covariances.ACCELERATION_COV_IMU

        # print ('publishing imu ', rotX, rotY, rotZ, accX, accY, accZ)

        self.imu_pub.publish(self.imu_msg)

    def publish_odom(self, x, y, theta, vx, vy, w):
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id  # i.e. '/odom'
        msg.child_frame_id = self.child_frame_id  # i.e. '/base_footprint'
        self.x = x
        self.y = y
        self.theta = theta

        self.vx = vx
        self.vy = vy
        self.w = w

        # pose part
        msg.pose.pose.position = Point(self.x, self.y, self.z)
        msg.pose.pose.orientation = Quaternion(
            *(kdl.Rotation.RPY(0, 0, self.theta).GetQuaternion()))

        # tuple(p_cov.ravel().tolist())
        msg.pose.covariance = covariances.ODOM_COV_WHEEL

        pos = (msg.pose.pose.position.x,
               msg.pose.pose.position.y,
               msg.pose.pose.position.z)

        ori = (msg.pose.pose.orientation.x,
               msg.pose.pose.orientation.y,
               msg.pose.pose.orientation.z,
               msg.pose.pose.orientation.w)

        # twist part
        msg.twist.twist.linear.x = self.vx
        msg.twist.twist.linear.y = self.vy
        msg.twist.twist.angular.z = self.w
        msg.twist.covariance = covariances.TWIST_COV_WHEEL

        

        # Publish odometry message
        self.odom_pub.publish(msg)

        # Also publish tf if necessary
        if self.publish_odom_tf:
            self.tf_br.sendTransform(
                pos, ori, msg.header.stamp, msg.child_frame_id, msg.header.frame_id)



"""
encoder and raw odometry classes
"""
exitFlag = 0


def sign(a): return (a > 0) - (a < 0)

"""
reads the encoder values periocally and publishes the estimated odometry
"""


class odometryThread(threading.Thread):
    def __init__(self, threadID, name, encoder, rosOdomPublisher, mutex, ticks_per_meter, axle ):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.sched = BackgroundScheduler()
        self.sched.start()        # start the scheduler
        self.encoder = encoder
        self.last_encoder_right = 0
        self.last_encoder_left = 0
        self.ticks_right = 0
        self.ticks_left = 0
        self.acc_ticks_right = 0
        self.acc_ticks_left = 0
        self.lower_bound = 0
        self.upper_bound = 0
        self.encoder_min = -32768
        self.encoder_max = 32768
        self.encoder_pos_right = 0
        self.encoder_pos_left = 0
        self.seconds = 0.05  # 0.03 time interval in which odometry is checked
        self.mutex = mutex

        self.doPublishOdom = rospy.get_param('~publish_odom', False)
        self.doPublishOdomOnlyVelocity = rospy.get_param(
            '~publish_odom_only_velocity', True)
        self.doPublishImu = rospy.get_param('~publish_imu', True)
        self.doPublishEncoders = rospy.get_param('~publish_encoders', True)
        self.doPublishJointStates = rospy.get_param(
            '~publish_joint_states', True)

        # tf broadcaster
        self.base_link = "base_link"
        self.odometry_frame = "odom"
        self.br = tf.TransformBroadcaster()

        logging.basicConfig()

        # encoder overflow values
        self.lower_bound = (self.encoder_max -
                            self.encoder_min) * 0.2 + self.encoder_min
        self.upper_bound = (self.encoder_max -
                            self.encoder_min) * 0.8 + self.encoder_min

        # wheel odometry: pose=(x,y,theta)        
        self.m_tic = 1. / ticks_per_meter
        self.b = axle
        self.x = 0.
        self.y = 0.
        self.theta = 0.
        self.last_theta = 0.
        self.last_ticks_right_odometry = 0
        self.last_ticks_left_odometry = 0

        self.last_timestamp = rospy.get_rostime()

        # wheel velocities
        self.vx = 0.
        self.vy = 0.
        self.w = 0.
        self.rosOdometry = rosOdomPublisher

        # imu
        self.roll = 0.
        self.pitch = 0.
        self.yaw = 0.

        # imu velocities
        self.imu_vroll = 0.
        self.imu_vpitch = 0.
        self.imu_vyaw = 0.
        self.imu_last_roll = 0.
        self.imu_last_pitch = 0.
        self.imu_last_yaw = 0.

        print('---odometryThread---')
        print(rospy.resolve_name('~publish_odom'), self.doPublishOdom)
        print(rospy.resolve_name('~publish_imu'), self.doPublishImu)
        print(rospy.resolve_name('~publish_encoders'), self.doPublishEncoders)
        print(rospy.resolve_name('~publish_joint_states'), self.doPublishJointStates)
        job = self.sched.add_job( self.updateTicks, 'interval', seconds=self.seconds, args=[])

    def updateTicks(self):

        self.mutex.acquire()
        cur_encoder_right = -self.encoder.getEnc2()  # -self.encoder.getEnc1()
        cur_encoder_left = self.encoder.getEnc1()
        self.encoder_pos_right = self.encoder.getAbsPos2()
        self.encoder_pos_left = self.encoder.getAbsPos1()
        # print('self.encoder_pos_right= ', self.encoder_pos_right, " self.encoder_pos_left=", self.encoder_pos_left)
        self.mutex.release()
        now = rospy.get_rostime()
        diff = now - self.last_timestamp
        self.time_interval = diff.to_sec()

        #print 'now time in secs=',  now
        #print 'diff time = ', diff, ' in secs=', diff.to_sec()
        
        self.last_timestamp = now

        self.mutex.acquire()
        self.roll = self.encoder.getRotX()
        self.pitch = self.encoder.getRotY()
        self.yaw = self.encoder.getRotZ()
        self.imu_vroll = self.encoder.getAccX()
        self.imu_vpitch = self.encoder.getAccY()
        self.imu_vyaw = self.encoder.getAccZ()

        self.mutex.release()

        # overflow left wheel
        if cur_encoder_right < self.lower_bound and self.last_encoder_right > self.upper_bound:
            self.acc_ticks_right = self.acc_ticks_right + 1
        # underflow
        if cur_encoder_right > self.upper_bound and self.last_encoder_right < self.lower_bound:
            self.acc_ticks_right = self.acc_ticks_right - 1

        # overflow right wheel
        if cur_encoder_left < self.lower_bound and self.last_encoder_left > self.upper_bound:
            self.acc_ticks_left = self.acc_ticks_left + 1
        # underflow
        if cur_encoder_left > self.upper_bound and self.last_encoder_left < self.lower_bound:
            self.acc_ticks_left = self.acc_ticks_left - 1

        self.ticks_right = (
            cur_encoder_right + self.acc_ticks_right * (self.encoder_max - self.encoder_min))
        self.last_encoder_right = cur_encoder_right
        self.ticks_left = (cur_encoder_left + self.acc_ticks_left *
                           (self.encoder_max - self.encoder_min))
        self.last_encoder_left = cur_encoder_left
        self.updateOdometry()
        
        if self.doPublishOdom:

            if self.doPublishOdomOnlyVelocity:
                self.rosOdometry.publish_odom(
                    0, 0, 0, self.vx, self.vy, self.w)

            else:
                self.rosOdometry.publish_odom(
                    self.x, self.y, self.theta, self.vx, self.vy, self.w)

            # self.br.sendTransform((self.x, self.y, 0),
            #                      tf.transformations.quaternion_from_euler(
            #                          0, 0, self.theta),
            #                      rospy.Time.now(),
            #                      self.base_link,
            #                      self.odometry_frame)

        if self.doPublishImu:
            self.rosOdometry.publish_imu(
                self.roll, self.pitch, self.yaw, self.imu_vroll, self.imu_vpitch, self.imu_vyaw)

        if self.doPublishEncoders:
            self.rosOdometry.publish_wheel_encoders(
                self.ticks_right, self.ticks_left)

        if self.doPublishJointStates:
            self.rosOdometry.publish_joint_states(
                self.encoder_pos_right, self.encoder_pos_left)

    def rotating(self):
        if self.ticks_right > 0 and self.ticks_left > 0:
            return True
        if self.ticks_right < 0 and self.ticks_left < 0:
            return True
        return False

    def updateOdometry(self):

        #print 'self.ticks_right=', self.ticks_right
        #print 'self.last_ticks_right_odometry=', self.last_ticks_right_odometry

        # right encoder goes negative when moving forward
        delta_Ur = -self.m_tic * \
            (self.ticks_right - self.last_ticks_right_odometry)
        delta_Ul = self.m_tic * \
            (self.ticks_left - self.last_ticks_left_odometry)
        #print 'self.ticks_right=', self.ticks_right
        #print 'self.ticks_left=', self.ticks_left
        self.last_ticks_right_odometry = self.ticks_right
        self.last_ticks_left_odometry = self.ticks_left
        delta_Ui = (delta_Ul+delta_Ur)/2.
        delta_theta = (delta_Ur-delta_Ul)/self.b

        #print 'delta_ur', delta_Ur


        self.x = self.x + delta_Ui * math.cos(self.theta)
        self.y = self.y + delta_Ui * math.sin(self.theta)

        self.theta = self.theta + delta_theta

        # update the velocities
        self.vx = delta_Ui / self.time_interval # self.seconds
        self.vy = 0.
        self.w = (self.theta - self.last_theta) / self.time_interval #self.seconds
        self.last_theta = self.theta

        # imu
        # self.imu_vroll = (self.roll - self.imu_last_roll) / self.seconds
        # self.imu_last_roll = self.roll

        # self.imu_vpitch = (self.pitch - self.imu_last_pitch) / self.seconds
        # self.imu_last_pitch = self.pitch

        # self.imu_vyaw = (self.yaw - self.imu_last_yaw) / self.seconds
        # self.imu_last_yaw = self.yaw

    # def run(self):
        
        #job = self.sched.add_job(
        #    self.updateTicks, 'interval', seconds=self.seconds, args=[])


class encoderThread (threading.Thread):
    def __init__(self, threadID, name, serial, mutex):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.serial = serial
        # encoders
        self.encoder1 = 0
        self.encoder2 = 0
        self.speed1 = 0
        self.speed2 = 0
        self.encoderPos1 = 0
        self.encoderPos2 = 0
        self.encoderPosMax = 360
        # IMU
        self.rotX = 0.
        self.rotY = 0.
        self.rotZ = 0.
        self.accX = 0.
        self.accY = 0.
        self.accZ = 0.

        self.publish_encoders = True
        self.mutex = mutex

    def __del__(self):
        print('shutting down serial port')
        self.serial.close()

    def getEnc2(self):
        return self.encoder2

    def getEnc1(self):
        return self.encoder1

    def getSpeed2(self):
        return self.speed2

    def getSpeed1(self):
        return self.speed1

    def getAbsPos1(self):
        return self.encoderPos1

    def getAbsPos2(self):
        return self.encoderPos2

    def getRotX(self):
        return self.rotX

    def getRotY(self):
        return self.rotY

    def getRotZ(self):
        return self.rotZ

    def getAccX(self):
        return self.accX

    def getAccY(self):
        return self.accY

    def getAccZ(self):
        return self.accZ

    # returns encoder value read from serial
    def readEncoder(self):
        n = ser.inWaiting()
        while n < 5:
            n = ser.inWaiting()
        typedata = ser.read(1)
        data = ser.read(4)
        return struct.unpack('i', data)[0]

    def readAngle(self):
        n = ser.inWaiting()
        while n < 5:
            n = ser.inWaiting()
        typedata = ser.read(1)
        angle = ser.read(4)
        return struct.unpack('f', angle)[0]

    def readVelocity(self):
        n = ser.inWaiting()
        while n < 5:
            n = ser.inWaiting()
        typedata = ser.read(1)
        angle = ser.read(4)
        return struct.unpack('f', angle)[0]

    def run(self):
        while not rospy.is_shutdown():

            n = ser.inWaiting()
            # print('encoder thread, ', n, ' bytes available')
            while n < 10:
                n = ser.inWaiting()

            # print n, ' available bytes '
            seq1 = ser.read(1)
            seq2= ser.read(1)
            # print( 'seq1=', seq1, ' seq2=', seq2)
            while seq1 != b'T' and seq2 != b'X':
                
                n = ser.inWaiting()
                while n < 2:
                    n = ser.inWaiting()

                seq1 = ser.read(1)
                seq2 = ser.read(1)
                
                print( 'Skipping bytes... seq1=', seq1, ' seq2=', seq2,' bytes available=', n)
            
            self.mutex.acquire()
            self.encoder1 = self.readEncoder()
            self.encoder2 = self.readEncoder()
            self.mutex.release()
            # print ('right (encoder1)',self.encoder1 , 'left(encoder2) ', self.encoder2)

            n = ser.inWaiting()
            while n < 10:
                n = ser.inWaiting()

            # print n, ' available bytes '
            self.mutex.acquire()

            self.speed1 = self.readVelocity()
            self.speed2 = self.readVelocity()
            self.mutex.release()
            self.encoderPos1 = self.encoder1 % self.encoderPosMax
            self.encoderPos2 = self.encoder2 % self.encoderPosMax

            # print 'speeds=', self.speed1, self.speed2

            n = ser.inWaiting()
            while n < 15:
                n = ser.inWaiting()

            self.mutex.acquire()
            self.rotX = self.readAngle()
            self.rotY = self.readAngle()
            self.rotZ = self.readAngle()
            self.mutex.release()
            # print 'rotations ', self.rotX, self.rotY, self.rotZ

            n = ser.inWaiting()
            while n < 15:
                n = ser.inWaiting()

            self.mutex.acquire()
            self.accX = self.readAngle()
            self.accY = self.readAngle()
            self.accZ = self.readAngle()
            self.mutex.release()
            # print ('accelerations x=', self.accX, ' y=', self.accY, ' z=',self.accZ)

            # print 'abs pos | right=', self.encoderPos1, 'left=', self.encoderPos2

            # sleep(0.02)


class ReadControls(threading.Thread):

    def __init__(self, threadID, name, axle, m_tics, radius):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.sched = BackgroundScheduler()
        self.sched.start()        # start the scheduler
        self.x = 0
        self.y = 0
        self.max_speed = rospy.get_param('~velocity', 100)
        self.hard_coded_max_speed = 150
        self.subscriber = rospy.Subscriber("/joy", Joy, self.callback_joy)
        self.twist_subscriber = rospy.Subscriber(
            "/cmd_vel", Twist, self.callback_twist)
        self.last_received = rospy.Time.now()
        self.mutex = Lock()
        # check whether no command has been received
        job = self.sched.add_job(
            self.check_commands, 'interval', seconds=0.2, args=[])
        #job2 = self.sched.add_job(self.sendMotionCommand, seconds = 1.5, args=[])

        # axle
        self.b = axle
        self.m_tics = m_tics  # tics per meter
        self.radius = radius
        # distance travelled in one revolution
        self.m_pro_rev = 2 * math.pi * self.radius
        self.linear_vel_to_rpm = (
            1. / self.m_pro_rev) * 60.  # transforms linear wheel speeds to the controller interface in rpm
        self.vr = 0.  # right wheel vel in m/s
        self.vl = 0.  # left wheel vel in m/s

        print('--- ReadControls ---')
        print(rospy.resolve_name('~velocity'), self.max_speed)

    def check_commands(self):
        now = rospy.Time.now()
        if now.to_sec() - self.last_received.to_sec() > 0.200:
            self.mutex.acquire()
            self.x = 0
            self.y = 0
            self.vl = 0
            self.vr = 0
            self.mutex.release()

    def twist_to_velocities(self, twist):
        
        vx = twist.linear.x
        vtheta = twist.angular.z

        # turning
        if vx == 0.:
            vr = vtheta * self.b / 2.
            vl = -vr
        elif vtheta == 0.:
            vl = vx
            vr = vx
        else:
            vl = vx - vtheta * self.b / 2.
            vr = vx + vtheta * self.b / 2.

        return (vl * self.linear_vel_to_rpm, vr * self.linear_vel_to_rpm)

    def callback_twist(self, msg):

        self.last_received = rospy.Time.now()
        (vl, vr) = self.twist_to_velocities(msg)
        self.mutex.acquire()
        self.vl = vl
        self.vr = vr
        self.mutex.release()

    def callback_joy(self, msg):

        #self.last_received = rospy.Time.now()
        #print msg.axes
        #(rightOut, leftOut) = joystickToDiff(msg.axes[1], msg.axes[0], -1, 1, -self.max_speed, self.max_speed)
        #print 'rightMotor, leftMotor=', rightOut, leftOut
        speedDown = msg.buttons[4]
        speedUp = msg.buttons[5]
        if speedUp == 1:
            if self.max_speed < self.hard_coded_max_speed:
                self.max_speed = self.max_speed + 1
                print ('speed Up: ', self.max_speed)

        elif speedDown == 1:
            if self.max_speed > 0:
                self.max_speed = self.max_speed - 1
                print ('speed Down: ', self.max_speed)

    def get_wheel_rpms(self):
        return (self.vl, self.vr)

    def getCommand(self):
        return (self.y, self.x)

    # returns encoder value read from serial

    def readEncoder(self):
        typedata = ser.read(1)
        n = ser.inWaiting()
        while n < 4:
            n = ser.inWaiting()
        data = ser.read(4)
        return struct.unpack('i', data)[0]

    def readAngle(self):
        typedata = ser.read(1)
        n = ser.inWaiting()
        while n < 4:
            n = ser.inWaiting()
        angle = ser.read(4)
        return struct.unpack('f', angle)[0]

    def run(self):
        print ('letting the IMU autocalibrate... No not move the robot!')
        sleep(4)
        print ('ReadControls Thread: starting sending motion commands!')
        while not rospy.is_shutdown():
            self.mutex.acquire()
            (vl, vr) = self.get_wheel_rpms()
            # print 'sending', vl, vr
            self.mutex.release()
            motor1 = struct.pack('f', vl)
            motor2 = struct.pack('f', vr)

            ser.write(str.encode('SX'))
            ser.write(motor1)
            ser.write(motor2)
            sleep(0.03)

    def run_joy(self):
        while not rospy.is_shutdown():
            self.mutex.acquire()
            cmd = self.getCommand()
            #cmd = (+125, +125)
            self.mutex.release()
            motor1 = struct.pack('h', cmd[1])
            motor2 = struct.pack('h', cmd[0])

            ser.write(str.encode('SX'))
            ser.write(motor1)
            ser.write(motor2)
            sleep(0.03)


if __name__ == '__main__':

    mutex = Lock()
    ser = serial.Serial('/dev/arduino', 115200, timeout=10)  # open serial port
    print(ser.name)
    sleep(2)

    # Robot parameters
    axle = rospy.get_param(
            '~wheel_separation', 0.17)  # wheel separation
    ticks_per_meter = rospy.get_param(
            '~ticks_per_meter', 2800)
    m_tic = 1. / ticks_per_meter   # meters per tic
    radius = rospy.get_param(
            '~radius', 0.02)

    # read the version data
    version = 20
    read = 0
    n = ser.inWaiting()
    while read < version:
        n = ser.inWaiting()
        if n > 0:
            # print  n,' bytes available'
            ser.read(n)
            read = read + n
    print ("version # read\n")

    # create the ROS odometry publisher
    odometryPublisher = RosOdomPublisher()

    # reads and stores encoder + IMU data
    threadEncoder = encoderThread(1, "Thread-encoder", ser, mutex)
    # computes odometry data from the data stored by the encoderThread
    threadOdom = odometryThread(
        2, "Thread-odometry", threadEncoder, odometryPublisher, mutex, ticks_per_meter, axle )

    # subscribes to /cmd_vel commands
    readControls = ReadControls(3, 'Thread-readcontrols', axle, m_tic, radius)

    # start the threads
    threadEncoder.start()
    threadOdom.start()
    readControls.start()

    start_time = time.time()
    elapsed_time = time.time() - start_time

    try:
        while not rospy.is_shutdown():
            pass

            #n = ser.inWaiting()

            # if n > 0:
            #	buffer = ser.read(n)
            #	print 'read this many bytes: ', n
            #	print buffer

            sleep(0.1)
    except rospy.ROSInterruptException:
        print('exiting')
        pass
