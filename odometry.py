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
from apscheduler.scheduler import Scheduler
import logging

#imports for ROS odometry
import numpy as np
import rospy
import roslib
import tf
import PyKDL as kdl
# Messages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from gamecontrol.msg import Motioncmd
from mk_odometry.msg import Encoder

from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy

#to register  an exit function
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

	#return (rightOut, leftOut)
	return (leftOut,rightOut)


class RosOdomPublisher:
	def __init__(self):
		rospy.init_node('odometryPublisher', anonymous=True)
		
		self.odom_pub = rospy.Publisher('/makeblock/odom', Odometry, queue_size=10)
		self.imu_pub = rospy.Publisher('/makeblock/imu', Imu, queue_size=10)
		self.lwheel_pub = rospy.Publisher('/makeblock/lwheel', Encoder, queue_size=10)
		self.rwheel_pub = rospy.Publisher('/makeblock/rwheel', Encoder, queue_size=10)
		self.tf_br = tf.TransformBroadcaster()

		self.publish_odom_tf = False

		self.frame_id = 'odom'
		self.child_frame_id = 'base_link'
		
		self.vx = self.vy = self.x = self.y = self.z = 0
		self.w = self.theta = 0

		self.encoder_msg_l = Encoder()
		self.encoder_msg_r = Encoder()
		
	def publish_wheel_encoders(self, rwheel, lwheel):

		self.encoder_msg_l.ticks = lwheel
		self.encoder_msg_r.ticks = rwheel
		
		self.encoder_msg_l.header.stamp = rospy.Time.now()
		self.encoder_msg_r.header.stamp = self.encoder_msg_l.header.stamp
		self.encoder_msg_l.header.frame_id = 'lwheel'
		self.encoder_msg_r.header.frame_id = 'rwheel'

		self.lwheel_pub.publish(self.encoder_msg_l)
		self.rwheel_pub.publish(self.encoder_msg_r)

	def publish_imu(self, roll, pitch, yaw, imu_x, imu_y, imu_w):
		imu_msg = Imu()
		imu_msg.header.stamp = rospy.Time.now()
		imu_msg.header.frame_id = self.child_frame_id # i.e. '/base_link'


		imu_msg.orientation = Quaternion(*(kdl.Rotation.RPY(roll, pitch, yaw).GetQuaternion()))
		imu_msg.orientation_covariance = covariances.ODOM_COV_IMU
		
		imu_msg.angular_velocity.x = imu_x
		imu_msg.angular_velocity.y = imu_y
		imu_msg.angular_velocity.z = imu_w

		imu_msg.angular_velocity_covariance = covariances.TWIST_COV_IMU		
		self.imu_pub.publish(imu_msg)
		
	
	def publish_odom(self,x,y,theta, vx, vy, w):

		msg = Odometry()
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = self.frame_id # i.e. '/odom'
		msg.child_frame_id = self.child_frame_id # i.e. '/base_footprint'
		self.x = x
		self.y = y
		self.theta = theta

		self.vx = vx
		self.vy = vy
		self.w = w

		#pose part
		msg.pose.pose.position = Point(self.x, self.y, self.z)
		msg.pose.pose.orientation = Quaternion(*(kdl.Rotation.RPY(0, 0, self.theta).GetQuaternion()))

		msg.pose.covariance = covariances.ODOM_COV_WHEEL #tuple(p_cov.ravel().tolist())

		pos = (msg.pose.pose.position.x,
			 msg.pose.pose.position.y,
			 msg.pose.pose.position.z)

		ori = (msg.pose.pose.orientation.x,
			 msg.pose.pose.orientation.y,
			 msg.pose.pose.orientation.z,
			 msg.pose.pose.orientation.w)

		#twist part
		msg.twist.twist.linear.x = self.vx
		msg.twist.twist.linear.y = self.vy
		msg.twist.twist.angular.z  = self.w
		msg.twist.covariance = covariances.TWIST_COV_WHEEL
		
		# Publish odometry message
		self.odom_pub.publish(msg)


		# Also publish tf if necessary
		if self.publish_odom_tf:
		  self.tf_br.sendTransform(pos, ori, msg.header.stamp, msg.child_frame_id, msg.header.frame_id)


"""
encoder and raw odometry classes
"""
exitFlag = 0

sign = lambda a: (a>0) - (a<0)

"""
reads the encoder values periocally and publishes the estimated odometry
"""
class odometryThread(threading.Thread):
	def __init__(self,threadID, name,encoder,rosOdomPublisher, mutex):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.name = name
		self.sched = Scheduler()
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
		self.seconds = 0.03  # time interval in which odometry is checked
		self.mutex = mutex

		self.doPublishOdom = rospy.get_param('publish_odom', True)
		self.doPublishImu = rospy.get_param('publish_imu', True)
		self.doPublishEncoders = rospy.get_param('publish_encoders', True)

		logging.basicConfig()

		# encoder overflow values
		self.lower_bound = (self.encoder_max - self.encoder_min) * 0.2 + self.encoder_min
		self.upper_bound = (self.encoder_max - self.encoder_min ) * 0.8 + self.encoder_min

		
		# wheel odometry: pose=(x,y,theta)
		self.m_tic = 0.00038 # empirical value meters per tic | 2pi*0.04 / 255 =  0.00098559769
		self.b = 0.174 # wheels separation; should be read from urdf or config file
		self.x = 0.
		self.y = 0.
		self.theta = 0.
		self.last_theta = 0.
			
		
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
	

	def updateTicks(self):
		
		#update the yaw directly from the encoder
		self.mutex.acquire()
		self.roll = self.encoder.getRoll() * 2.*math.pi/360.
		self.pitch = self.encoder.getPitch()*2.*math.pi/360.
		self.yaw = -self.encoder.getYaw()*2.*math.pi/360.
		self.mutex.release()

		self.mutex.acquire()
		cur_encoder_right = self.encoder.getEnc1()
		cur_encoder_left = self.encoder.getEnc2()
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

		self.ticks_right = (cur_encoder_right + self.acc_ticks_right * (self.encoder_max - self.encoder_min) )
		self.last_encoder_right = cur_encoder_right
		self.ticks_left = (cur_encoder_left + self.acc_ticks_left * (self.encoder_max - self.encoder_min) )
		self.last_encoder_left = cur_encoder_left
		
		self.updateOdometry()

		if self.doPublishOdom:
			self.rosOdometry.publish_odom(self.x,self.y,self.theta, self.vx, self.vy, self.w)
		if self.doPublishImu:
			self.rosOdometry.publish_imu(self.roll, self.pitch, self.yaw, self.imu_vroll, self.imu_vpitch, self.imu_vyaw)
		if self.doPublishEncoders:
			self.rosOdometry.publish_wheel_encoders(self.ticks_right, self.ticks_left)
		
	def rotating(self):
		if self.ticks_right > 0 and self.ticks_left> 0:
			return True
		if self.ticks_right < 0 and self.ticks_left < 0:
			return True
		return False
		
	
	def updateOdometry(self):
		
		delta_Ur = self.m_tic * self.ticks_right
		delta_Ul = self.m_tic * self.ticks_left
		delta_Ui = (delta_Ul+delta_Ur)/2.
		delta_theta = (delta_Ur-delta_Ul)/self.b


		self.x = self.x + delta_Ui * math.cos(self.theta)
		self.y = self.y + delta_Ui * math.sin(self.theta)


		self.theta = self.theta + delta_theta
		
			
		# update the velocities
		self.vx = delta_Ui / self.seconds
		self.vy = 0.
		self.w = (self.theta - self.last_theta) / self.seconds
		self.last_theta = self.theta
		
		# imu
		self.imu_vroll = (self.roll - self.imu_last_roll) / self.seconds
		self.imu_last_roll = self.roll
		
		self.imu_vpitch = (self.pitch - self.imu_last_pitch) / self.seconds
		self.imu_last_pitch = self.pitch
		
		self.imu_vyaw = (self.yaw - self.imu_last_yaw) / self.seconds
		self.imu_last_yaw = self.yaw
		
	
	def run(self):
		#job = self.sched.add_interval_job(self.updateTicks, seconds=0.04, args=[])
		job = self.sched.add_interval_job(self.updateTicks, seconds = self.seconds, args=[])


class encoderThread (threading.Thread):
	def __init__(self, threadID, name, serial, mutex):
		threading.Thread.__init__(self)
		self.threadID =	threadID
		self.name = name
		self.serial = serial
		self.encoder1 = 0
		self.encoder2 = 0
		self.speed1 = 0
		self.speed2 = 0
		self.roll = 0.
		self.pitch = 0.
		self.yaw = 0.
		self.last_yaw = 0.
		self.publish_encoders = True
		self.mutex = mutex

	def __del__(self):
		print('shutting down serial port')
		self.serial.close()

	def getEnc2(self):
		return self.encoder2

	def getEnc1(self):
		return self.encoder1

	def getAngles(self):
		return (self.roll, self.pitch, self.yaw)
	
	def getRoll(self):
		return self.roll

	def getPitch(self):
		return self.pitch

	def getYaw(self):
		return self.yaw


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

	def readVelocity(self):
		typedata = ser.read(1)
		n = ser.inWaiting()
		while n < 4:
			n = ser.inWaiting()
		angle = ser.read(4)
		return struct.unpack('f', angle)[0]


	def run(self):
		while not rospy.is_shutdown():

			n = ser.inWaiting()
			
			if n > 0:

				self.mutex.acquire()
				self.encoder1 = self.readEncoder()
				self.encoder2 = self.readEncoder()
				self.mutex.release()

				self.mutex.acquire()
				self.speed1 = self.readVelocity()
				self.speed2 = self.readVelocity()
				self.mutex.release()

				self.mutex.acquire()
				self.roll = self.readAngle()
				self.pitch = self.readAngle()
				cur_yaw = self.readAngle()
				self.mutex.release()
				update_yaw = (cur_yaw - self.last_yaw)
				
				if abs(update_yaw) < 0.04:
					update_yaw = 0.0
				self.yaw = self.yaw + update_yaw
				self.last_yaw = cur_yaw

				#print self.encoder1, self.encoder2
				#print 'speeds=', self.speed1, self.speed2
				

			#sleep(0.01)

class ReadControls(threading.Thread):

	def __init__(self, threadID, name):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.name = name
		self.sched = Scheduler()
		self.sched.start()        # start the scheduler
		self.x = 0
		self.y = 0
		self.max_speed = 180
		self.subscriber = rospy.Subscriber("/joy", Joy, self.callback)
		self.last_received = rospy.Time.now()
		self.mutex = Lock()
		#check whether no command has been received
		job = self.sched.add_interval_job(self.check_commands, seconds=0.2, args=[])
		#job2 = self.sched.add_interval_job(self.sendMotionCommand, seconds = 1.5, args=[])

	
	def check_commands(self):
		now = rospy.Time.now()
		if now - self.last_received > 100:
			self.mutex.acquire()
			self.x = 0
			self.y = 0
			self.mutex.release()


	def callback(self,msg):
		
		self.last_received = rospy.Time.now()
		#print msg.axes
		(rightOut, leftOut) = joystickToDiff(msg.axes[1], msg.axes[0], -1, 1, -self.max_speed, self.max_speed)
		#print 'rightMotor, leftMotor=', rightOut, leftOut
		self.mutex.acquire()
		self.x = int(leftOut)
		self.y = int(rightOut)
		self.mutex.release()

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
		while not rospy.is_shutdown():
			self.mutex.acquire()
			cmd = self.getCommand()
			#cmd = (+125, +125)
			self.mutex.release()
			motor1 = struct.pack('h', cmd[1])
			motor2 = struct.pack('h', cmd[0])

			ser.write('SX')
			ser.write( motor1 )
			ser.write( motor2 )
			sleep(0.03)




if __name__ == '__main__':
	
	mutex = Lock()
	ser = serial.Serial('/dev/arduino',115200,timeout=10) # open serial port	
	print(ser.name)
	sleep(2)
	
	#read the version data
	version = 20
	read = 0
	n = ser.inWaiting()
	while read < version:
		n = ser.inWaiting()
		if n > 0:
			print  n,' bytes available'
			ser.read(n)
			read = read + n
	print "version # read\n"
	
	#create the ROS odometry publisher
	odometryPublisher = RosOdomPublisher()
	
	# reads and stores encoder + IMU data
	threadEncoder = encoderThread(1, "Thread-encoder", ser, mutex)
	# computes odometry data from the data stored by the encoderThread
	threadOdom = odometryThread(2,"Thread-odometry", threadEncoder,odometryPublisher, mutex)
	
	# subscribes to /joy commands
	readControls = ReadControls(3, 'Thread-readcontrols')
		
	#start the threads
	threadEncoder.start()
	threadOdom.start()
	readControls.start()
	

	start_time = time.time()
	elapsed_time = time.time() - start_time
	

	try:
		while not rospy.is_shutdown():
			pass
			
			
			#n = ser.inWaiting()
			
			#if n > 0:
		   	#	buffer = ser.read(n)
			#	print 'read this many bytes: ', n
			#	print buffer
				
			sleep(0.1)
	except rospy.ROSInterruptException:		
		print('exiting')
		pass
