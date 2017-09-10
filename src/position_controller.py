#!/usr/bin/env python2

"""Class for writing position controller."""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np
import math
from math import sin, cos, pi, asin

import serial, string, math, time, calendar


# Import class that computes the desired positions
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Twist, TwistStamped, PoseStamped, Vector3


class PositionController(object):
	#ROS interface for controlling the Parrot ARDrone in the Vicon Lab."""
	# write code here for position controller
	def __init__(self):
		self.model_name = 'ardrone'

		self.stval = 0.2

		# Containers
		self.opti_data = PoseStamped() # container to store incoming vicon data
		self.des_position = PoseStamped() # container to store incoming desired pose
		self.opti_pose = PoseStamped() # container to publish Vicon data as PoseStamped to visualize quadrotor pose in RViz
		self.des_velocity = TwistStamped() # container to store incoming desired velocity
		self.quadrotor_command = Twist() # container to publish command velocity to quadrotor

		self.opti_at_pbvs_receive = Twist()
		self.PBVSOpti = True
		self.CtrlToUse = "OPTI"

		self.pbvs_data = Twist()
		self.ibvs_data = Twist()
		self.vis_deltas = Vector3()

		self.targetInSight = False

		# Pblishers and subscribers
		self.sub_opti_data = rospy.Subscriber('/vrpn_client_node/{0}/pose'.format(self.model_name),PoseStamped, self.update_opti_data) # subscribe to incoming vicon data
		self.pub_des_vel = rospy.Publisher('cmd_vel',Twist,queue_size=2) # publish desired velocity commands to quadrotor
		self.visual_params = rospy.Publisher('visual_params',Vector3,queue_size=2) # publish desired velocity commands to quadrotor
		self.pub_opti_data = rospy.Publisher('opti_data',PoseStamped,queue_size=2) # publish vicon data for use in RViz
		self.sub_des_pose = rospy.Subscriber('des_pos',PoseStamped,self.des_pose) # subscribe to desired pose
		self.sub_des_vel = rospy.Subscriber('des_vel',TwistStamped,self.des_velo) # subscribe to desired velocity

		self.sub_PBVS_vel = rospy.Subscriber('cmd_vel_PBVS',Twist,self.update_PBVS_data) # receive PBVS desired velocity commands to quadrotor
		self.sub_IBVS_vel = rospy.Subscriber('cmd_vel_IBVS',Twist,self.update_IBVS_data) # receive PBVS desired velocity commands to quadrotor

		# Constants
		self.g = 9.81 # gravity constant

		# Tuning parameters
		self.zeta_x = 0.85 # 0.7 .. 1 - this is like the damping, so D
		self.omega_x = 1.1 # rise_time / 1.8 - this is like the P term, rise time
		self.zeta_y = 0.85 # 0.7 .. 1
		self.omega_y = 1.1 # rise_time / 1.8
		self.tau_z = 1
		self.tau_omega = 2.2

		# Initialization
		self.x_prev = 0 # previous x position
		self.y_prev = 0 # previous y position
		self.z_prev = 0 # previous z position
		self.z_vel_prev = 0 # previous z velocity
		self.x_vel_des = 0 # desired x velocity
		self.y_vel_des = 0 # desired y velocity

		# TIme tracking variables
		self.startTime = rospy.get_time(); # start time
		self.prevTime = self.startTime; # previous loop time

	def satur(self, val, satval):
		return (min(satval,max(val,-satval)))

	def des_pose(self, des_pos_msg): # subscriber to obtain desired pose
		self.des_position = des_pos_msg
		if (self.des_position.pose.position.x==-9999999) and self.des_position.pose.position.y==-9999999 and self.des_position.pose.position.z==9999999:
			self.CtrlToUse="PBVS"
		elif (self.des_position.pose.position.x==-9999998) and self.des_position.pose.position.y==-9999998 and self.des_position.pose.position.z==9999998:
			self.CtrlToUse = "IBVS"
		elif (self.des_position.pose.position.x==-9999997) and self.des_position.pose.position.y==-9999997 and self.des_position.pose.position.z==9999997:
			self.CtrlToUse = "ZP-IBVS"
		else:
			self.CtrlToUse = "OPTI"

	def des_velo(self, des_vel_msg): # subscriber to obtain desired velocity
		self.des_velocity = des_vel_msg	

	def update_opti_data(self, actual_opti_data): # subscriber to obtain vicon data
		self.opti_data.pose.position.x = actual_opti_data.pose.position.x
		self.opti_data.pose.position.y = actual_opti_data.pose.position.z
		self.opti_data.pose.position.z = -actual_opti_data.pose.position.y
		
		self.opti_data.pose.orientation.x = actual_opti_data.pose.orientation.x
		self.opti_data.pose.orientation.y = actual_opti_data.pose.orientation.z
		self.opti_data.pose.orientation.z = -actual_opti_data.pose.orientation.y
		self.opti_data.pose.orientation.w = actual_opti_data.pose.orientation.w
		#rospy.loginfo("x : %f    y : %f     z : %f      angular: %f   ", roll, pitch, z_vel, yaw_rate)

	def update_PBVS_data(self, actual_pbvs_data): # subscriber to obtain vicon data
		self.pbvs_data = actual_pbvs_data 
		self.deadReckoning = False # Just reset this to false every time.
		if(self.PBVSOpti): #This means we are fusing the optitrack data with our visual data to create the vel cmd.
			self.x_tgt = self.opti_data.pose.position.x - self.pbvs_data.angular.x # see September 9 2017 notes for why this is -ve
			self.y_tgt = self.opti_data.pose.position.y + self.pbvs_data.angular.y
			# Desired velocity
			#self.x_vel_pbvs = 0
			#self.y_vel_pbvs = 0
			self.x_vel_pbvs = self.pbvs_data.linear.x
			self.y_vel_pbvs = self.pbvs_data.linear.y
			self.vis_deltas.x = self.x_tgt
			self.vis_deltas.y = self.y_tgt
			self.vis_deltas.z = 0
			self.visual_params.publish(self.vis_deltas) # Publishes the optitrack coordinates we want to go to
		if (self.pbvs_data.linear.x == 0 and self.pbvs_data.linear.y == 0 and self.pbvs_data.linear.z == 0 and self.pbvs_data.angular.x == 111 and self.pbvs_data.angular.y ==111 and self.pbvs_data.angular.z == 0):
			self.targetInSight = False
		else:
			self.targetInSight = True
		#rospy.loginfo("x : %f    y : %f     z : %f      angular: %f   ", roll, pitch, z_vel, yaw_rate)
		
	def update_IBVS_data(self, actual_ibvs_data):
		self.ibvs_data = actual_ibvs_data		

	def desired(self):
		# Time
		currentTime = rospy.get_time() # Get the current time in float seconds
		time = currentTime - self.startTime # Current time from the start
		dt = currentTime - self.prevTime # Time step

		self.opti_pose.header.stamp = rospy.Time.now() # timestamp quadrotor pose
		self.opti_pose.header.frame_id = "map" # RViz base frame for visualization

		# store opti data as PoseStamped geometry msg for visualization in RViZ
		self.opti_pose = self.opti_data

		# Desired position and orientation
		self.x_des = self.des_position.pose.position.x
		self.y_des = self.des_position.pose.position.y
		self.z_des = self.des_position.pose.position.z
		quaternion = (self.des_position.pose.orientation.x,self.des_position.pose.orientation.y,self.des_position.pose.orientation.z,self.des_position.pose.orientation.w)
		(self.phi_des,self.theta_des,self.psi_des) = euler_from_quaternion(quaternion)

		# Desired velocity
		self.x_vel_des = self.des_velocity.twist.linear.x
		self.y_vel_des = self.des_velocity.twist.linear.y
		
		# Actual position and orientation
		self.x = self.opti_data.pose.position.x
		self.y = self.opti_data.pose.position.y
		self.z = self.opti_data.pose.position.z
		quaternion = (self.opti_data.pose.orientation.x,self.opti_data.pose.orientation.y,self.opti_data.pose.orientation.z,self.opti_data.pose.orientation.w)
		(angle1,angle2,self.psi) = euler_from_quaternion(quaternion)
		self.phi = -angle1
		self.theta = -angle2

		# Actual velocity and acceleration obtained by numerial differentiation
		self.x_vel = (self.x - self.x_prev)/dt
		self.y_vel = (self.y - self.y_prev)/dt
		self.z_vel = (self.z - self.z_prev)/dt
		z_accel = (self.z_vel - self.z_vel_prev)/dt

		# Controller
		f = (z_accel + self.g)/(cos(self.theta)*cos(self.phi))
		temp_x_accel_comm = (3*self.zeta_x*self.omega_x*(self.x_vel_des-self.x_vel)) + 3*((self.omega_x**2)*(self.x_des-self.x))
		temp_y_accel_comm = (3*self.zeta_y*self.omega_y*(self.y_vel_des-self.y_vel)) + 3*((self.omega_y**2)*(self.y_des-self.y))
		
		# Here we correct for varying psi angles:
		x_accel_comm = temp_x_accel_comm*cos(self.psi) + temp_y_accel_comm * sin(self.psi)
		y_accel_comm = temp_x_accel_comm*sin(self.psi) - temp_y_accel_comm * cos(self.psi)

		

		roll = asin(min(1,max((-y_accel_comm/f),-1))) # roll angle
		pitch = asin(min(1,max((x_accel_comm/(f*cos(roll))),-1))) #pitch angle
		z_vel = (self.z_des - self.z)/self.tau_z # z velocity
		yaw_rate = (self.psi_des - self.psi)/self.tau_omega # yaw rate
		#print("Modified: %.2f %.2f\n" % (pitch,-roll))

		if (self.CtrlToUse == "PBVS"):
			if(self.PBVSOpti):
			# In this case try to use the optitrack/PBVS controller - using the x_vel_pbvs and x_tgt/y equivalents in the Pbvs callback.
				#print("opti_x, pos_diff_x: %.2f %.2f" % (self.opti_at_pbvs_receive.linear.x ,self.pbvs_data.angular.x))
				#print("opti_y, pos_diff_y: %.2f %.2f" % (self.opti_at_pbvs_receive.linear.y ,self.pbvs_data.angular.y))
				print("x, y guessed by PBVS:  %.2f  %.2f" % (self.x_tgt, self.y_tgt))

				# Controller
				f = (z_accel + self.g)/(cos(self.theta)*cos(self.phi))
				temp_x_accel_comm = (self.zeta_x*self.omega_x*(self.x_vel_pbvs-self.x_vel)) + ((self.omega_x**2)*(self.x_tgt-self.x))
				temp_y_accel_comm = (self.zeta_y*self.omega_y*(self.y_vel_pbvs-self.y_vel)) + ((self.omega_y**2)*(self.y_tgt-self.y))
		
				# Here we correct for varying psi angles:
				x_accel_comm = temp_x_accel_comm*cos(self.psi) + temp_y_accel_comm * sin(self.psi)
				y_accel_comm = temp_x_accel_comm*sin(self.psi) - temp_y_accel_comm * cos(self.psi)

				roll = asin(min(1,max((y_accel_comm/f),-1))) # roll angle
				pitch = asin(min(1,max((x_accel_comm/(f*cos(roll))),-1))) #pitch angle

				self.quadrotor_command.linear.x = self.satur(pitch,self.stval)
				self.quadrotor_command.linear.y = self.satur(roll,self.stval)
				self.quadrotor_command.linear.z = self.pbvs_data.linear.z
				self.quadrotor_command.angular.x = 0
				self.quadrotor_command.angular.y = 0
				self.quadrotor_command.angular.z = self.pbvs_data.angular.z
				#print("PBVS control")
				#print("PBVS control: %.2f %.2f %.2f %.2f" % (self.quadrotor_command.linear.x,self.quadrotor_command.linear.y,self.quadrotor_command.linear.z,self.quadrotor_command.angular.z))

				if (self.targetInSight == False): # This means that the target is no longer in the FOV
					self.quadrotor_command.linear.x = 0
					self.quadrotor_command.linear.y = 0
					self.quadrotor_command.linear.z = 0
					self.quadrotor_command.angular.x = 0.1 #enable autohover
					self.quadrotor_command.angular.y = 0
					self.quadrotor_command.angular.z = 0

			else: #PBVS without Opti
				self.quadrotor_command.linear.x = self.satur(self.pbvs_data.linear.x,self.stval)
				self.quadrotor_command.linear.y = self.satur(self.pbvs_data.linear.y,self.stval)
				self.quadrotor_command.linear.z = self.pbvs_data.linear.z
				self.quadrotor_command.angular.x = 0
				self.quadrotor_command.angular.y = 0
				self.quadrotor_command.angular.z = self.pbvs_data.angular.z
		elif self.CtrlToUse=="IBVS":
			self.quadrotor_command = self.ibvs_data
			#print("IBVS control")
		elif self.CtrlToUse =="ZP-IBVS":
			self.quadrotor_command.linear.x = pitch #command from optitrack
			self.quadrotor_command.linear.y = -roll #command from optitrack
			self.quadrotor_command.linear.z = self.ibvs_data.linear.z
			self.quadrotor_command.angular.z = self.ibvs_data.angular.z
			#print("ZP_IBVS control")
		else:
			# Publish command velocity to quadrotor
			self.quadrotor_command.linear.x = pitch
			self.quadrotor_command.linear.y = -roll
			self.quadrotor_command.angular.z = -yaw_rate
			self.quadrotor_command.linear.z = -z_vel
		#print("pos_controller cmd_sent: %.2f %.2f %.2f %.2f" % (self.quadrotor_command.linear.x,self.quadrotor_command.linear.y,self.quadrotor_command.linear.z,self.quadrotor_command.angular.z))
		self.pub_des_vel.publish(self.quadrotor_command)
		#rospy.loginfo("x : %f    y : %f     z : %f      angular: %f   ", roll, pitch, z_vel, yaw_rate)

		# Publish vicon data as PoseStamped geometry msg
		self.pub_opti_data.publish(self.opti_pose)

		# Update previous values for next iteration
		self.prevTime = currentTime;
		self.x_prev = self.x
		self.y_prev = self.y
		self.z_prev = self.z
		self.z_vel_prev = self.z_vel

if __name__ == '__main__':
	rospy.init_node('position_controller') # initialize node
	obj = PositionController() # define class object
	rate = rospy.Rate(20) # loop rate

	while not rospy.is_shutdown():
		obj.desired() # call class function through object to publish command velocity 
		rate.sleep() # used to loop at desired rate
	rospy.spin()
