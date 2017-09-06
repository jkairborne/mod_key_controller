#!/usr/bin/env python2

"""ROS Node for publishing desired positions."""

from __future__ import division, print_function, absolute_import

# Import libraries
import roslib
import rospy
import numpy as np
import math
from math import sin, cos, pi, atan2
from geometry_msgs.msg import TwistStamped, PoseStamped
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Int32

# Import class that computes the desired positions
# from aer1217_ardrone_simulator import PositionGenerator

__all__ = ['ROSDesiredPositionGenerator']

class ROSDesiredPositionGenerator(object):
    #ROS interface for publishing desired positions.

    def __init__(self):
        # Publishers
        # Desired position publisher
        self.pub_des_pose = rospy.Publisher('des_pos',PoseStamped,queue_size=300)
        # Desired velocity publisher
        self.pub_des_vel = rospy.Publisher('des_vel',TwistStamped,queue_size=300)
        # Desired path index
        self.sub_path = rospy.Subscriber('path', Int32, self.comm_path)
        self.sub_z = rospy.Subscriber('zpixel', Int32, self.comm_z)

        # Containers for desired pose and desired velocity to be published
    	self.quadrotor_pose = PoseStamped()
    	self.quadrotor_twist = TwistStamped()

        # Obtain start time in float seconds
    	self.startTime = rospy.get_time()

    	# Container for desired path index
        self.command_path = Int32()
        self.z_pixel = Int32() # receives hula hoop center height in image frame
        self.Z = 1.75 # start height
        self.flag = 0 # height correction flag

    def comm_path(self, path_msg): # subscriber to obtain desired path index
        self.command_path = path_msg

    def comm_z(self, z_msg): # subscriber to obtain hula hoop center height in image frame
        self.z_pixel = z_msg

    def command(self):
        # Time
    	currentTime = rospy.get_time() # get the current time in float seconds
    	time = currentTime - self.startTime # current time from the start

    	# (1/loop_rate_frequency) used to calculate future desired x and y position in order to obtain desired yaw
        time_step = 0.05 

        # X-Y coordinates to fly along line perpendicular to hula hoop plane
        X = [1.4142, 1.0607, 0.7070, 0, -1];

        # RViz base frame for visualization
        self.quadrotor_pose.header.frame_id = "map"

    	self.quadrotor_pose.header.stamp = rospy.Time.now() # timestamp desired pose
    	self.quadrotor_twist.header.stamp = rospy.Time.now() # timestamp desired velocity

        index = self.command_path.data # extract Int32 msg
        heightOffset = self.z_pixel.data # height offset to correct for

        # Hover at origin
        if index == 0:
            #print('Hover')

            # Desired position
            self.quadrotor_pose.pose.position.x = 0 # desired x position
            self.quadrotor_pose.pose.position.y = 0 # deisred y position
            self.quadrotor_pose.pose.position.z = -1 # desired z position

            # Desired velocities
            self.quadrotor_twist.twist.linear.x = 0 # desired x velocity obtained by differentiating desired x position
            self.quadrotor_twist.twist.linear.y = 0 # desired y velocity obtained by differentiating desired y position 

            # Desired yaw
            yaw = 0 # desired yaw

        elif index == 1:
            # Desired position
            self.quadrotor_pose.pose.position.x = -9999999 # desired x position
            self.quadrotor_pose.pose.position.y = -9999999 # deisred y position
            self.quadrotor_pose.pose.position.z = 9999999 # desired z position

            # Desired velocities
            self.quadrotor_twist.twist.linear.x = 0 # desired x velocity obtained by differentiating desired x position
            self.quadrotor_twist.twist.linear.y = 0 # desired y velocity obtained by differentiating desired y position
        
        elif index == 2:
            # Desired position
            self.quadrotor_pose.pose.position.x = -9999998 # desired x position
            self.quadrotor_pose.pose.position.y = -9999998 # deisred y position
            self.quadrotor_pose.pose.position.z = 9999998 # desired z position

            # Desired velocities
            self.quadrotor_twist.twist.linear.x = 0 # desired x velocity obtained by differentiating desired x position
            self.quadrotor_twist.twist.linear.y = 0 # desired y velocity obtained by differentiating desired y position
        elif (index == 3 or index==4):
            # Desired position
            self.quadrotor_pose.pose.position.x = -9999997 # desired x position
            self.quadrotor_pose.pose.position.y = -9999997 # deisred y position
            self.quadrotor_pose.pose.position.z = 9999997 # desired z position

            # Desired velocities
            self.quadrotor_twist.twist.linear.x = 0 # desired x velocity obtained by differentiating desired x position
            self.quadrotor_twist.twist.linear.y = 0 # desired y velocity obtained by differentiating desired y position


        # Desired orientation   
        yaw = 0 # 3*pi/2 #0# zero yaw (comment this for non-zero yaw)
    	(self.quadrotor_pose.pose.orientation.x,self.quadrotor_pose.pose.orientation.y,self.quadrotor_pose.pose.orientation.z,self.quadrotor_pose.pose.orientation.w) = quaternion_from_euler(0, 0, yaw) # desired roll, pitch, yaw converted to quaternion

        # Publish pose and velocity containers
        self.pub_des_pose.publish(self.quadrotor_pose) # publish desired pose
        self.pub_des_vel.publish(self.quadrotor_twist) # publish desired velocity

if __name__ == '__main__': # main function
    rospy.init_node('desired_positions') # initialize node
    obj1 = ROSDesiredPositionGenerator() # define class object
    rate = rospy.Rate(20) # loop rate

    while not rospy.is_shutdown():
        obj1.command() # call class function through object to publish desired pose and velocity
        rate.sleep() # used to loop at desired rate
    rospy.spin()
