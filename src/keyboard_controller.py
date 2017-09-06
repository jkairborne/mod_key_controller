#!/usr/bin/env python

# The Keyboard Controller Node for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials

# This controller extends the base DroneVideoDisplay class, adding a keypress handler to enable keyboard control of the drone

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib
import rospy

# Load the DroneController class, which handles interactions with the drone, and the DroneVideoDisplay class, which handles video display
from drone_controller import BasicDroneController
from drone_video_display import DroneVideoDisplay
from ardrone_autonomy.srv import *
import numpy as np

# Finally the GUI libraries
from PySide import QtCore, QtGui

# Import integer message
from std_msgs.msg import Int32

# Here we define the keyboard map for our controller (note that python has no enums, so we use a class)
class KeyMapping(object):
	PitchForward     = QtCore.Qt.Key.Key_E
	PitchBackward    = QtCore.Qt.Key.Key_D
	RollLeft         = QtCore.Qt.Key.Key_S
	RollRight        = QtCore.Qt.Key.Key_F
	YawLeft          = QtCore.Qt.Key.Key_W
	YawRight         = QtCore.Qt.Key.Key_R
	IncreaseAltitude = QtCore.Qt.Key.Key_Q
	DecreaseAltitude = QtCore.Qt.Key.Key_A
	Takeoff          = QtCore.Qt.Key.Key_Y
	Land             = QtCore.Qt.Key.Key_H
	###
	Emergency        = QtCore.Qt.Key.Key_Space
	ThrottleCut      = QtCore.Qt.Key.Key_K # kill switch
	OptiControl      = QtCore.Qt.Key.Key_M # Use Optitrack control
	PBVSControl      = QtCore.Qt.Key.Key_P # Use PBVS control
	IBVSControl      = QtCore.Qt.Key.Key_I # IBVS control
	ZP_IBVSControl   = QtCore.Qt.Key.Key_O # Z and Psi IBVS
	ZP_IBVS_startRot = QtCore.Qt.Key.Key_L # Start rotation of Z/Psi IBVS

# Our controller definition, note that we extend the DroneVideoDisplay class
class KeyboardController(DroneVideoDisplay):
	def __init__(self):
		super(KeyboardController,self).__init__()
		
		self.pitch = 0
		self.roll = 0
		self.yaw_velocity = 0 
		self.z_velocity = 0
		self.moving = False # quadrotor state

		self.pub_path = rospy.Publisher('path', Int32, queue_size=10)  # publish desired path index of integer type
		self.path_param = Int32() # Container for desired path index of integer type

# We add a keyboard handler to the DroneVideoDisplay to react to keypresses
	def keyPressEvent(self, event):
		key = event.key()

		# If we have constructed the drone controller and the key is not generated from an auto-repeating key
		if controller is not None and not event.isAutoRepeat():
			# Handle the important cases first!
			if key == KeyMapping.ThrottleCut:
				controller.SendEmergency()
			elif key == KeyMapping.Emergency:
				controller.SendLand() # modify to prevent damage to quad
			elif key == KeyMapping.Takeoff:
				controller.SendTakeoff()
				self.path_param = 0 # Set desired path index to hover
				self.pub_path.publish(self.path_param) # publish desired path index			
			elif key == KeyMapping.Land:
				controller.SendLand()	
			else:
				# Now we handle moving, notice that this section is the opposite (+=) of the keyrelease section
				if key == KeyMapping.YawLeft:
					self.yaw_velocity += 1
				elif key == KeyMapping.YawRight:
					self.yaw_velocity += -1

				elif key == KeyMapping.PitchForward:
					self.pitch += 1
				elif key == KeyMapping.PitchBackward:
					self.pitch += -1

				elif key == KeyMapping.RollLeft:
					self.roll += 1
				elif key == KeyMapping.RollRight:
					self.roll += -1

				elif key == KeyMapping.IncreaseAltitude:
					self.z_velocity += 1
				elif key == KeyMapping.DecreaseAltitude:
					self.z_velocity += -1
				elif key == KeyMapping.OptiControl:
					self.path_param = 0 # Update waypoint
					self.pub_path.publish(self.path_param) # publish desired path index
					print("In the opti control")
					self.moving = True
				elif key == KeyMapping.PBVSControl:
					self.path_param = 1 # Update waypoint
					self.pub_path.publish(self.path_param) # publish desired path index
					self.moving = True
					print("In the PBVS control")
				elif key == KeyMapping.IBVSControl:
					self.path_param = 2 #update Wpt
					self.pub_path.publish(self.path_param)
					self.moving = True
					print("In the IBVS control")
				elif key == KeyMapping.ZP_IBVSControl:
					self.path_param = 3
					self.pub_path.publish(self.path_param)
					self.moving = True
					print("In the ZP-IBVS control")
				elif key == KeyMapping.ZP_IBVS_startRot:
					self.path_param = 4
					self.pub_path.publish(self.path_param)
					self.moving = True
					print("In the ZP-IBVS control")


			if self.moving == False:	
				# finally we set the command to be sent. The controller handles sending this at regular intervals
				controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)


	def keyReleaseEvent(self,event):
		key = event.key()

		# If we have constructed the drone controller and the key is not generated from an auto-repeating key
		if controller is not None and not event.isAutoRepeat():
			# Note that we don't handle the release of emergency/takeoff/landing keys here, there is no need.
			# Now we handle moving, notice that this section is the opposite (-=) of the keypress section
			if key == KeyMapping.YawLeft:
				self.yaw_velocity -= 1
			elif key == KeyMapping.YawRight:
				self.yaw_velocity -= -1

			elif key == KeyMapping.PitchForward:
				self.pitch -= 1
			elif key == KeyMapping.PitchBackward:
				self.pitch -= -1

			elif key == KeyMapping.RollLeft:
				self.roll -= 1
			elif key == KeyMapping.RollRight:
				self.roll -= -1

			elif key == KeyMapping.IncreaseAltitude:
				self.z_velocity -= 1
			elif key == KeyMapping.DecreaseAltitude:
				self.z_velocity -= -1

			if self.moving == False:	
				# finally we set the command to be sent. The controller handles sending this at regular intervals
				controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)

# Setup the application
if __name__=='__main__':
	import sys
	# Firstly we setup a ros node, so that we can communicate with the other packages
	rospy.init_node('ardrone_keyboard_controller')

	# Now we construct our Qt Application and associated controllers and windows
	app = QtGui.QApplication(sys.argv)
	controller = BasicDroneController()
	display = KeyboardController()
	capture = DroneVideoDisplay()

	display.show()

	# executes the QT application
	status = app.exec_()

	# and only progresses to here once the application has been shutdown
	rospy.signal_shutdown('Great Flying!')
	sys.exit(status)
