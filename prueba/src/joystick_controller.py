#!/usr/bin/env python

# The Joystick Controller Node for the tutorial "Up and flying with the AR.Drone and ROS | Joystick Control"
# https://github.com/mikehamer/ardrone_tutorials

# This controller implements the base DroneVideoDisplay class, the DroneController class and subscribes to joystick messages

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('prueba')
import rospy

# Load the DroneController class, which handles interactions with the drone, and the DroneVideoDisplay class, which handles video display
from drone_controller import BasicDroneController
from drone_video_display import DroneVideoDisplay

# Import the joystick message
from sensor_msgs.msg import Joy

# Finally the GUI libraries
from PySide import QtCore, QtGui

# define the default mapping between joystick buttons and their corresponding actions
ButtonEmergency = 13
ButtonLand      = 15
ButtonTakeoff   = 14

ButtonUp = 8
ButtonDown = 9
ButtonPitchForward = 4
ButtonPitchBackward = 6
ButtonRollLeft = 7
ButtonRollRight = 5
ButtonYawLeft = 10
ButtonYawRight = 11


# handles the reception of joystick packets
def ReceiveJoystickMessage(data):
	if data.buttons[ButtonEmergency]==1:
		rospy.loginfo("Emergency Button Pressed")
		controller.SendEmergency()
	elif data.buttons[ButtonLand]==1:
		rospy.loginfo("Land Button Pressed")
		controller.SendLand()
	elif data.buttons[ButtonTakeoff]==1:
		rospy.loginfo("Takeoff Button Pressed")
		controller.SendTakeoff()
	elif data.buttons[ButtonUp]==1:
		rospy.loginfo("Up Button Pressed")
		controller.SetCommand(0,0,0,1)
	elif data.buttons[ButtonDown]==1:
		rospy.loginfo("Down Button Pressed")
		controller.SetCommand(0,0,0,-1)
	elif data.buttons[ButtonPitchForward]==1:
		rospy.loginfo("Pitch Forward Button Pressed")
		controller.SetCommand(0,1,0,0)
	elif data.buttons[ButtonPitchBackward]==1:
		rospy.loginfo("Pitch Backward Button Pressed")
		controller.SetCommand(0,-1,0,0)
	elif data.buttons[ButtonRollLeft]==1:
		rospy.loginfo("Roll Left Button Pressed")
		controller.SetCommand(1,0,0,0)
	elif data.buttons[ButtonRollRight]==1:
		rospy.loginfo("Roll Right Button Pressed")
		controller.SetCommand(-1,0,0,0)
	elif data.buttons[ButtonYawLeft]==1:
		rospy.loginfo("Yaw Left Button Pressed")
		controller.SetCommand(0,0,1,0)
	elif data.buttons[ButtonYawRight]==1:
		rospy.loginfo("Yaw Right Button Pressed")
		controller.SetCommand(0,0,-1,0)
	else:
		controller.SetCommand(0,0,0,0)

# Setup the application
if __name__=='__main__':
	import sys
	# Firstly we setup a ros node, so that we can communicate with the other packages
	rospy.init_node('ardrone_joystick_controller')

	# Next load in the parameters from the launch-file
	ButtonEmergency = int (   rospy.get_param("~ButtonEmergency",ButtonEmergency) )
	ButtonLand      = int (   rospy.get_param("~ButtonLand",ButtonLand) )
	ButtonTakeoff   = int (   rospy.get_param("~ButtonTakeoff",ButtonTakeoff) )
	ButtonUp   = int (   rospy.get_param("~ButtonUp",ButtonUp) )
	ButtonDown   = int (   rospy.get_param("~ButtonDown",ButtonDown) )
	ButtonPitchForward   = int (   rospy.get_param("~ButtonPitchForward",ButtonPitchForward) )
	ButtonPitchBackward   = int (   rospy.get_param("~ButtonPitchBackward",ButtonPitchBackward) )
	ButtonRollLeft   = int (   rospy.get_param("~ButtonRollLeft",ButtonRollLeft) )
	ButtonRollRight   = int (   rospy.get_param("~ButtonRollRight",ButtonRollRight) )
	ButtonYawLeft   = int (   rospy.get_param("~ButtonYawLeft",ButtonYawLeft) )
	ButtonYawRight   = int (   rospy.get_param("~ButtonYawRight",ButtonYawRight) )


	# Now we construct our Qt Application and associated controllers and windows
	app = QtGui.QApplication(sys.argv)
	display = DroneVideoDisplay()
	controller = BasicDroneController()

	# subscribe to the /joy topic and handle messages of type Joy with the function ReceiveJoystickMessage
	subJoystick = rospy.Subscriber('/joy', Joy, ReceiveJoystickMessage)

	# executes the QT application
	display.show()
	status = app.exec_()

	# and only progresses to here once the application has been shutdown
	rospy.signal_shutdown('Great Flying!')
	sys.exit(status)
