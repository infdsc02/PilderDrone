#!/usr/bin/env python

# A basic drone controller class for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

# This class implements basic control functionality which we will be using in future tutorials.
# It can command takeoff/landing/emergency as well as drone movement
# It also tracks the drone state based on navdata feedback

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib;
import rospy
import sys
# Import the messages we're interested in sending and receiving
from geometry_msgs.msg import Twist  	 # for sending commands to the drone
from std_msgs.msg import Empty       	 # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

# An enumeration of Drone Statuses
from drone_status import DroneStatus


# Some Constants
COMMAND_PERIOD = 10 #ms


class BasicDroneController(object):
	def __init__(self):
		# Holds the current drone status
		self.status = -1
		self.navdata = None
		self.logFile = open('salida.txt', 'w')
		self.nVueltas = 0
		self.primeraVuelta = True
		self.rotZ_ant = 0

		self.roll = 0
		self.pitch = 0
		self.yaw = 0
		self.zVelocity = 0

		# Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata)

		# Allow the controller to publish to the /ardrone/takeoff, land and reset topics
		#self.pubLand    = rospy.Publisher('/ardrone/land',Empty)
		#self.pubTakeoff = rospy.Publisher('/ardrone/takeoff',Empty)
		#self.pubReset   = rospy.Publisher('/ardrone/reset',Empty)

		# Allow the controller to publish to the /cmd_vel topic and thus control the drone
		self.pubCommand = rospy.Publisher('/cmd_vel',Twist)

		# Setup regular publishing of control packets
		self.command = Twist()
		self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),self.SendCommand)

		# Land the drone if we are shutting down
		#rospy.on_shutdown(self.SendLand)

	def getNavdata(self):
		return self.navdata

	def ReceiveNavdata(self,navdata):
		# Although there is a lot of data in this packet, we're only interested in the state at the moment
		self.status = navdata.state
		self.navdata = navdata

	"""
	def SendTakeoff(self):
		# Send a takeoff message to the ardrone driver
		# Note we only send a takeoff message if the drone is landed - an unexpected takeoff is not good!
		if(self.status == DroneStatus.Landed):
			self.pubTakeoff.publish(Empty())

	def SendLand(self):
		# Send a landing message to the ardrone driver
		# Note we send this in all states, landing can do no harm
		self.pubLand.publish(Empty())

	def SendEmergency(self):
		# Send an emergency (or reset) message to the ardrone driver
		self.pubReset.publish(Empty())
	"""
	def getYawVelocity(self):
		return self.yaw

	def getLinearZ(self):
		return self.zVelocity

	def getPitch(self):
		return self.pitch

	def writeLog(self, msg):
		aux = sys.stdout
		rospy.loginfo(msg)
		sys.stdout = self.logFile
		rospy.loginfo(msg)
		sys.stdout = aux

	def contVueltas (self, rotZ_act):
		if not self.primeraVuelta:
			if self.rotZ_ant < 0 and rotZ_act >= 0 and self.rotZ_ant > -90:
				self.nVueltas = self.nVueltas + 1
			elif self.rotZ_ant >= 0 and rotZ_act < 0  and self.rotZ_ant < 90:
				self.nVueltas = self.nVueltas - 1
		else:
			self.primeraVuelta = False

		self.rotZ_ant = rotZ_act

	def getRotZ(self):
		rotZ_real = self.navdata.rotZ
		self.contVueltas(rotZ_real)
		if rotZ_real >= 0:
			rotZ_calculado = rotZ_real + 360*self.nVueltas
		else:
			rotZ_calculado = rotZ_real + 360*(self.nVueltas+1)

		return rotZ_calculado

	def setRoll(self, roll):
		self.roll = roll

	def setPitch(self, pitch):
		self.pitch = pitch

	def setYaw(self, yaw):
		self.yaw = yaw

	def setZvelocity(self, zVelocity):
		self.zVelocity = zVelocity

	def SetCommand(self, roll, pitch, yaw, zVelocity):
		# Called by the main program to set the current command
		self.pitch = pitch
		self.roll = roll
		self.zVelocity = zVelocity
		self.yaw = yaw


	def SendCommand(self, event):
		self.command.linear.x  = self.pitch
		self.command.linear.y  = self.roll
		self.command.linear.z  = self.zVelocity
		self.command.angular.x = 0
		self.command.angular.y = 0
		self.command.angular.z = self.yaw

		# The previously set command is then sent out periodically if the drone is flying
		if self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering:
			self.pubCommand.publish(self.command)

