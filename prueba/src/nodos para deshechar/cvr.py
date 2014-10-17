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
from geometry_msgs.msg import Twist       # for sending commands to the drone
from std_msgs.msg import Empty            # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback
from drone_controller import BasicDroneController

# An enumeration of Drone Statuses
from drone_status import DroneStatus


# Some Constants
COMMAND_PERIOD = 100 #ms


class navdata(object):
    def __init__(self):
        # Holds the current drone status
        self.status = -1
        self.navdata = None
        self.controller = BasicDroneController()

    def getRotZ(self):
        rotZ = self.controller.getNavdata().rotZ
        if  rotZ < 0:
            return rotZ + 360
        else:
            return rotZ

def main(args):
    rospy.init_node('navdata', anonymous=True)
    ic = navdata()
    i = 1
    girando = 0
    angulo_destino = 0
    while i != 0:
        if girando != 0:
            if (ic.getRotZ() < angulo_destino-5):
                ic.controller.SetCommand(0, 0, 0.2, 0)
            elif (ic.getRotZ() > angulo_destino+5):
                ic.controller.SetCommand(0, 0, -0.2, 0)
            else:
                girando = 0
                print 'LLEGAMOS!!!!!!!'
                ic.controller.SetCommand(0, 0, 0, 0)
        else:
            i = int(input())
            if i != 0:
                girando = 1
                angulo_destino = ic.getRotZ() + i * 10
                print angulo_destino

if __name__ == '__main__':
    main(sys.argv)





