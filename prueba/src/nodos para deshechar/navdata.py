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

# An enumeration of Drone Statuses
from drone_status import DroneStatus


# Some Constants
COMMAND_PERIOD = 100 #ms


class navdata(object):
    def __init__(self):
        # Holds the current drone status
        self.status = -1
        self.navdata = None
        # Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
        self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata)


    def ReceiveNavdata(self,navdata):
        self.navdata = navdata
        rotZ=-1
        if  self.navdata.rotZ < 0:
            rotZ = self.navdata.rotZ + 360
        else:
            rotZ = self.navdata.rotZ
    #    info = "rotX " + str(self.navdata.rotX) + " rotY " + str(self.navdata.rotY)+ " rotZ " + str(rotZ)+ " Alt " + str(self.navdata.altd)+ " vX " + str(self.navdata.vx)+ " vY " + str(self.navdata.vy)+ " vZ " + str(self.navdata.vz)+ " aX " + str(self.navdata.ax)+ " aY " + str(self.navdata.ay)+ " aZ " + str(self.navdata.az)
        info = 'rotZ ' + str(rotZ)
        rospy.loginfo(info)


def main(args):
    rospy.init_node('navdata', anonymous=True)
    ic = navdata()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__ == '__main__':
    main(sys.argv)





