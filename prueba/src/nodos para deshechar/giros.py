#!/usr/bin/env python
import roslib
roslib.load_manifest('prueba')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import constantes
from aruco_ros.msg import markerId
import numpy as np
import math
import pygame
from ordenes import Ordenes

from drone_controller import BasicDroneController
from drone_video_display import DroneVideoDisplay

# Finally the GUI libraries
from PySide import QtCore, QtGui

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
    Emergency        = QtCore.Qt.Key.Key_Space


class KeyboardController(DroneVideoDisplay):
    def __init__(self):
        super(KeyboardController,self).__init__()

        self.ordenes = Ordenes()


# We add a keyboard handler to the DroneVideoDisplay to react to keypresses
    def keyPressEvent(self, event):
        key = event.key()

        # If we have constructed the drone controller and the key is not generated from an auto-repeating key
        if self.ordenes is not None and not event.isAutoRepeat():
            # Handle the important cases first!
            if key == KeyMapping.Emergency:
                self.ordenes.emergencia()
            elif key == KeyMapping.Takeoff:
                self.ordenes.despega()
            elif key == KeyMapping.Land:
                self.ordenes.aterriza()
            elif key == KeyMapping.YawLeft:
                if not self.ordenes.estaGirando():
                    self.ordenes.gira90Izq()
            elif key == KeyMapping.YawRight:
                if not self.ordenes.estaGirando():
                    self.ordenes.corregirYaw(10)


# Setup the application
if __name__=='__main__':
    import sys
    # Firstly we setup a ros node, so that we can communicate with the other packages
    rospy.init_node('ardrone_keyboard_controller')

    # Now we construct our Qt Application and associated controllers and windows
    app = QtGui.QApplication(sys.argv)
    display = KeyboardController()

    display.show()

    # executes the QT application
    status = app.exec_()

    # and only progresses to here once the application has been shutdown
    rospy.signal_shutdown('Great Flying!')
    sys.exit(status)