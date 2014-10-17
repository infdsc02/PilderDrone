#!/usr/bin/env python
import roslib
roslib.load_manifest('prueba')
import sys
import rospy
import cv2
import cv2.cv as cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
import Image as Img
import math
import os
import pygame
from geometry_msgs.msg import Point
from aruco_ros.msg import markerId
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from ordenes import Ordenes
from std_msgs.msg import Bool
from drone_status import DroneStatus

class lector:

    def __init__(self):
        self.rate = rospy.Rate(10) #10Hz
        self.sonido1 = '/home/david/proyecto/a.wav'
        self.sonido2 = '/home/david/proyecto/blaster.wav'
        self.contImg = 0
        self.ordenes = Ordenes()
        cv2.namedWindow("Image window", 1)
        self.bridge = CvBridge()
        self.img = None
        self.manualMode = False
        """
        Para usar la camara de abajo cambiar '/ardrone/image_raw' por
        '/ardrone/botton/image_raw' luego antes de ejecutar el nodo
        ejecutar el comando rosservice call /ardrone/togglecam
        """
        #Subscribers
        self.image_sub = rospy.Subscriber('/aruco_single/result',Image,self.imageCallback)
        self.marker_center_sub = rospy.Subscriber('/aruco_single/markerCenter',Point,self.centerCallback)
        self.marker_detected_sub = rospy.Subscriber('/aruco_single/markerDetected',Bool,self.markerDetectedCallback)
        self.manual_mode_sub = rospy.Subscriber('/joystick_controller/manualMode',Bool,self.manualModeCallback)

    def markerDetectedCallback(self, detected):
        print detected
        if not detected and not self.manualMode:
            print 'TIENE QUE ATERRIZAR'
            self.ordenes.aterriza()

    def manualModeCallback(self, manualMode):
        self.manualMode = manualMode

    def centerCallback(self, center):
        height, width, depth = self.img.shape
        umbral = 20
        if (center.x <= umbral + width/2) and (center.x > width/2 - umbral): #and (center.y <= umbral + height/2) and (center.y > height/2 - umbral):
            self.ordenes.para()
        elif center.x < width/2 - umbral:
            if not self.manualMode and self.ordenes.getStatus() == DroneStatus.Hovering:
                self.ordenes.despLateralDer()
        elif center.x <= umbral + width/2:
            if not self.manualMode and self.ordenes.getStatus() == DroneStatus.Hovering:
                self.ordenes.despLateralIzq()


    def sonido(self, sonido):
        pygame.mixer.init()
        sound = pygame.mixer.Sound(sonido)
        sound.play()

    def dibujaMira(self, width, height, img):
        cv2.line(img,(0,height/2),(width,height/2),(0,0,255),1)
        cv2.line(img,(width/2,0),(width/2,height),(0,0,255),1)
        cv2.circle(img,(width/2,height/2), 4, (0,0,255), 1)
        cv2.circle(img,(width/2,height/2), 10, (0,0,255), 1)
        cv2.circle(img,(width/2,height/2), 20, (0,0,255), 1)
        cv2.circle(img,(width/2,height/2), 30, (0,0,255), 1)
        cv2.circle(img,(width/2,height/2), 40, (0,0,255), 1)

    def imageCallback(self,data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")

            height, width, depth = self.img.shape
            self.dibujaMira(width, height, self.img)

        except CvBridgeError, e:
            print e
        cv2.imshow("Image window", self.img)
        cv2.waitKey(3)


def main(args):
    rospy.init_node('lector', anonymous=True)
    ic = lector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    #cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
