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
from geometry_msgs.msg import PoseStamped
from aruco_ros.msg import markerId
from cv_bridge import CvBridge, CvBridgeError
import constantes
import numpy as np
from ordenes import Ordenes
from ardrone_autonomy.msg import Navdata
import yaml
import signal
from geometry_msgs.msg import Twist
class lector:

    def __init__(self):
        self.rate = rospy.Rate(10) #10Hz
        self.contImg = 0
        self.contImgAruco = 0
        cv2.namedWindow("Image window", 1)
        self.bridge = CvBridge()
        self.accion_act = None
        self.accion_ant = None
        self.navdata = None
        self.path = '/home/david/proyecto/fotos/'
        self.file = open(self.path + 'navdata.yaml', 'w')
        self.orden = ''
        self.cmd = None

        """
        Para usar la camara de abajo cambiar '/ardrone/image_raw' por
        '/ardrone/botton/image_raw' luego antes de ejecutar el nodo
        ejecutar el comando rosservice call /ardrone/togglecam
        """
        #Subscribers
        self.image_sub = rospy.Subscriber('ardrone/image_raw',Image,self.imageCallback)
        self.aruco_sub = rospy.Subscriber('/aruco_single/result',Image,self.arucoCallback)
        self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata)
        self.orde_sub = rospy.Subscriber('/distAruco/orden',String,self.ordenCallback)
        self.command_sub = rospy.Subscriber('/cmd_vel',Twist,self.commandCallback)
        signal.signal(signal.SIGINT, self.intHandler)

    def commandCallback(self, cmd):
        self.cmd = cmd
        print 'cmd'

    def ordenCallback(self, orden):
        print 'orden2 ' + orden.data
        self.orden = orden.data

    def intHandler(self, signum, frame):
        print "user pressed ctrl-c"
        self.file.close()
        raise KeyboardInterrupt()

    def ReceiveNavdata(self,navdata):
        self.navdata = navdata

    def writeYaml(self, captureNumber, imgFileName, navdata):
        d = {'capture ' +  str(captureNumber) : {'imageFileName' : imgFileName,
                          'orden' : self.orden,
                          'cmdvel' : str(self.cmd),
                          'state' : str(navdata.state),
                          'rotX' : str(navdata.rotX),
                          'rotY' : str(navdata.rotY),
                          'rotZ' : str(navdata.rotZ),
                          'magX' : str(navdata.magX),
                          'magY' : str(navdata.magY),
                          'magZ' : str(navdata.magZ),
                          'batteryPercent' : str(navdata.batteryPercent),
                          'pressure' : str(navdata.pressure),
                          'temp' : str(navdata.temp),
                          'wind_speed' : str(navdata.wind_speed),
                          'wind_angle' : str(navdata.wind_angle),
                          'wind_comp_angle' : str(navdata.wind_comp_angle),
                          'altd' : str(navdata.altd),
                          'motor1' : str(navdata.motor1),
                          'motor2' : str(navdata.motor2),
                          'motor3' : str(navdata.motor3),
                          'motor4' : str(navdata.motor4),
                          'vx' : str(navdata.vx),
                          'vy' : str(navdata.vy),
                          'vz' : str(navdata.vz),
                          'ax' : str(navdata.ax),
                          'ay' : str(navdata.ay),
                          'az' : str(navdata.az),
                          'tm' : str(navdata.tm),
                          'header' : {'seq' : str(navdata.header.seq),
                                      'stamp.secs' : str(navdata.header.stamp.secs),
                                      'stamp.nsecs' : str(navdata.header.stamp.nsecs),
                                      'frame_id' : navdata.header.frame_id}}}
        self.file.write( yaml.dump(d, default_flow_style=False))

    def grabar(self, img, outName):
            fileName = ''
            if self.contImg < 10:
                fileName = self.path + outName + '_000'+str(self.contImg)+'.jpeg'
                cv2.imwrite(fileName, img)
            elif self.contImg < 100:
                fileName = self.path + outName + '_00'+str(self.contImg)+'.jpeg'
                cv2.imwrite(fileName, img)
            elif self.contImg < 1000:
                fileName = self.path + outName + '_0'+str(self.contImg)+'.jpeg'
                cv2.imwrite(fileName, img)
            else:
                fileName = self.path + outName + '_'+str(self.contImg)+'.jpeg'
                cv2.imwrite(fileName, img)
            self.contImg = self.contImg + 1
            return fileName


    def grabarAruco(self, img, outName):
            fileName = ''
            if self.contImgAruco < 10:
                fileName = self.path + outName + '_000'+str(self.contImgAruco)+'.jpeg'
                cv2.imwrite(fileName, img)
            elif self.contImgAruco < 100:
                fileName = self.path + outName + '_00'+str(self.contImgAruco)+'.jpeg'
                cv2.imwrite(fileName, img)
            elif self.contImgAruco < 1000:
                fileName = self.path + outName + '_0'+str(self.contImgAruco)+'.jpeg'
                cv2.imwrite(fileName, img)
            else:
                fileName = self.path + outName + '_'+str(self.contImgAruco)+'.jpeg'
                cv2.imwrite(fileName, img)
            self.contImgAruco = self.contImgAruco + 1
            return fileName



    def imageCallback(self,data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            fileName = self.grabar(img, 'salida_raw')
            self.writeYaml(self.contImg-1, fileName, self.navdata)
        except CvBridgeError, e:
            print e


    def arucoCallback(self,data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.grabarAruco(img, 'salida_aruco')
        except CvBridgeError, e:
            print e



def main(args):
    rospy.init_node('lector', anonymous=True)
    try:
        ic = lector()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

    #cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
