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
from geometry_msgs.msg import Point
from aruco_ros.msg import markerId
from cv_bridge import CvBridge, CvBridgeError
import constantes
import numpy as np
from ordenes import Ordenes
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from drone_status import DroneStatus
from drone_controller import BasicDroneController
from aruco_ros.msg import points
from geometry_msgs.msg import PoseStamped
from ardrone_autonomy.msg import Navdata
import tf
from drone_status import DroneStatus
from geometry_msgs.msg import Twist
import time

class lector:

    def __init__(self):
        self.rate = rospy.Rate(10) #10Hz
        self.sonido1 = '/home/david/proyecto/a.wav'
        self.sonido2 = '/home/david/proyecto/blaster.wav'
        self.contImg = 0
        self.ordenes = Ordenes()
        #self.controller = BasicDroneController()
        cv2.namedWindow("Image window", 1)
        self.bridge = CvBridge()
        self.img = None
        self.manualMode = False
        self.nMuestra = 0
        self.posX = 0
        self.posY = 0
        self.posZ = 0
        self.oriX = 0
        self.oriY = 0
        self.oriZ = 0
        self.oriW = 0
        self.navdata = None

        self.centered = False

        self.distAct = 2
        self.markerDetected = False
        self.markerDetected_ant = False

        #Subscribers
        self.image_sub = rospy.Subscriber('/aruco_single/result',Image,self.imageCallback)

        #self.marker_points_sub = rospy.Subscriber('/aruco_single/markerPoints',points,self.pointsCallback)

        self.marker_pose_sub = rospy.Subscriber('/aruco_single/pose',PoseStamped,self.distCallback)
        self.dist_sub = rospy.Subscriber('distance',Float32,self.distActCallback)
        self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata)


    def ReceiveNavdata(self,navdata):
        self.navdata = navdata

    def distActCallback(self, data):
        self.distAct = data.data


    def distance(self, distance):
        umbral = 0.4 #40 cm
        height, width, depth = self.img.shape
        font = cv2.FONT_HERSHEY_SIMPLEX
        if self.ordenes.getStatus() != DroneStatus.Landed or self.ordenes.getStatus() != DroneStatus.TakingOff:
            cv2.putText(self.img,'Dist act: ' + str(distance) ,(0,height-20), font, 0.75,(255,255,255),1,255)
            cv2.putText(self.img,'Dist fijada: ' + str(self.distAct) ,(0,25), font, 0.75,(255,255,255),1,255)
            print 'Distancia ' + str(distance)
            if (distance < self.distAct - umbral):
                self.ordenes.retrocede(0.25)
                self.markerDetected_ant = True
                print 'Atras'
            elif (distance > self.distAct + umbral):
                self.ordenes.avanza(0.25)
                self.markerDetected_ant = True
                print 'Adelante'
            elif (distance > self.distAct - umbral) and (distance < self.distAct + umbral):
                self.ordenes.para()
                """
                time.sleep(1)
                if not self.ordenes.girando:
                    self.ordenes.gira180()
                """
                self.markerDetected_ant = False
                print 'Distancia correcta'


    def distCallback(self, poseS):
        pose = poseS.pose
        position = pose.position


        if self.img != None:
            self.distance(position.z)
            cv2.imshow("dist", self.img)
            cv2.waitKey(3)


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
            """
            if not self.markerDetected:
                #if self.ordenes.getStatus() != DroneStatus.Landed:
                self.ordenes.para()
                self.markerDetected_ant = False
            """
        except CvBridgeError, e:
            print e



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
