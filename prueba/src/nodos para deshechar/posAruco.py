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


        self.command = Twist()

        self.distAct = 1.5

        self.markerDetected = False

        self.center = None

        #Subscribers
        self.image_sub = rospy.Subscriber('/aruco_single/result',Image,self.imageCallback)

        self.marker_center_sub = rospy.Subscriber('/aruco_single/markerCenter',Point,self.centerCallback)

        #self.marker_points_sub = rospy.Subscriber('/aruco_single/markerPoints',points,self.pointsCallback)

        self.marker_pose_sub = rospy.Subscriber('/aruco_single/pose',PoseStamped,self.poseCallback)
        self.marker_detected_sub = rospy.Subscriber('/aruco_single/markerDetected',Bool,self.markerDetectedCallback)
        self.dist_sub = rospy.Subscriber('/getDistance/distance',Float32,self.distActCallback)
        self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata)

    def markerDetectedCallback(self, detected):
        self.markerDetected = detected.data

    def ReceiveNavdata(self,navdata):
        self.navdata = navdata

    def distActCallback(self, dist):
        self.distAct = dist

    def poseCallback(self, poseS):
        pose = poseS.pose
        position = pose.position

        self.posX = position.x
        self.posY = position.y
        self.posZ = position.z



    def moveToCenter(self, centerPos):
        height, width, depth = self.img.shape
        umbral = 50
        font = cv2.FONT_HERSHEY_SIMPLEX
        if ((centerPos.y <= umbral + height/2) and (centerPos.y > height/2 - umbral)) and ((centerPos.x <= umbral + width/2) and (centerPos.x > width/2 - umbral)):
            cv2.putText(self.img,'centered',(0,height), font, 1,(255,255,255),1,255)
            self.ordenes.para()
            self.centered = True
            print 'centered'
        elif (centerPos.y > umbral + height/2) and (centerPos.x > umbral + width/2):
            cv2.putText(self.img,'Baja Derecha',(0,height), font, 1,(255,255,255),1,255)
            self.ordenes.bajaDer()
            self.centered = False
        elif (centerPos.y > umbral + height/2) and (centerPos.x < width/2 - umbral):
            cv2.putText(self.img,'Baja Izquierda',(0,height), font, 1,(255,255,255),1,255)
            self.ordenes.bajaIzq()
            self.centered = False
        elif (centerPos.y < height/2 - umbral) and (centerPos.x > umbral + width/2):
            cv2.putText(self.img,'Sube Derecha',(0,height), font, 1,(255,255,255),1,255)
            self.ordenes.subeDer()
            self.centered = False
        elif (centerPos.y < height/2 - umbral) and (centerPos.x < width/2 - umbral):
            cv2.putText(self.img,'Sube Izquierda',(0,height), font, 1,(255,255,255),1,255)
            self.ordenes.subeIzq()
            self.centered = False
        elif (centerPos.y > umbral + height/2):
            cv2.putText(self.img,'Baja',(0,height), font, 1,(255,255,255),1,255)
            self.ordenes.baja()
            self.centered = False
            print 'Baja'
        elif (centerPos.y < height/2 - umbral):
            cv2.putText(self.img,'Sube',(0,height), font, 1,(255,255,255),1,255)
            self.ordenes.sube()
            self.centered = False
            print 'Sube'
        elif (centerPos.x > umbral + width/2):
            cv2.putText(self.img,'Derecha',(0,height), font, 1,(255,255,255),1,255)
            self.ordenes.despLateralDer()
            self.centered = False
            print 'Derecha'
        elif (centerPos.x < width/2 - umbral):
            cv2.putText(self.img,'Izquierda',(0,height), font, 1,(255,255,255),1,255)
            self.ordenes.despLateralIzq()
            self.centered = False
            print 'Izquierda'
    """
    def distance(self, distance):
        umbral = 0.1 #10 cm

        if (distance< self.distAct - umbral):
            self.pitch = -1
            print 'Atras'
        elif (distance > self.distAct + umbral):
            self.pitch = 1
            print 'Adelante'
        elif (distance > self.distAct - umbral) and (distance < self.distAct + umbral):
            self.pitch = 0
            self.roll = 0
            self.zVelocity = 0
            self.SetCommand(self.roll, self.pitch, self.yaw, self.zVelocity)
            self.SendCommand(self.command)
            self.ordenes.gira180()
            print 'Distancia correcta'

    """
    def centerCallback(self, center):
        self.center = center


    def dibujaMira(self, width, height, img):
        cv2.line(img,(0,height/2),(width,height/2),(0,0,255),1)
        cv2.line(img,(width/2,0),(width/2,height),(0,0,255),1)
        cv2.circle(img,(width/2,height/2), 4, (0,0,255), 1)
        cv2.circle(img,(width/2,height/2), 10, (0,0,255), 1)
        cv2.circle(img,(width/2,height/2), 20, (0,0,255), 1)
        cv2.circle(img,(width/2,height/2), 30, (0,0,255), 1)
        cv2.circle(img,(width/2,height/2), 40, (0,0,255), 1)

    def imageCallback(self,data):
        font = cv2.FONT_HERSHEY_SIMPLEX
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")

            height, width, depth = self.img.shape

            self.dibujaMira(width, height, self.img)
            cv2.putText(self.img,'Marker detected ' + str(self.markerDetected),(0,125), font, 0.75,(255,255,255),1,255)
            if self.markerDetected:
                self.moveToCenter(self.center)
            else:
                self.ordenes.para()
            cv2.imshow("centered", self.img)
            cv2.waitKey(3)
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
