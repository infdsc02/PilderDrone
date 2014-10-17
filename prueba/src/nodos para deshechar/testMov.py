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
import numpy as np
from ordenes import Ordenes
from std_msgs.msg import Bool
from drone_status import DroneStatus
from drone_controller import BasicDroneController
from aruco_ros.msg import points
from geometry_msgs.msg import PoseStamped
from ardrone_autonomy.msg import Navdata
import tf

class lector:

    def __init__(self):
        self.rate = rospy.Rate(10) #10Hz
        self.sonido1 = '/home/david/proyecto/a.wav'
        self.sonido2 = '/home/david/proyecto/blaster.wav'
        self.contImg = 0
        self.ordenes = Ordenes()
        self.controller = BasicDroneController()
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
        self.euler = None
        #Subscribers
        self.image_sub = rospy.Subscriber('/aruco_single/result',Image,self.imageCallback)

        #self.marker_center_sub = rospy.Subscriber('/aruco_single/markerCenter',Point,self.centerCallback)

        #self.marker_points_sub = rospy.Subscriber('/aruco_single/markerPoints',points,self.pointsCallback)

        self.marker_pose_sub = rospy.Subscriber('/aruco_single/pose',PoseStamped,self.poseCallback)
        self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata)

    """
    def virtHorz(self, angY, angZ):
        height, width, depth = self.img.shape

        horz = (angY + 23) * 360/43
        vert = (angZ + 30) * 32/3

        if horz > 360:
            horz = 360.0
        elif horz < 0:
            horz = 0.0

        horz = int(round(horz))

        if vert > 640:
            vert = 640.0
        elif vert < 0:
            vert = 0.0

        vert = int(round(vert))

        cv2.line(self.img,(0,horz),(width,horz),(255,0,0),1)
        cv2.line(self.img,(vert,0),(vert,height),(255,0,0),1)
    """

    def virtHorz(self, angX, angY, angZ):
        height, width, depth = self.img.shape

        horz = (angY + 23) * 360/43
        vert = (angZ + 30) * 32/3

        if horz > 360:
            horz = 360.0
        elif horz < 0:
            horz = 0.0


        if vert > 640:
            vert = 640.0
        elif vert < 0:
            vert = 0.0

        x1 = -width/2
        y1 = horz - height/2
        x2 = width/2
        y2 = horz - height/2
        x1 = int(round(x1 * math.cos(angX) - y1 * math.sin(angX)))
        y1 = int(round(x1 * math.sin(angX) + y1 * math.cos(angX)))
        x2 = int(round(x2 * math.cos(angX) - y2 * math.sin(angX)))
        y2 = int(round(x2 * math.sin(angX) + y2 * math.cos(angX)))

        cv2.line(self.img,(x1,y1),(x2,y2),(255,0,0),1)
        #cv2.line(self.img,(vert,0),(vert,height),(255,0,0),1)

    def ReceiveNavdata(self,navdata):
        self.navdata = navdata

    def poseCallback(self, poseS):
        pose = poseS.pose
        position = pose.position
        orientation = pose.orientation
        #height, width, depth = self.img.shape

        self.posX = position.x
        self.posY = position.y
        self.posZ = position.z
        self.oriX = orientation.x
        self.oriY = orientation.y
        self.oriZ = orientation.z
        self.oriW = orientation.w
        quaternion = (self.oriX, self.oriY, self.oriZ, self.oriW)
        self.euler = tf.transformations.euler_from_quaternion(quaternion)
        print quaternion
        print 'Euler [0] ' + str(self.euler[0]) + ' Euler [1] ' + str(self.euler[1]) + ' Euler [2] ' + str(self.euler[2])
        """
        La componente z de position nos indica la distancia en metros que hay entre
        la camara y el tag.

        if self.nMuestra < 10:
            self.posX = self.posX + position.x
            self.posY = self.posY + position.y
            self.posZ = self.posZ + position.z
            self.oriX = self.oriX + orientation.x
            self.oriY = self.oriY + orientation.y
            self.oriZ = self.oriZ + orientation.z
            self.oriW = self.oriW + orientation.w
            self.nMuestra = self.nMuestra + 1
        elif self.nMuestra == 10:
            self.posX = self.posX/self.nMuestra
            self.posY = self.posY/self.nMuestra
            self.posZ = self.posZ/self.nMuestra
            self.oriX = self.oriX/self.nMuestra
            self.oriY = self.oriY/self.nMuestra
            self.oriZ = self.oriZ/self.nMuestra
            self.oriW = self.oriW/self.nMuestra
            self.nMuestra = self.nMuestra + 1
        else:
            self.nMuestra = 0
            print 'Position x ' + str(self.posX) + ' y ' + str(self.posY) + ' z ' + str(self.posZ)
            quaternion = (self.oriX, self.oriY, self.oriZ, self.oriW)
            self.euler = tf.transformations.euler_from_quaternion(quaternion)

            print 'roll ' + str(self.euler[0]) + ' pitch ' + str(self.euler[1]) + ' yaw ' + str(self.euler[2])
    """

    """
    def calDist(self, tam):
        return 105.760431653/tam

    def calAng(self, tam):
        return 105.760431653*90/tam

    def pointsCallback(self, points):
        tam = math.sqrt(math.pow((points.x1 - points.x2), 2) + math.pow((points.y1 - points.y2), 2))
        self.distancia = self.calDist(tam)
        self.ang = self.calAng(tam)
        #print "Distancia al tag " + str(self.distancia) + " angulo " + str(self.ang) + ' tam lado tag ' + str(tam)
    """

    def centerCallback(self, center):
        if self.img != None:
            height, width, depth = self.img.shape
            umbral = 20
            if (center.y <= umbral + height/2) and (center.y > height/2 - umbral):
                self.controller.SetCommand(0, 0, 0, 0)
            elif center.y < height/2 + umbral:
                self.controller.SetCommand(0, 0, 0, 1)
            elif center.y > umbral + height/2:
                self.controller.SetCommand(0, 0, 0, -1)



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
            font = cv2.FONT_HERSHEY_SIMPLEX
            height, width, depth = self.img.shape
            self.dibujaMira(width, height, self.img)
            cv2.putText(self.img,'Angulo: ' + "%0.2f" % self.navdata.rotZ ,(0,25), font, 0.75,(255,255,255),1,255)
            """
            if (self.euler != None):
                cv2.putText(self.img,'Euler[0]: ' + "%0.2f" % self.euler[0] ,(0,25), font, 0.75,(255,255,255),1,255)
                cv2.putText(self.img,'Euler[1]: ' + "%0.2f" % self.euler[1] ,(0,50), font, 0.75,(255,255,255),1,255)
                cv2.putText(self.img,'Euler[2]: ' + "%0.2f" % self.euler[2] ,(0,75), font, 0.75,(255,255,255),1,255)
                cv2.putText(self.img,'Distancia: ' + "%0.2f" % self.posZ ,(0,100), font, 0.75,(255,255,255),1,255)
            """
            """
            if self.navdata != None:
                self.virtHorz(self.navdata.rotX, self.navdata.rotY, self.navdata.rotZ)
            """
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
