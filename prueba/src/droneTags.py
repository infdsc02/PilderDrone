#!/usr/bin/env python
import roslib
roslib.load_manifest('prueba')
import sys
import rospy
import cv2
import cv2.cv as cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import Image as Img
import math
import os
from geometry_msgs.msg import Point
from aruco_ros.msg import markerId
from cv_bridge import CvBridge, CvBridgeError
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

        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.zVelocity = 0

        self.cameraHeight = 0
        self.cameraWidth = 0

        self.command = Twist()


        #Subscribers
        self.image_sub = rospy.Subscriber('/ardrone/bottom/image_raw',Image,self.imageCallback)

        self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata)

        self.subCameraInfo = rospy.Subscriber('/ardrone/camera_info',CameraInfo,self.cameraInfoCallback)


    def cameraInfoCallback(self, info):
        self.cameraHeight = info.height
        self.cameraWidth = info.width

    def ReceiveNavdata(self,navdata):
        self.navdata = navdata

    def calculaCoords(self, x1, y1):
        x = int(self.cameraWidth*x1/1000)
        y = int(self.cameraHeight*y1/1000)

        return (x, y)

    def corregirX(self, x):
        umbral = 0
        if x > self.cameraWidth + umbral:
            self.ordenes.despLateralIzq()
        elif x < self.cameraWidth - umbral:
            self.ordenes.despLateralDer()
        elif x < self.cameraWidth + umbral and x > self.cameraWidth - umbral:
            self.ordenes.paraDespLateral()

    def corregirY(self, y):
        umbral = 0
        if y > self.cameraHeight + umbral:
            self.ordenes.avanza(0.15)
        elif y < self.cameraHeight - umbral:
            self.ordenes.retrocede(0.15)
        elif y < self.cameraHeight + umbral and y > self.cameraHeight - umbral:
            self.ordenes.paraPitch()

    def corregirAngulo(self, angulo):
        umbral = 10
        if (angulo < 270 + umbral) and angulo > 270 - umbral:
            self.ordenes.paraYaw()
        elif angulo > 270 + umbral:
            self.ordenes.yawDer()
        elif angulo < 270 - umbral:
            self.ordenes.yawIzq()

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

            auxNavdata = self.navdata

           # print 'height ' + str(self.cameraHeight) + ' width ' + str(self.cameraWidth)
            self.dibujaMira(width, height, self.img)
            font = cv2.FONT_HERSHEY_SIMPLEX
            if auxNavdata != None:
                cv2.putText(self.img,'Tags Detected ' + str(auxNavdata.tags_count) ,(0,25), font, 0.5,(255,255,255),1,255)
                if auxNavdata.tags_count != 0:
                    cv2.putText(self.img,'Tag Type ' + str(auxNavdata.tags_type[0]) ,(0,50), font, 0.5,(255,255,255),1,255)
                    cv2.putText(self.img,'Tag xc ' + str(auxNavdata.tags_xc[0]) + ' Tag yc ' + str(auxNavdata.tags_yc[0]),(0,75), font, 0.5,(255,255,255),1,255)
                    pos = self.calculaCoords(auxNavdata.tags_xc[0], auxNavdata.tags_yc[0])
                    self.corregirX(pos[0])
                    self.corregirY(pos[1])
                    #self.corregirAngulo(auxNavdata.tags_orientation[0])
                    cv2.putText(self.img,'Tag x pixels ' + str(pos[0]) + ' Tag y pixels ' + str(pos[1]),(0,100), font, 0.5,(255,255,255),1,255)
                    cv2.putText(self.img,'Tag orientation ' + "%0.2f" % auxNavdata.tags_orientation[0], (0,125), font, 0.5,(255,255,255),1,255)
                    cv2.putText(self.img,'Roll ' + str(self.roll), (0,150), font, 0.5,(255,255,255),1,255)
                else:
                    self.ordenes.para()
            cv2.imshow("Image window", self.img)
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
