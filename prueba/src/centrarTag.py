#!/usr/bin/env python
import roslib
roslib.load_manifest('prueba')
import sys
import rospy
import cv2
import cv2.cv as cv
from sensor_msgs.msg import Image
import Image as Img
import os
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from ardrone_autonomy.msg import Navdata
from std_msgs.msg import Empty

class centrarTag:

    def __init__(self):
        self.rate = rospy.Rate(10) #10Hz
        self.bridge = CvBridge()
        self.img = None
        self.posX = 0
        self.posY = 0
        self.posZ = 0
        self.oriX = 0
        self.oriY = 0
        self.oriZ = 0
        self.oriW = 0
        self.navdata = None
        self.centered = False
        self.markerDetected = False
        self.center = None

        #Subscribers
        self.image_sub = rospy.Subscriber('/aruco_single/result',Image,self.imageCallback)
        self.marker_center_sub = rospy.Subscriber('/aruco_single/markerCenter',Point,self.centerCallback)
        self.marker_pose_sub = rospy.Subscriber('/aruco_single/pose',PoseStamped,self.poseCallback)
        self.marker_detected_sub = rospy.Subscriber('/aruco_single/markerDetected',Bool,self.markerDetectedCallback)
        self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata)

        #Publishers
        self.izq_pub = rospy.Publisher('despIzq', Empty)
        self.der_pub = rospy.Publisher('despDer', Empty)
        self.arriba_pub = rospy.Publisher('arriba', Empty)
        self.arribaDer_pub = rospy.Publisher('arribaDer', Empty)
        self.arribaIzq_pub = rospy.Publisher('arribaIzq', Empty)
        self.abajo_pub = rospy.Publisher('abajo', Empty)
        self.abajoDer_pub = rospy.Publisher('abajoDer', Empty)
        self.abajoIzq_pub = rospy.Publisher('abajoIzq', Empty)
        self.para_pub = rospy.Publisher('paraCentrar', Empty)
        self.centrado_pub = rospy.Publisher('centrado', Empty)

    def markerDetectedCallback(self, detected):
        self.markerDetected = detected.data

    def ReceiveNavdata(self,navdata):
        self.navdata = navdata

    def poseCallback(self, poseS):
        pose = poseS.pose
        position = pose.position
        self.posX = position.x
        self.posY = position.y
        self.posZ = position.z


    def moveToCenter(self, centerPos):
        height, width, depth = self.img.shape
        umbral = 50
        if ((centerPos.y <= umbral + height/2) and (centerPos.y > height/2 - umbral)) and ((centerPos.x <= umbral + width/2) and (centerPos.x > width/2 - umbral)):
            self.centrado_pub.publish(Empty())
        elif (centerPos.y > umbral + height/2) and (centerPos.x > umbral + width/2):
            self.abajoDer_pub.publish(Empty())
        elif (centerPos.y > umbral + height/2) and (centerPos.x < width/2 - umbral):
            self.abajoIzq_pub.publish(Empty())
        elif (centerPos.y < height/2 - umbral) and (centerPos.x > umbral + width/2):
            self.arribaDer_pub.publish(Empty())
        elif (centerPos.y < height/2 - umbral) and (centerPos.x < width/2 - umbral):
            self.arribaIzq_pub.publish(Empty())
        elif (centerPos.y > umbral + height/2):
            self.abajo_pub.publish(Empty())
        elif (centerPos.y < height/2 - umbral):
            self.arriba_pub.publish(Empty())
        elif (centerPos.x > umbral + width/2):
            self.der_pub.publish(Empty())
        elif (centerPos.x < width/2 - umbral):
            self.izq_pub.publish(Empty())

    def centerCallback(self, center):
        self.center = center

    def imageCallback(self,data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            if self.markerDetected:
                self.moveToCenter(self.center)
            else:
                self.para_pub.publish(Empty())
        except CvBridgeError, e:
            print e



def main(args):
    rospy.init_node('centrarTag', anonymous=True)
    ct = centrarTag()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    #cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
