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
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import Bool

class distanciaTag:

    def __init__(self):
        self.rate = rospy.Rate(10) #10Hz
        self.bridge = CvBridge()
        self.img = None
        self.distAct = 2
        self.markerDetected = False

        #Subscribers
        self.image_sub = rospy.Subscriber('/aruco_single/result',Image,self.imageCallback)

        self.marker_pose_sub = rospy.Subscriber('/aruco_single/pose',PoseStamped,self.distCallback)
        self.dist_sub = rospy.Subscriber('distance',Float32,self.distActCallback)
        self.marker_detected_sub = rospy.Subscriber('/aruco_single/markerDetected',Bool,self.markerDetectedCallback)

        #Publishers
        self.avanza_pub = rospy.Publisher('avanza', Empty)
        self.retrocede_pub = rospy.Publisher('retrocede', Empty)
        self.distCorrect_pub = rospy.Publisher('distCorrecta', Empty)
        self.para_pub = rospy.Publisher('paraPitch', Empty)


    def markerDetectedCallback(self, detected):
        self.markerDetected = detected.data

    def distActCallback(self, data):
        self.distAct = data.data


    def distance(self, distance):
        umbral = 0.1 #10 cm
        print 'Distancia ' + str(distance)
        if (distance < self.distAct - umbral):
            self.retrocede_pub.publish(Empty())
            print 'Atras'
        elif (distance > self.distAct + umbral):
            self.avanza_pub.publish(Empty())
            print 'Adelante'
        elif (distance > self.distAct - umbral) and (distance < self.distAct + umbral):
            self.distCorrect_pub.publish(Empty())
            print 'Distancia correcta'


    def distCallback(self, poseS):
        pose = poseS.pose
        position = pose.position

        if self.img != None:
            self.distance(position.z)

    def imageCallback(self,data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")

            if not self.markerDetected:
                self.para_pub.publish(Empty())


        except CvBridgeError, e:
            print e



def main(args):
    rospy.init_node('distanciaTag', anonymous=True)
    dt = distanciaTag()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    #cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
