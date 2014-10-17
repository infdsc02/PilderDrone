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
import tf
import math

class anguloTag:

    def __init__(self):
        self.rate = rospy.Rate(10) #10Hz
        self.bridge = CvBridge()
        self.img = None
        self.navdata = None
        self.markerDetected = False
        self.euler = None

        #Subscribers
        self.image_sub = rospy.Subscriber('/aruco_single/result',Image,self.imageCallback)
        self.marker_pose_sub = rospy.Subscriber('/aruco_single/pose',PoseStamped,self.poseCallback)
        self.marker_detected_sub = rospy.Subscriber('/aruco_single/markerDetected',Bool,self.markerDetectedCallback)
        self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata)

        #Publishers
        self.yaw_izq_pub = rospy.Publisher('yawIzq', Empty)
        self.yaw_der_pub = rospy.Publisher('yawDer', Empty)
        self.yaw_para_pub = rospy.Publisher('paraYaw', Empty)
        self.yaw_correcto_pub = rospy.Publisher('yawCorrecto', Empty)

    def markerDetectedCallback(self, detected):
        self.markerDetected = detected.data

    def ReceiveNavdata(self,navdata):
        self.navdata = navdata

    def poseCallback(self, poseS):
        pose = poseS.pose
        orientation = pose.orientation

        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        self.euler = tf.transformations.euler_from_quaternion(quaternion)
        print 'Euler [0] ' + str(self.euler[0]) + ' Euler [1] ' + str(self.euler[1]) + ' Euler [2] ' + str(self.euler[2])

    def corregirYaw(self, euler):
        umbral = 10
        ang = (-1)*euler[0]*180/math.pi #pasamos el angulo de radianes a grados

        if ang < 90 + umbral and ang > 90 - umbral:
            self.yaw_correcto_pub.publish(Empty())
        elif ang < 90 - umbral:
            self.yaw_izq_pub.publish(Empty())
        elif ang > 90 + umbral:
            self.yaw_der_pub.publish(Empty())



    def imageCallback(self,data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            if self.markerDetected:
                self.corregirYaw(self.euler)
            else:
                self.yaw_para_pub.publish(Empty())
        except CvBridgeError, e:
            print e



def main(args):
    rospy.init_node('anguloTag', anonymous=True)
    ct = anguloTag()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    #cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
