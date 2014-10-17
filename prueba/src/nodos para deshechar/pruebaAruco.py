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
from geometry_msgs.msg import PoseStamped, Vector3Stamped, Pose, Point, Quaternion

from ordenes import Ordenes

class lector:

    def __init__(self):
        self.ordenes = Ordenes()
        self.accion_ant = None
        self.accion_act = None
        cv2.namedWindow("Image window", 1)
        self.bridge = CvBridge()

        #Subscribers
        self.image_sub = rospy.Subscriber('/aruco_single/result',Image,self.imageCallback)
        self.pose_sub = rospy.Subscriber('/aruco_single/pose',PoseStamped,self.poseCallback)
        #self.position_sub = rospy.Subscriber('/aruco_single/position',Vector3Stamped,self.positionCallback)

    def poseCallback(self, data):
        pose = data.pose
        position = pose.position
        orientation = pose.orientation

        print "Position: x=" + str(position.x) + " y=" + str(position.y) + " z=" + str(position.z)
        print "Orientation: x=" + str(orientation.x) + " y="  + str(orientation.y) + " z="  + str(orientation.z) + " w=" +  + str(orientation.w)


    def imageCallback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError, e:
            print e
        cv2.imshow("Image window", cv_image)
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
