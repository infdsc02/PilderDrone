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
        self.aruco_sub = rospy.Subscriber('/aruco_single/markerId',markerId,self.idCallback)

    def parser(self, data):
        if data == constantes.DESPEGA:
            self.ordenes.despega()
        elif data == constantes.ATERRIZA:
            self.ordenes.aterriza()
        elif data == constantes.GIRA_IZQ:
            print 'giraIzq'
            self.ordenes.gira90Izq()
        elif data == constantes.GIRA_DER:
            self.ordenes.gira90Der()
        elif data == constantes.GIRA_180:
            self.ordenes.gira180()
        elif data == constantes.SUBE:
            self.ordenes.sube(10)
        elif data == constantes.BAJA:
            self.ordenes.baja(10)
        elif data == constantes.AVANZA:
            self.ordenes.avanza()
        elif data == constantes.NONE:
            self.ordenes.para()

    def idCallback(self, data):
        self.accion_act = data.markerId
        if self.accion_ant != self.accion_act:
            self.parser(self.accion_act)
        self.accion_ant = self.accion_act

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
