#!/usr/bin/env python
import roslib
roslib.load_manifest('prueba')
import sys
import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from ordenes import Ordenes
from geometry_msgs.msg import Vector3

class GiraDer:

    def __init__(self):
        self.ordenes = Ordenes()
        #Subscribers
        self.giraDer_sub = rospy.Subscriber('corregirGiro',Vector3,self.callback)

    def callback(self,data):
        angulo = data.x
        if not self.ordenes.estaGirando():
            self.ordenes.corregirYaw(angulo)

def main(args):
    rospy.init_node('nodoGiraDer', anonymous=True)
    giraDer = GiraDer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    #cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
