#!/usr/bin/env python
import roslib
roslib.load_manifest('prueba')
import sys
import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from drone_controller import BasicDroneController


class Aterriza:

    def __init__(self):
        self.controller = BasicDroneController()
        #Subscribers
        self.aterriza_sub = rospy.Subscriber('Aterriza',Empty,self.callback)

    def callback(self,data):
        self.controller.SendLand()

def main(args):
    rospy.init_node('nodoAterriza', anonymous=True)
    aterriza = Aterriza()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    #cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
