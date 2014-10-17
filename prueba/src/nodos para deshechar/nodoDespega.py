#!/usr/bin/env python
import roslib
roslib.load_manifest('prueba')
import sys
import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from drone_controller import BasicDroneController


class Despega:

    def __init__(self):
        self.controller = BasicDroneController()
        #Subscribers
        self.despega_sub = rospy.Subscriber('Despega',Empty,self.callback)

    def callback(self,data):
        self.controller.SendTakeoff()

def main(args):
    rospy.init_node('nodoDespega', anonymous=True)
    despega = Despega()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    #cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
