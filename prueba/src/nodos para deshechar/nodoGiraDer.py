#!/usr/bin/env python
import roslib
roslib.load_manifest('prueba')
import sys
import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from drone_controller import BasicDroneController


class GiraDer:

    def __init__(self):
        self.controller = BasicDroneController()
        #Subscribers
        self.giraDer_sub = rospy.Subscriber('GiraDer',Empty,self.callback)

    def callback(self,data):
        rotZ = self.controller.getRotZ()
        rotZ_dest = rotZ - 90
        girando = True
        while girando:
                rotZ = self.controller.getRotZ()
                if (rotZ > rotZ_dest):
                    if self.controller.getYawVelocity() >= 0:
                        self.controller.SetCommand(0, 0, -1, 0)
                else:
                    girando = False
                    self.controller.SetCommand(0, 0, 0, 0)

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
