#!/usr/bin/env python
import roslib
roslib.load_manifest('prueba')
import sys
import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from drone_controller import BasicDroneController


class GiraIzq:

    def __init__(self):
        self.controller = BasicDroneController()
        #Subscribers
        self.giraIzq_sub = rospy.Subscriber('GiraIzq',Empty,self.callback)

    def callback(self,data):

        rotZ = self.controller.getRotZ()
        rotZ_dest = rotZ + 90
        #self.controller.writeLog('rotZ_ini ' + str(rotZ))
        #self.controller.writeLog('rotZ_dest ' + str(rotZ_dest))
        girando = True
        while girando:
                rotZ = self.controller.getRotZ()
                if (rotZ < rotZ_dest):
                    #self.controller.writeLog('rotZ ' + str(rotZ))

                    if self.controller.getYawVelocity() <= 0:
                        self.controller.SetCommand(0, 0, 1, 0)

                else:
                    girando = False
                    #self.controller.writeLog('LLEGAMOS')
                    self.controller.SetCommand(0, 0, 0, 0)

"""
                elif (rotZ > rotZ_dest+5):
                    if self.controller.getYawVelocity() >= 0:
                        self.controller.SetCommand(0, 0, -1, 0)
"""


def main(args):
    rospy.init_node('nodoGiraIzq', anonymous=True)
    giraIzq = GiraIzq()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    #cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
