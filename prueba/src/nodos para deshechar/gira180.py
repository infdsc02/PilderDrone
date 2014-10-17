#!/usr/bin/env python
import roslib
roslib.load_manifest('prueba')
import sys
import rospy
from std_msgs.msg import Empty
from ardrone_autonomy.msg import Navdata

class gira180:

    def __init__(self):
        self.rate = rospy.Rate(10) #10Hz

        self.nVueltas = 0
        self.primeraVuelta = True
        self.rotZ_ant = 0

        self.navdata = None

        self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata)
        self.gira_pub = rospy.Publisher('gira180', Empty)
        self.para_pub = rospy.Publisher('paraGiro180', Empty)


    def ReceiveNavdata(self,navdata):
        self.navdata = navdata

    def contVueltas (self, rotZ_act):
        if not self.primeraVuelta:
            if self.rotZ_ant < 0 and rotZ_act >= 0 and self.rotZ_ant > -90:
                self.nVueltas = self.nVueltas + 1
            elif self.rotZ_ant >= 0 and rotZ_act < 0  and self.rotZ_ant < 90:
                self.nVueltas = self.nVueltas - 1
        else:
            self.primeraVuelta = False

        self.rotZ_ant = rotZ_act

    def getRotZ(self):
        rotZ_real = self.navdata.rotZ
        self.contVueltas(rotZ_real)
        if rotZ_real >= 0:
            rotZ_calculado = rotZ_real + 360*self.nVueltas
        else:
            rotZ_calculado = rotZ_real + 360*(self.nVueltas+1)

        return rotZ_calculado

    def gira180(self):
        rotZ = self.getRotZ()
        rotZ_dest = rotZ + 160
        self.girando = True
        while self.girando:
                rotZ = self.getRotZ()
                if (rotZ < rotZ_dest):
                    self.gira_pub.publish(Empty())
                else:
                    self.girando = False
                    self.para_pub.publish(Empty())



def main(args):
    rospy.init_node('gira180', anonymous=True)
    g = gira180()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    #cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
