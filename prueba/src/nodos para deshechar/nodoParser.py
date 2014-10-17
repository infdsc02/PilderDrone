#!/usr/bin/env python
import roslib
roslib.load_manifest('prueba')
import sys
import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty

class Parser:
    def __init__(self):
        #Subscribers
        self.orden_sub = rospy.Subscriber('Orden',String,self.callback)

        #Publishers
        self.despega_pub = rospy.Publisher('Despega', Empty)
        self.aterriza_pub = rospy.Publisher('Aterriza', Empty)
        self.giraIzq_pub = rospy.Publisher('GiraIzq', Empty)
        self.giraDer_pub = rospy.Publisher('GiraDer', Empty)
        self.gira180_pub = rospy.Publisher('Gira180', Empty)
        self.sube_pub = rospy.Publisher('Sube', Empty)
        self.baja_pub = rospy.Publisher('Baja', Empty)
        self.avanza_pub = rospy.Publisher('Avanza', Empty)

#fLog = open('salida.txt', 'w')



    """
    def writeLog(fLog, msg):
        aux = sys.stdout
        rospy.loginfo(msg)
        sys.stdout = fLog
        rospy.loginfo(msg)
        sys.stdout = aux
    """
    def callback(self, data):
        print data.data
        if data.data == 'despega':
            self.despega_pub.publish(Empty())
        elif data.data == 'aterriza':
            self.aterriza_pub.publish(Empty())
        elif data.data == 'giraIzq':
            self.giraIzq_pub.publish(Empty())
        elif data.data == 'giraDer':
            self.giraDer_pub.publish(Empty())
        elif data.data == 'gira180':
            self.gira180_pub.publish(Empty())
        elif data.data == 'avanza':
            self.avanza_pub.publish(Empty())
        """
        elif data.find('sube') != -1:
            print data.data
        elif data.find('baja') != -1:
            print data.data
        """

def main(args):
    rospy.init_node('nodoParser', anonymous=True)
    p = Parser()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    #cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
