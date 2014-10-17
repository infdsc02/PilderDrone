#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32


class getDistance:

    def __init__(self):
        self.distAct = 1
        self.distAnt = 0
        self.pub = rospy.Publisher('distance', Float32, queue_size=10)
        self.r = rospy.Rate(10) # 10hz

    def talker(self):
        while not rospy.is_shutdown():
            self.distAnt = self.distAct
            self.distAct = float(input('Introduce distancia:'))
            if self.distAct != self.distAnt:
                self.pub.publish(self.distAct)
            self.r.sleep()

if __name__ == '__main__':
    rospy.init_node('getDistance', anonymous=True)
    dist = getDistance()
    try:
        dist.talker()
    except rospy.ROSInterruptException: pass