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
import numpy as np
import math
import pygame
from ordenes import Ordenes
from geometry_msgs.msg import Vector3

class lector:

    def __init__(self):
        self.ordenes = Ordenes()
        self.accion_ant = None
        self.accion_act = None
        cv2.namedWindow("Image window", 1)
        self.bridge = CvBridge()
        self.ordenes = Ordenes()
        self.sonido1 = '/home/david/proyecto/a.wav'
        self.sonido2 = '/home/david/proyecto/blaster.wav'
        #Subscribers
        self.image_sub = rospy.Subscriber('/aruco_single/result',Image,self.imageCallback)

        self.pubGiro    = rospy.Publisher('corregirGiro',Vector3,queue_size=1)
        self.contLLamadas = 0

    def sonido(self, sonido):
        pygame.mixer.init()
        sound = pygame.mixer.Sound(sonido)
        sound.play()

    def angulo(self, x1, y1, x2, y2):
        if x1 != x2:
            return math.degrees(math.atan((y2-y1)/(x2-x1)))
        else:
            return 90

    def maxMinXY(self, contours):
        if len(contours) > 0:
            i = 1
            x_max = contours[0][0][0][0]
            x_min = contours[0][0][0][0]
            y_max = contours[0][0][0][1]
            y_min = contours[0][0][0][1]
            while i < len(contours[0]):
                if x_max < contours[0][i][0][0]:
                    x_max = contours[0][i][0][0]
                if x_min > contours[0][i][0][0]:
                    x_min = contours[0][i][0][0]
                if y_max < contours[0][i][0][1]:
                    y_max = contours[0][i][0][1]
                if y_min > contours[0][i][0][1]:
                    y_min = contours[0][i][0][1]
                i = i + 1
            return (x_max, x_min, y_max, y_min)
        else:
            return None

    def longSegment(self, x1, x2, y1, y2):
        return math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))

    def maxLine(self, lines):
        max_long = -1
        line = None
        if len(lines) > 0:
            for x1,y1,x2,y2 in lines[0]:
                longSe = self.longSegment(x1, x2, y1, y2)
                if  longSe > max_long:
                    max_long = longSe
                    line = (x1, y1, x2, y2)
        return line

    def calculaGiro(self, ang):
        return 90 - abs(ang)

    def procesarImagen(self, img):
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        height, width, depth = img.shape
        ret,the = cv2.threshold(gray,180,255,cv2.THRESH_BINARY)
        cv2.line(img,(0,height/2),(width,height/2),(0,0,255),1)
        cv2.line(img,(width/2,0),(width/2,height),(0,0,255),1)
        contours, hierarchy = cv2.findContours(the,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        minLineLength = 10
        maxLineGap = 5
        lines = cv2.HoughLinesP(the,1,np.pi/180,100,minLineLength,maxLineGap)
        ang = 0
        giro = Vector3()
        if lines != None:
            line = self.maxLine(lines)
            ang = self.angulo(line[0],line[1],line[2],line[3])
            #print "angulo " + str(ang)
            giro.x = self.calculaGiro(ang)
            #print "Grado que tiene que girar " + str(giro.x)
            """
            if giro.x != 0 and not self.ordenes.estaGirando():
                #self.pubGiro.publish(giro)
                self.contLLamadas = self.contLLamadas + 1
                self.ordenes.controller.writeLog("CONTADOR " + str(self.contLLamadas))
                self.ordenes.corregirYaw(giro.x)
            """
            cv2.line(img,(line[0],line[1]),(line[2],line[3]),(255,0,0),3)

        cv2.circle(img,(width/2,height/2), 4, (0,0,255), 1)
        cv2.circle(img,(width/2,height/2), 10, (0,0,255), 1)
        cv2.circle(img,(width/2,height/2), 20, (0,0,255), 1)
        cv2.circle(img,(width/2,height/2), 30, (0,0,255), 1)
        cv2.circle(img,(width/2,height/2), 40, (0,0,255), 1)

        cv2.imshow("Threshold window", the)

    def imageCallback(self,data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.procesarImagen(img)
            cv2.imshow("Image window", img)
            cv2.waitKey(3)
        except CvBridgeError, e:
            print e




def main(args):
    rospy.init_node('lector', anonymous=True)
    ic = lector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        ic.ordenes.aterriza()
        print "Shutting down"
    #cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)