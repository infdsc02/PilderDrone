#!/usr/bin/env python
import roslib
roslib.load_manifest('prueba')
import sys
import rospy
import cv2
import cv2.cv as cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
import Image as Img
import math
import os
import pygame
from geometry_msgs.msg import PoseStamped
from aruco_ros.msg import markerId
from cv_bridge import CvBridge, CvBridgeError
import constantes
import numpy as np
from ordenes import Ordenes

class lector:

    def __init__(self):
        self.rate = rospy.Rate(10) #10Hz
        self.sonido1 = '/home/david/proyecto/a.wav'
        self.sonido2 = '/home/david/proyecto/blaster.wav'
        self.contImg = 0
        cv2.namedWindow("Image window", 1)
        self.bridge = CvBridge()
        self.ordenes = Ordenes()
        self.accion_act = None
        self.accion_ant = None

        """
        Para usar la camara de abajo cambiar '/ardrone/image_raw' por
        '/ardrone/botton/image_raw' luego antes de ejecutar el nodo
        ejecutar el comando rosservice call /ardrone/togglecam
        """
        #Subscribers
        self.image_sub = rospy.Subscriber('/aruco_single/result',Image,self.imageCallback)

        self.aruco_sub = rospy.Subscriber('/aruco_single/markerId',markerId,self.idCallback)

       # self.drone_image_sub = rospy.Subscriber('/ardrone/image_raw',markerId,self.detectColor)

    def idCallback(self, data):
        self.accion_act = data.markerId
        if self.accion_ant != self.accion_act:
            self.parser(data)
        self.accion_ant = self.accion_act

    def parser(self, data):
        if data == constantes.MARGEN_IZQ:
            print "ME DESPLAZO LATERALMENTE HACIA LA DERECHA"
            self.ordenes.despLateralDer()
        elif data == constantes.MARGEN_DER:
            print "ME DESPLAZO LATERALMENTE HACIA LA IZQUIERDA"
            self.ordenes.despLateralIzq()


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

    def grabarImagenes(self, img, the):
            if self.contImg < 10:
                cv2.imwrite('salida_000'+str(self.contImg)+'.jpeg', img)
            elif self.contImg < 100:
                cv2.imwrite('salida_00'+str(self.contImg)+'.jpeg', img)
            elif self.contImg < 1000:
                cv2.imwrite('salida_0'+str(self.contImg)+'.jpeg', img)
            else:
                cv2.imwrite('salida_'+str(self.contImg)+'.jpeg', img)

            if self.contImg < 10:
                cv2.imwrite('canny_000'+str(self.contImg)+'.jpeg', the)
            elif self.contImg < 100:
                cv2.imwrite('canny_00'+str(self.contImg)+'.jpeg', the)
            elif self.contImg < 1000:
                cv2.imwrite('canny_0'+str(self.contImg)+'.jpeg', the)
            else:
                cv2.imwrite('canny_'+str(self.contImg)+'.jpeg', the)

            self.contImg = self.contImg + 1


    def dibujaMira(self, width, height, img):
        cv2.line(img,(0,height/2),(width,height/2),(0,0,255),1)
        cv2.line(img,(width/2,0),(width/2,height),(0,0,255),1)
        cv2.circle(img,(width/2,height/2), 4, (0,0,255), 1)
        cv2.circle(img,(width/2,height/2), 10, (0,0,255), 1)
        cv2.circle(img,(width/2,height/2), 20, (0,0,255), 1)
        cv2.circle(img,(width/2,height/2), 30, (0,0,255), 1)
        cv2.circle(img,(width/2,height/2), 40, (0,0,255), 1)

    def detectaContornos(self, bordes, img_orig):
        contours, hierarchy = cv2.findContours(bordes,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(img_orig,contours,-1,(0,255,0),3)

    def mensajePantalla(self, img, msg, x, y):
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img,msg,(x,y), font, 0.5,(255,0,255),2)


    def detectaLinea(self, bordes, img_orig):
        minLineLength = 20
        maxLineGap = 4
        lines = cv2.HoughLinesP(bordes,1,np.pi/180,100,minLineLength,maxLineGap)
        ang = None
        puntoMedio = None
        if lines != None:
            line = self.maxLine(lines)
            ang = self.angulo(line[0],line[1],line[2],line[3])
            puntoMedio = ((line[2]-line[0])/2, (line[3]-line[1])/2)


            cv2.line(img_orig,(line[0],line[1]),(line[2],line[3]),(255,0,0),3)
        return (ang, puntoMedio)

    def imageCallback(self,data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

            height, width, depth = img.shape
            thresh = 125
            ret,the = cv2.threshold(gray,thresh,220,cv2.THRESH_BINARY)
            self.detectaContornos(the, img)


            result = self.detectaLinea(the, img)
            if result[0] != None and result[1] != None:
                ang = result[0]
                xMedio = result[1][0]
                umbral = 5
                distanciaCentro = xMedio# - width/2
                if math.fabs(distanciaCentro) > umbral:
                    if distanciaCentro > 0:
                        self.mensajePantalla(the, str(distanciaCentro), width/2, height/2)
                    elif distanciaCentro < 0:
                        self.mensajePantalla(the, str(distanciaCentro), width/2, height/2)
                else:
                    self.mensajePantalla(the, "PARA", width/2, height/2)




            self.dibujaMira(width, height, img)

        except CvBridgeError, e:
            print e
        cv2.imshow("Threshold window", the)
        cv2.imshow("Image window", img)
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
