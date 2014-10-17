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
from geometry_msgs.msg import Point
from aruco_ros.msg import markerId
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from ordenes import Ordenes
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from drone_status import DroneStatus
from drone_controller import BasicDroneController
from aruco_ros.msg import points
from geometry_msgs.msg import PoseStamped
from ardrone_autonomy.msg import Navdata
import tf
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import time

class Piloto:

    def __init__(self):

        self.DEBUG = True

        self.rate = rospy.Rate(10) #10Hz
        self.ordenes = Ordenes()
        self.bridge = CvBridge()

        self.centrado = False
        self.alineado = False
        self.giros180 = 0

        if self.DEBUG:
            self.marker_pose_sub = rospy.Subscriber('/aruco_single/pose',PoseStamped,self.distCallback)
            self.dist = -1

        #Subscriber para la imagen
        self.image_sub = rospy.Subscriber('/aruco_single/result',Image,self.imageCallback)

        #Subscribers para el centrado

        self.arriba_sub = rospy.Subscriber('/arriba',Empty,self.arribaCallback)
        self.abajo_sub = rospy.Subscriber('/abajo',Empty,self.abajoCallback)
        self.izq_sub = rospy.Subscriber('/despIzq',Empty,self.izqCallback)
        self.der_sub = rospy.Subscriber('/despDer',Empty,self.derCallback)
        self.arribaIzq_sub = rospy.Subscriber('/arribaIzq',Empty,self.arribaIzqCallback)
        self.arribaDer_sub = rospy.Subscriber('/arribaDer',Empty,self.arribaDerCallback)
        self.abajoIzq_sub = rospy.Subscriber('/abajoIzq',Empty,self.abajoIzqCallback)
        self.abajo_sub = rospy.Subscriber('/abajoDer',Empty,self.abajoDerCallback)
        self.paraCentrar_sub = rospy.Subscriber('/paraCentrar',Empty,self.paraCentrarCallback)
        self.centrado_sub = rospy.Subscriber('/centrado',Empty,self.centradoCallback)

        #Subscribers para el avance/retroceso

        self.avanza_sub = rospy.Subscriber('/avanza',Empty,self.avanzaCallback)
        self.retrocede_sub = rospy.Subscriber('/retrocede',Empty,self.retrocedeCallback)
        self.paraPitch_sub = rospy.Subscriber('/paraPitch',Empty,self.paraPitchCallback)
        self.distCorrect_sub = rospy.Subscriber('/distCorrecta',Empty,self.distCorrectaCallback)


        #Subscribers para la correccion del Yaw
        self.yaw_izq_sub = rospy.Subscriber('/yawIzq',Empty,self.yawIzqCallback)
        self.yaw_der_sub = rospy.Subscriber('/yawDer',Empty,self.yawDerCallback)
        self.yaw_para_sub = rospy.Subscriber('/paraYaw',Empty,self.paraYawCallback)
        self.yaw_correcto_sub = rospy.Subscriber('/yawCorrecto',Empty,self.yawCorrectoCallback)

    def distCallback(self, poseS):
        self.dist = poseS.pose.position.z

    def yawIzqCallback(self, data):
        if self.centrado:
            self.alineado = False
            self.ordenes.yawIzq()

    def yawDerCallback(self, data):
        if self.centrado:
            self.alineado = False
            self.ordenes.yawDer()

    def paraYawCallback(self, data):
        self.alineado = False
        self.ordenes.paraYaw()

    def yawCorrectoCallback(self, data):
        self.alineado = True
        self.ordenes.paraYaw()

    def avanzaCallback(self, data):
        if self.alineado:
            self.ordenes.avanza(0.15)
            self.giros180 = 0
            #print 'Avanza'

    def retrocedeCallback(self, data):
        if self.alineado:
            self.ordenes.retrocede(0.15)
            self.giros180 = 0
            #print 'Retrocede'

    def paraPitchCallback(self, data):
        self.ordenes.paraPitch()
        self.giros180 = 0
        #print 'Para Pitch'

    def distCorrectaCallback(self, data):
        if self.alineado:
            self.ordenes.paraPitch()
            if self.giros180 == 0:
                self.ordenes.gira180()
                self.giros180 = self.giros180 + 1
            print 'Distancia correcta'

    def centradoCallback(self, data):
        self.ordenes.paraCentrar()
        self.centrado = True
        print 'Centrado correctamente'

    def arribaCallback(self, data):
        self.ordenes.sube()
        self.centrado = False
        print 'Arriba'

    def abajoCallback(self, data):
        self.ordenes.baja()
        self.centrado = False
        print 'Abajo'

    def izqCallback(self, data):
        self.ordenes.despLateralIzq()
        self.centrado = False
        print 'Izquierda'

    def derCallback(self, data):
        self.ordenes.despLateralDer()
        self.centrado = False
        print 'Derecha'

    def arribaIzqCallback(self, data):
        self.ordenes.subeIzq()
        self.centrado = False
        print 'Arriba Izquierda'

    def arribaDerCallback(self, data):
        self.ordenes.subeDer()
        self.centrado = False
        print 'Arriba Derecha'

    def abajoIzqCallback(self, data):
        self.ordenes.bajaIzq()
        self.centrado = False
        print 'Abajo Izquierda'

    def abajoDerCallback(self, data):
        self.ordenes.bajaDer()
        self.centrado = False
        print 'Abajo Derecha'

    def paraCentrarCallback(self, data):
        self.ordenes.paraCentrar()
        self.centrado = False
        #print 'Para de centrar'

    def dibujaMira(self, width, height, img):
        cv2.line(img,(0,height/2),(width,height/2),(0,0,255),1)
        cv2.line(img,(width/2,0),(width/2,height),(0,0,255),1)
        cv2.circle(img,(width/2,height/2), 4, (0,0,255), 1)
        cv2.circle(img,(width/2,height/2), 10, (0,0,255), 1)
        cv2.circle(img,(width/2,height/2), 20, (0,0,255), 1)
        cv2.circle(img,(width/2,height/2), 30, (0,0,255), 1)
        cv2.circle(img,(width/2,height/2), 40, (0,0,255), 1)

    def imageCallback(self,data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            height, width, depth = self.img.shape
            self.dibujaMira(width, height, self.img)

            if self.DEBUG:
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(self.img,'Dist: ' + "%0.2f" % self.dist ,(0,25), font, 0.75,(255,255,255),1,255)
                cv2.putText(self.img,'Centrado: ' + str(self.centrado) ,(0,50), font, 0.75,(255,255,255),1,255)
                #cv2.putText(self.img,'Alineado: ' + str(self.alineado) ,(0,75), font, 0.75,(255,255,255),1,255)

            cv2.imshow("Piloto", self.img)
            cv2.waitKey(3)
        except CvBridgeError, e:
            print e



def main(args):
    rospy.init_node('piloto', anonymous=True)
    piloto = Piloto()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    #cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
