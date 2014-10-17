#!/usr/bin/env python
import roslib
roslib.load_manifest('prueba')
import sys
import rospy
import cv2
import cv2.cv as cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import Image as Img
from cv_bridge import CvBridge, CvBridgeError
import yaml

class imgArectificar:

    def __init__(self):

        cv2.namedWindow("Image window", 1)
        self.bridge = CvBridge()
        self.cameraInfo = CameraInfo()
        self. pubImg = rospy.Publisher('/imgArectificar/image_raw',Image,100)
        self. pubCamInfo = rospy.Publisher('/imgArectificar/camera_info',CameraInfo,100)

    def sendImg(self):
        stream = open("/home/david/.ros/camera_info/ardrone_front.yaml", "r")
        docs = yaml.load_all(stream)
        for doc in docs:
            for k,v in doc.items():
                print k, "->", v
                if k == 'image_width':
                    self.cameraInfo.width = int(v)
                elif k == 'image_height':
                    self.cameraInfo.height = int(v)
            print "\n",
        """
        i = 0
        while i <= 1273:
            if i < 10:
                foto = '/home/david/proyecto/fotos/planta_0/salida_000'+str(i)+'.jpeg'
            elif i < 100:
                foto = '/home/david/proyecto/fotos/planta_0/salida_00'+str(i)+'.jpeg'
            elif i < 1000:
                foto = '/home/david/proyecto/fotos/planta_0/salida_0'+str(i)+'.jpeg'
            else:
                foto = '/home/david/proyecto/fotos/planta_0/salida_'+str(i)+'.jpeg'
            print foto
            im = cv2.imread(foto)

            imgMsg = self.bridge.cv2_to_imgmsg(im, "passthrough")
            """


def main(args):
    rospy.init_node('imgArectificar', anonymous=True)
    im = imgArectificar()
    im.sendImg()

if __name__ == '__main__':
    main(sys.argv)