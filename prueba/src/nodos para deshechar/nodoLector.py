#!/usr/bin/env python
import roslib
roslib.load_manifest('prueba')
import sys
import rospy
import cv2
import cv2.cv as cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import zbar
import Image as Img

class lector:

    def __init__(self):
        self.accion_ant = None
        self.accion_act = None
        self.scanner = zbar.ImageScanner()
        self.scanner.parse_config('enable')
        cv2.namedWindow("Image window", 1)
        self.bridge = CvBridge()

        """
        Para usar la camara de abajo cambiar '/ardrone/image_raw' por
        '/ardrone/botton/image_raw' luego antes de ejecutar el nodo
        ejecutar el comando rosservice call /ardrone/togglecam
        """
        #Subscribers
        self.image_sub = rospy.Subscriber('/ardrone/image_raw',Image,self.callback)

        #Publishers
        self.orden_pub = rospy.Publisher('Orden', String, queue_size=10)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            # converts numpy.ndarray to ipl image
            """
            bitmap = cv.CreateImageHeader((cv_image.shape[1], cv_image.shape[0]), cv.IPL_DEPTH_8U, 3)
            cv.SetData(bitmap, cv_image.tostring(), cv_image.dtype.itemsize * 3 * cv_image.shape[1])
            """

            cv2.imwrite('salida.png', cv_image)
            pil = Img.open('salida.png').convert('L')

            raw = pil.tostring()
            image = zbar.Image(cv_image.shape[1], cv_image.shape[0], 'Y800', raw)

            self.scanner.scan(image)

            # extract results
            for symbol in image:
                self.accion_act = symbol.data
                if self.accion_act != self.accion_ant:
                    info = 'Symbol ' + symbol.data
                    self.orden_pub.publish(symbol.data)
                    self.accion_ant = self.accion_act
            del(image)
        except CvBridgeError, e:
            print e

      #  cv2.imshow("Image window", cv_image)
      #  cv2.waitKey(3)

def main(args):
    rospy.init_node('nodoLector', anonymous=True)
    ic = lector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    #cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
