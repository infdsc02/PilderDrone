#!/usr/bin/env python
import numpy as np
import cv2
import math

def drawHorizon(img):
    height, width, depth = img.shape
    cv2.line(img,(0,height/2 - 30),(width,height/2 - 30),(0,0,255),1)

def distPuntos(x1, y1, x2, y2):
    dist = math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2))

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
"""
cap = cv2.VideoCapture(1)
#cap = cv2.VideoCapture('/home/david/proyecto/grabacion_con_filtro_ir_claro.avi')
ret, im = cap.read()
height, width, depth = im.shape

fourcc = cv2.cv.CV_FOURCC(*'XVID')
video = cv2.VideoWriter('output_filtro_color_medio.avi',fourcc, 20.0, (width,height))

#cap = cv2.VideoCapture(1)
while(cap.isOpened() or im.empty()):
    ret, im = cap.read()

    #drawHorizon(im)
    imgray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
    #imgray = cv2.GaussianBlur(imgray,(3,3),0)
    """
    VALORES PARA EL FILTRO OSCURO
    """
    ret,thresh = cv2.threshold(imgray,100,150,cv2.THRESH_BINARY)

    """
    VALORES PARA EL FILTRO MAS CLARO

    ret,thresh = cv2.threshold(imgray,230,255,cv2.THRESH_BINARY)
    """
    """
    VALORES PARA EL FILTRO MEDIO

    ret,thresh = cv2.threshold(imgray,230,255,cv2.THRESH_BINARY)
    """
    #height, width, depth = im.shape
    contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    minUmbralArea = 0
    maxUmbralArea = 400000

    #Calculo del centroide de cada contorno
    contFiltered = []
    centroids = []
    for contour in contours:
        M = cv2.moments(contour)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
        else:
            cx = int(M['m10'])
            cy = int(M['m01'])
        #Eliminamos todos los contornos que quedan por debajo del horizonte
        #if cy <= height/2 - 30:
            #Filtramos aquellos contornos que tengan un area que no estea entre los umbrales

        if M['m00'] >= minUmbralArea and M['m00'] <= maxUmbralArea:
            print 'AREA ' + str(M['m00'])
            contFiltered.append(contour)
            centroids.append((cx, cy))
            cv2.circle( im, (cx, cy), 3, ( 0, 0, 255 ), -1, 8 )

    cv2.drawContours(im,contFiltered,-1,(0,255,0),3)
    if len(centroids) > 1:
        if centroids[0] != (0,0):
            cv2.line(im,centroids[0],centroids[len(centroids)-1],(0,0,255),1)
    video.write(im)
    #cv2.imshow("Canny window", edges)
    cv2.imshow("Threshold window", thresh)
    cv2.imshow("Gray window", imgray)
    cv2.imshow("Image window", im)
    cv2.waitKey(33)
    #i = i + 1
video.release()