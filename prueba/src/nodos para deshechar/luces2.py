#!/usr/bin/env python
import numpy as np
import cv2
import math

def drawHorizon(img):
    height, width, depth = img.shape
    cv2.line(img,(0,height/2 - 30),(width,height/2 - 30),(0,0,255),1)

def distPuntos(x1, y1, x2, y2):
    dist = math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2))

def calculateCentroid(contour):
    M = cv2.moments(contour)
    if M['m00'] > 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
    else:
        cx = int(M['m10'])
        cy = int(M['m01'])
    return(cx, cy)

def compareCentroids(centroids):
    ret = []
    i = 0
    umbral = 1
    while i < len(centroids) - 1:
        j = i + 1
        while j < len(centroids):
            if (centroids[i][0] >= centroids[j][0] - umbral) and (centroids[i][0] <= centroids[j][0] + umbral):
                ret.append(centroids[i])
                ret.append(centroids[j])
            j = j + 1
        i = i + 1
    return ret

i = 0
while i <= 5381:
    if i < 10:
        foto = '/home/david/proyecto/fotos/planta_2/salida_000'+str(i)+'.jpeg'
    elif i < 100:
        foto = '/home/david/proyecto/fotos/planta_2/salida_00'+str(i)+'.jpeg'
    elif i < 1000:
        foto = '/home/david/proyecto/fotos/planta_2/salida_0'+str(i)+'.jpeg'
    else:
        foto = '/home/david/proyecto/fotos/planta_2/salida_'+str(i)+'.jpeg'
    print foto
    im = cv2.imread(foto)


    #drawHorizon(im)
    imgray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
    #imgray = cv2.GaussianBlur(imgray,(3,3),0)

    ret,thresh = cv2.threshold(imgray,245,255,cv2.THRESH_BINARY)

    #thresh = cv2.adaptiveThreshold(imgray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,-15)

    height, width, depth = im.shape
    contours, hierarchy = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    #Calculo del centroide de cada contorno
    contFiltered = []
    centroids = []
    for contour in contours:
        centroid = calculateCentroid(contour)
        #Eliminamos todos los contornos que quedan por debajo del horizonte

        contFiltered.append(contour)
        centroids.append(centroid)

    centroids = compareCentroids(centroids)
    for centroid in centroids:
        cv2.circle( im, centroid, 3, ( 0, 0, 255 ), -1, 8 )

    cv2.drawContours(im,contFiltered,-1,(0,255,0),3)
    if len(centroids) > 1:
        if centroids[0] != (0,0):
            cv2.line(im,centroids[0],centroids[len(centroids)-1],(0,0,255),1)

    #cv2.imshow("Canny window", edges)
    cv2.imshow("Threshold window", thresh)
    cv2.imshow("Gray window", imgray)
    cv2.imshow("Image window", im)
    cv2.waitKey(33)
    i = i + 1