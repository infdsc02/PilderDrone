#!/usr/bin/env python
import numpy as np
import cv2
import math

def maxMinX(contours):
    i = 1
    x_max = contours[0][0][0][0]
    x_min = contours[0][0][0][0]
    print len(contours[0])
    while i < len(contours[0]):
        if x_max < contours[0][i][0][0]:
            x_max = contours[0][i][0][0]
        if x_min > contours[0][i][0][0]:
            x_min = contours[0][i][0][0]
        i = i + 1
    return (x_max, x_min)


def longSegment(x1, x2, y1, y2):
    return math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))

def maxLine(lines):
    max_long = -1
    if len(lines) > 0:
        for x1,y1,x2,y2 in lines[0]:
            longSe = longSegment(x1, x2, y1, y2)
            if  longSe > max_long:
                max_long = longSe
                line = (x1, y1, x2, y2)
    return line

i = 0
while i <= 2138:
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
    imgray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
    ret,thresh = cv2.threshold(imgray,127,255,0)
    blur = cv2.GaussianBlur(thresh,(7,7),5)
    #blur = cv2.bilateralFilter(thresh,9,75,75)

    contours, hierarchy = cv2.findContours(blur,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(im,contours,-1,(0,255,0),3)
    """
    print contours
    x = maxMinX(contours)
    print "x_max = " + str(x[0]) + " x_min = " + str(x[1])
    height, width, depth = im.shape
    print "height/2 " + str(height/2) + ", width/2 " + str(width/2)

    minLineLength = 10000
    maxLineGap = 5
    lines = cv2.HoughLinesP(thresh,1,np.pi/180,100,minLineLength,maxLineGap)
    if lines != None:
        for x1,y1,x2,y2 in lines[0]:
            cv2.line(im,(x1,y1),(x2,y2),(255,0,0),1)


    cv2.line(im,(0,height/2),(width,height/2),(0,0,255),1)
    cv2.line(im,(width/2,0),(width/2,height),(0,0,255),1)]
    cv2.circle(im,(95,54), 2, (0,0,255), 3)
    cv2.circle(im,(93,142), 2, (0,0,255), 3)
    """

    cv2.imshow("Blur window", blur)
    cv2.imshow("Image window", im)
    cv2.waitKey(33)
    i = i + 1
