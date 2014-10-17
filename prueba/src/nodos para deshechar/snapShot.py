#!/usr/bin/env python

"""
webcam-snapshot.py:
A simple tool for taking snapshots from webcam. The images are saved in the
current directory named 1.jpg, 2.jpg, ...

Usage:
    Press [SPACE] to take snapshot
    Press 'q' to quit
"""

import cv2


def grabarImagenes():
    cap = cv2.VideoCapture(1)
    contImg = 0
    if not cap.isOpened():
        print "Cannot open camera!"
        return

    while True:
        val, img = cap.read()
        cv2.imshow("video", img)

        if contImg < 10:
            cv2.imwrite('salida_000'+str(contImg)+'.jpeg', img)
        elif contImg < 100:
            cv2.imwrite('salida_00'+str(contImg)+'.jpeg', img)
        elif contImg < 1000:
            cv2.imwrite('salida_0'+str(contImg)+'.jpeg', img)
        else:
            cv2.imwrite('salida_'+str(contImg)+'.jpeg', img)

        contImg = contImg + 1

if __name__ == "__main__":
  grabarImagenes()