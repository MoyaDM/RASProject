# import required libraries
import numpy as np
import matplotlib.pyplot as plt
import cv2
import math
import RPi.GPIO as GPIO

#cap=cv2.VideoCapture(0)

#fourcc=cv2.VideoWriter_fourcc(*'XVID')
#out=cv2.VideoWriter('Prueba2.avi',fourcc,20.0,(640,480))

dirA1 = 16
dirA2 = 26
spdA = 13
dirB1 = 5
dirB2 = 6
spdB = 12

GPIO.cleanup()
#GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(dirA1,GPIO.OUT)
GPIO.setup(dirA2,GPIO.OUT)
GPIO.setup(spdA,GPIO.OUT)
GPIO.setup(dirB1,GPIO.OUT)
GPIO.setup(dirB2,GPIO.OUT)
GPIO.setup(spdB,GPIO.OUT)
pwmA=GPIO.PWM(spdA,1000)
pwmB=GPIO.PWM(spdB,1000)
pwmA.start(0)
pwmB.start(0)


def Forward():
    pwmA.ChangeDutyCycle(100)
    pwmB.ChangeDutyCycle(100)
    GPIO.output(dirA1,True)
    GPIO.output(dirA2,False)
    GPIO.output(dirB1,True)
    GPIO.output(dirB2,False)
    
def RightFor():
    pwmA.ChangeDutyCycle(100)
    pwmB.ChangeDutyCycle(70)
    GPIO.output(dirA1,True)
    GPIO.output(dirA2,False)
    GPIO.output(dirB1,True)
    GPIO.output(dirB2,False)
    
def LeftFor():
    pwmA.ChangeDutyCycle(70)
    pwmB.ChangeDutyCycle(100)
    GPIO.output(dirA1,True)
    GPIO.output(dirA2,False)
    GPIO.output(dirB1,True)
    GPIO.output(dirB2,False)

from numpy.core.numerictypes import maximum_sctype

def regionOfInterest(img, vertices):
    mask = np.copy(img)*0
    cv2.fillPoly(mask, vertices, 255)
    masked_image = cv2.bitwise_and(img,mask)
    return masked_image

#img_name = '../img/phoneVideo2.mp4'
cap = cv2.VideoCapture(0)

#cap = cv2.VideoCapture(img_name)

left_lineX = []
left_lineY = []
right_lineX = []
right_lineY = []

old_left_lineY = []
old_left_lineX = []
old_right_lineY = []
old_right_lineX = []

while(cap.isOpened()):

    ret, frame = cap.read()
    frame = cv2.resize(frame, (424,240))

    ##TO-DO Gray scale
    grey = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)


    ##TO-DO Gaussian Blur
    kernel_size = (5,5)
    blur_grey = cv2.GaussianBlur(grey, kernel_size, 0)
    for i in range(5):
        blur_grey = cv2.GaussianBlur(blur_grey, kernel_size, 0)

    ##TO-DO Canny
    low_threshold = 5
    high_threshold = 90
    edges = cv2.Canny(blur_grey, low_threshold, high_threshold, apertureSize=3)

    """
        *       *

    *               *


    *               *
    """

    ##TO-DO ROI (Region Of Interest)
    bottom_left = (0,240)
    bottom_right = (420,240)
    center_left = (5,60)
    center_right = (415,60)
    top_left = (10,40)
    top_right = (410,40)
    vertices = np.array([[bottom_left,center_left,top_left,top_right,center_right,bottom_right]],dtype=np.int32)
    masked_edges = regionOfInterest(edges,vertices)

    ##TO-DO HOUGHT LINES

    rho = 1                  # distance resolution in pixels of the Hough grid
    theta = np.pi/180        # angular resolution in radians of the Hough grid
    threshold = 13           # minimum number of votes (intersections in Hough grid cell)
    min_line_len = 90         # minimum number of pixels making up a line
    max_line_gap = 10        # maximum gap in pixels between connectable line segments


    houghtLines = cv2.HoughLinesP(masked_edges,rho,theta,threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)

    for x in range(0,len(houghtLines)):
        for x1,y1,x2,y2 in houghtLines[x]:
            slope = (y2-y1)/(x2-x1)
            
            if slope == 0:
                continue
            else:
                if math.fabs(slope) < 0.18 and math.fabs(slope) > -0.18:
                    continue
                #else:
                    #cv2.line(frame,(x1,y1),(x2,y2),(255,0,0),3)
            
    if houghtLines is not None:

        left_lineX = []
        left_lineY = []
        right_lineX = []
        right_lineY = []

        old_left_lineY = [2,2]
        old_left_lineX = [2,2]
        old_right_lineY = [2,2]
        old_right_lineX = [2,2]

        for line in houghtLines:
            for x1,y1,x2,y2 in line:
                slope = (y2-y1)/(x2-x1)
                if slope == 0:
                    continue
                else:
                    if math.fabs(slope) < 0.19 and math.fabs(slope) > -0.19:
                        continue
                    elif slope <=0 :
                        left_lineX.extend([x1,x2])
                        left_lineY.extend([y1,y2])
                    else:
                        right_lineX.extend([x1,x2])
                        right_lineY.extend([y1,y2])

                    #cv2.line(frame,(x1,y1),(x2,y2),(0,255,0),2)

    if len(left_lineX) > 0 and len(left_lineY) > 0 and len(right_lineX) > 0 and len(right_lineY) > 0:
        poly_left = np.poly1d(np.polyfit(left_lineY,left_lineX,deg=1))
        poly_right = np.poly1d(np.polyfit(right_lineY,right_lineX,deg=1))

        old_left_lineY = left_lineY
        old_left_lineX = left_lineX
        old_right_lineY = right_lineY
        old_right_lineX = right_lineX

        maxY = 240
        minY = 30

        leftXstart = int(poly_left(maxY))
        leftXend = int(poly_left(minY))

        rightXstart = int(poly_right(maxY))
        rightXend = int(poly_right(minY))

        cv2.line(frame,(leftXstart,maxY),(leftXend,minY),(0,0,255),3) # Linea roja
        cv2.line(frame,(rightXstart,maxY),(rightXend,minY),(0,0,255),3)

        centerXstar = int((rightXstart - leftXstart)/2) + leftXstart
        centerXend = int((rightXend - leftXend)/2) + leftXend

        cv2.line(frame,(centerXstar,maxY),(centerXend,minY),(163, 73, 164),2) # Linea morada
        
        Forward()

        print ("Adelante")

    elif len(left_lineX) <= 0 and len (right_lineX) > 0:
        poly_right = np.poly1d(np.polyfit(right_lineY,right_lineX,deg=1))
        
        maxY = 240
        minY = 30

        rightXstart = int(poly_right(maxY))
        rightXend = int(poly_right(minY))
        
        # Linea roja
        cv2.line(frame,(rightXstart,maxY),(rightXend,minY),(0,0,255),3)
        
        LeftFor()
        print ("Izquierda")

    elif len (right_lineX) <= 0 and len(left_lineX) > 0:
        poly_left = np.poly1d(np.polyfit(left_lineY,left_lineX,deg=1))

        maxY = 240
        minY = 30

        leftXstart = int(poly_left(maxY))
        leftXend = int(poly_left(minY))

        cv2.line(frame,(leftXstart,maxY),(leftXend,minY),(0,0,255),3) # Linea roja

        RightFor()
        print ("Derecha")

    else:
        Forward()    

    x1line = 212
    y1line = 240
    x2line = 212
    y2line = 0

    cv2.line(frame,(x1line,y1line),(x2line,y2line),(255,255,255),5)
    
    cv2.namedWindow('Video', cv2.WINDOW_NORMAL)
    cv2.imshow('Video', frame)
    cv2.resizeWindow('Video', 800,500)

    # Press Q on keyboard to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        GPIO.cleanup()
        break


GPIO.cleanup()
