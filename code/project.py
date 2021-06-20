# import required libraries
import numpy as np
import matplotlib.pyplot as plt
import cv2
import math

img_name = '../img/highway.mp4'

cap = cv2.VideoCapture(img_name)


while(cap.isOpened()):

    ret, frame = cap.read()

    
    ##TO-DO Gray scale
    img_gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    ##TO-DO Gaussian Blur
    kernel_size = (7,7)
    gauss = cv2.GaussianBlur(img_gray, kernel_size, 0)
    ##TO-DO Canny
    umbral_minimo = 125
    umbral_maximo = 325

    canny = cv2.Canny(frame, umbral_minimo, umbral_maximo)

    ##TO-DO ROI (Region Of Interest)
    


    ##TO-DO HOUGHT LINES

            
        
    cv2.namedWindow('Video', cv2.WINDOW_NORMAL)
    cv2.imshow('Video', frame)
    cv2.imshow('Canny', canny)
    cv2.resizeWindow('Video', 1000,900)


    # Press Q on keyboard to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


# When everything done, release the video capture object
cap.release()

# Closes all the frames
cv2.destroyAllWindows()
        

