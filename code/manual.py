import RPi.GPIO as GPIO
import time
import cv2

dirA1 = 5
dirA2 = 6
spdA = 13
dirB1 = 16
dirB2 = 26
spdB = 12

#GPIO.cleanup()
#GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(dirA1,GPIO.OUT)
GPIO.setup(dirA2,GPIO.OUT)
GPIO.setup(spdA,GPIO.OUT)
GPIO.setup(dirB1,GPIO.OUT)
GPIO.setup(dirB2,GPIO.OUT)
GPIO.setup(spdB,GPIO.OUT)
pwmA=GPIO.PWM(spdA,100)
pwmB=GPIO.PWM(spdB,100)
pwmA.start(0)
pwmB.start(0)

cap = cv2.VideoCapture(0)

x = 1

def Stop():
    GPIO.output(dirA1,False)
    GPIO.output(dirA2,False)
    GPIO.output(dirB1,False)
    GPIO.output(dirB2,False)

def Forward():
    pwmA.ChangeDutyCycle(100)
    pwmB.ChangeDutyCycle(100)
    GPIO.output(dirA1,True)
    GPIO.output(dirA2,False)
    GPIO.output(dirB1,True)
    GPIO.output(dirB2,False)

def Backward():
    pwmA.ChangeDutyCycle(100)
    pwmB.ChangeDutyCycle(100)
    GPIO.output(dirA1,False)
    GPIO.output(dirA2,True)
    GPIO.output(dirB1,False)
    GPIO.output(dirB2,True)
    
def RightFor():
    pwmA.ChangeDutyCycle(50)
    pwmB.ChangeDutyCycle(80)
    GPIO.output(dirA1,True)
    GPIO.output(dirA2,False)
    GPIO.output(dirB1,True)
    GPIO.output(dirB2,False)

def LeftFor():
    pwmA.ChangeDutyCycle(80)
    pwmB.ChangeDutyCycle(50)
    GPIO.output(dirA1,True)
    GPIO.output(dirA2,False)
    GPIO.output(dirB1,True)
    GPIO.output(dirB2,False)

def RightBack():
    pwmA.ChangeDutyCycle(50)
    pwmB.ChangeDutyCycle(80)
    GPIO.output(dirA1,False)
    GPIO.output(dirA2,True)
    GPIO.output(dirB1,False)
    GPIO.output(dirB2,True)

def LeftBack():
    pwmA.ChangeDutyCycle(80)
    pwmB.ChangeDutyCycle(50)
    GPIO.output(dirA1,False)
    GPIO.output(dirA2,True)
    GPIO.output(dirB1,False)
    GPIO.output(dirB2,True)

try:
    while(1):
        ret, frame = cap.read()

        cv2.namedWindow('Video', cv2.WINDOW_NORMAL)
        cv2.imshow('Video', frame)
        
        if ((cv2.waitKey(33) == -1) and (x==1)):
            print ("Ninguna")
            Stop()
            x = 0

        if (cv2.waitKey(33) == ord('w')):
            print ("Upkey")
            Forward()
            x = 1
    
        if (cv2.waitKey(33) == ord('s')):
            print ("Downkey")
            Backward()
            x = 1
        
        if ((cv2.waitKey(33) == ord('q'))):
            print ("Leftkey + Upkey")
            LeftFor()
            x = 1
            
        if ((cv2.waitKey(33) == ord('e'))):
            print ("Rightkey + Upkey")
            RightFor()
            x = 1 
            
        if ((cv2.waitKey(33) == ord('d'))):
            print ("Rightkey + Downkey")
            RightBack()
            x = 1 
            
        if ((cv2.waitKey(33) == ord('a'))):
            print ("Leftkey + Downkey")
            LeftBack()
            x = 1 
            
        if (cv2.waitKey(33) == 32) or (cv2.waitKey(33) == ord('m')):
            GPIO.cleanup()
            break
except KeyboardInterrupt:
    #pwmA.stop()
    #pwmB.stop()
    GPIO.cleanup()     

#Always at the end of the code
GPIO.cleanup()
