import pygame
import RPi.GPIO as GPIO
import time
import cv2
import numpy as np

cap=cv2.VideoCapture(0)

fourcc=cv2.VideoWriter_fourcc(*'XVID')
out=cv2.VideoWriter('Prueba2.avi',fourcc,20.0,(640,480))

dirA1 = 16
dirA2 = 26
spdA = 13
dirB1 = 5
dirB2 = 6
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
pwmA=GPIO.PWM(spdA,1000)
pwmB=GPIO.PWM(spdB,1000)
pwmA.start(0)
pwmB.start(0)


pygame.init()
j = pygame.joystick.Joystick(0)
j.init()
print ("Jostick iniciado") 

def Stop():
    GPIO.output(dirA1,False)
    GPIO.output(dirA2,False)
    GPIO.output(dirB1,False)
    GPIO.output(dirB2,False)

def Forward(Dutycicle):
    pwmA.ChangeDutyCycle(Dutycicle)
    pwmB.ChangeDutyCycle(Dutycicle)
    GPIO.output(dirA1,True)
    GPIO.output(dirA2,False)
    GPIO.output(dirB1,True)
    GPIO.output(dirB2,False)

def Backward(Dutycicle):
    pwmA.ChangeDutyCycle(Dutycicle)
    pwmB.ChangeDutyCycle(Dutycicle)
    GPIO.output(dirA1,False)
    GPIO.output(dirA2,True)
    GPIO.output(dirB1,False)
    GPIO.output(dirB2,True)
    
def RightFor(Dutycicle):
    pwmA.ChangeDutyCycle(100)
    pwmB.ChangeDutyCycle(100 - Dutycicle)
    GPIO.output(dirA1,False)
    GPIO.output(dirA2,True)
    GPIO.output(dirB1,True)
    GPIO.output(dirB2,False)
    
def LeftFor(Dutycicle):
    pwmA.ChangeDutyCycle(100 - Dutycicle)
    pwmB.ChangeDutyCycle(100)
    GPIO.output(dirA1,True)
    GPIO.output(dirA2,False)
    GPIO.output(dirB1,False)
    GPIO.output(dirB2,True)
    
def RightBack(Dutycicle):
    pwmA.ChangeDutyCycle(100)
    pwmB.ChangeDutyCycle(100 - Dutycicle)
    GPIO.output(dirA1,True)
    GPIO.output(dirA2,False)
    GPIO.output(dirB1,False)
    GPIO.output(dirB2,True)

def LeftBack(Dutycicle):
    pwmA.ChangeDutyCycle(100 - Dutycicle)
    pwmB.ChangeDutyCycle(100)
    GPIO.output(dirA1,False)
    GPIO.output(dirA2,True)
    GPIO.output(dirB1,True)
    GPIO.output(dirB2,False)
    
try:
    while(1):
        pygame.event.pump()
        
        ret, frame=cap.read()
        if(ret==True):
            out.write(frame)
            cv2.imshow('output',frame)
            if (cv2.waitKey(1) & 0xFF == ord('q')):
                break
                
        if (j.get_axis(1) < -0.2) and (j.get_axis(0) < 0.2) and (j.get_axis(0) > -0.2):
            print ("Arriba")
            Dutycicle = j.get_axis(1) * (-100)
            if Dutycicle > 100:
                Dutycicle = 100    
            Forward(Dutycicle)
        
        if (j.get_axis(1) > 0.2) and (j.get_axis(0) < 0.2) and (j.get_axis(0) > -0.2):
            print ("Abajo")
            Dutycicle = j.get_axis(1) * (100)
            if Dutycicle > 100:
                Dutycicle = 100 
            Backward(Dutycicle)
            
        if (j.get_axis(1) < -0.2) and (j.get_axis(0) > 0.2):
            print ("Derecha + Arriba")
            Dutycicle = j.get_axis(0) * (200)
            if Dutycicle > 30:
                Dutycicle = 30 
            RightFor(Dutycicle)
        
        if (j.get_axis(1) < -0.2) and (j.get_axis(0) < -0.2):
            print ("Izquierda + Arriba")
            Dutycicle = j.get_axis(0) * (-100)
            if Dutycicle > 70:
                Dutycicle = 70 
            LeftFor(Dutycicle)
                
        if (j.get_axis(1) > 0.2) and (j.get_axis(0) > 0.2):
            print ("Derecha + Abajo")
            Dutycicle = j.get_axis(0) * (100)
            if Dutycicle > 70:
                Dutycicle = 70 
            RightBack(Dutycicle)
        
        if (j.get_axis(1) > 0.2) and (j.get_axis(0) < -0.2):
            print ("Izquierda + Abajo")
            Dutycicle = j.get_axis(0) * (-100)
            if Dutycicle > 70:
                Dutycicle = 70 
            LeftBack(Dutycicle)
              
        if (j.get_axis(1) > -0.2) and (j.get_axis(1) < 0.2) and (j.get_axis(0) < 0.2) and (j.get_axis(0) > -0.2):
            print ("Detenido")
            Stop() 
            
except KeyboardInterrupt:
    #pwmA.stop()
    #pwmB.stop()
    GPIO.cleanup()
    cap.release()
    cv2.destroyAllWindows()      

#Always at the end of the code
GPIO.cleanup()
cap.release()
cv2.destroyAllWindows()

