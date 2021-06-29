"""
sudo apt-get update
sudo apt-get install rpi.gpio
"""

import RPi.GPIO as GPIO
import time

dirA1 = 5
dirA2 = 6
spdA = 12

#GPIO.setwarnings(False)
#GPIO.setmode(GPIO.BOARD)
GPIO.setmode(GPIO.BCM)
GPIO.setup(dirA1,GPIO.OUT)
GPIO.setup(dirA2,GPIO.OUT)
GPIO.setup(spdA,GPIO.OUT)
pwmA=GPIO.PWM(spdA,1000) #freq
pwmA.start(50)
pwmA.ChangeDutyCycle(10)

print ("Entrar")
for  x in range ( 0, 2):
    GPIO.output(dirA1,GPIO.LOW)
    GPIO.output(dirA2,GPIO.LOW)
    
    time.sleep(10)
    
    GPIO.output(dirA1,GPIO.HIGH)
    GPIO.output(dirA2,GPIO.HIGH)
    pwmA.ChangeDutyCycle(50)
    
    time.sleep(10)

    #Final
    GPIO.output(dirA1,GPIO.LOW)
    GPIO.output(dirA2,GPIO.LOW)
    pwmA.ChangeDutyCycle(10)
    print (x)
print ("Salir")
#Always at the end of the code
GPIO.cleanup()  

