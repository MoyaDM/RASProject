import RPi.GPIO as GPIO
import time

dirA1 = 5
dirA2 = 6
 
#GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(dirA1,False)
GPIO.setup(dirA2,False)
 
while(1):
    print ("x")
    GPIO.output(dirA1,True)
    GPIO.output(dirA2,False)
    time.sleep(5)
    GPIO.output(dirA1,False)
    GPIO.output(dirA2,True)
    time.sleep(5)

#Always at the end of the code
GPIO.cleanup()

