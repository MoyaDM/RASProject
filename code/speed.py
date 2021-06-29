import RPi.GPIO as GPIO
import time

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

try:
    while(1):
        print("Front")
        GPIO.output(dirA1,True)
        GPIO.output(dirA2,False)
        GPIO.output(dirB1,True)
        GPIO.output(dirB2,False)
        #time.sleep(.1)
        pwmA.ChangeDutyCycle(100)
        pwmB.ChangeDutyCycle(100)
        time.sleep(5)
        print("Back")
        GPIO.output(dirA1,False)
        GPIO.output(dirA2,True)
        GPIO.output(dirB1,False)
        GPIO.output(dirB2,True)
        #time.sleep(.1)
        pwmA.ChangeDutyCycle(100)
        pwmB.ChangeDutyCycle(100)
        time.sleep(5)
except KeyboardInterrupt:
    #pwmA.stop()
    #pwmB.stop()
    GPIO.cleanup()     

#Always at the end of the code
GPIO.cleanup()
