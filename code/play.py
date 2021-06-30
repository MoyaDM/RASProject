import RPi.GPIO as GPIO		#inicializo los pines GPIO de la RPi
import time			#inicializo los timers y contadores de la RPi
import pygame			#inicializo la librería para la lectura de los botones del control del PS4

dirA1 = 5
dirA2 = 6
spdA = 13
dirB1 = 16
dirB2 = 26
spdB = 12

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

print 'Jostick iniciado : %s' % j.get_name()	

try:
	while True:
		pygame.event.pump()					#se empieza librería para poder hacer lectura de los botones del control del PS3 como eventos
		if (j.get_axis(1) > 0.2) or (j.get_axis(1) < -0.2):	#adelante y atras, para que funcione solo cuando la palanca izquierda esté a más de 20% de la amplitud total
			if j.get_axis(1) < -0.2:			#para movimiento del eje Y hacia arriba
				CicloDeTrabajo = j.get_axis(1)*(-100) 	#multiplica el valor del la palanca por 100 para conseguir el cilco de trabajo de 0 a 100%
				if CicloDeTrabajo>100:			#si se llegara a pasar de 100% se regresa a 100%
					CicloDeTrabajo=100
				print ("Arriba")		#conjunto de ciclos de trabajo para que camione hacia adelante
			if j.get_axis(1) > 0.2:				#con el movimiento del eje y hacia abajo
				CicloDeTrabajo = j.get_axis(1)*(100)	#se multiplica por 100 para llegar al ciclo de trabajo de 0 a 100 %
                if CicloDeTrabajo>100:			#si se llegara a pasar de 100% se regresa a 100%
                    CicloDeTrabajo=100
                print ("Abajo")
		
		if (j.get_axis(1)> -0.2) and (j.get_axis(1)<0.2) and (j.get_axis(2)<0.2) and (j.get_axis(2)>-0.2):#robot parado si la palanca se encuentra a menos del 20% de su amplitud para darle un juego sin que se mueva
			print ("Quieto")			#conjunto de PWM para que el robot se pare completamente
		
		if (j.get_axis(2) > 0.2) or (j.get_axis(2) < -0.2):	#giro derecha e izquierda
            if j.get_axis(2) < -0.2:
                CicloDeTrabajo = j.get_axis(2)*(-100)	#multiplica el valor del la palanca por 100 para conseguir el cilco de trabajo de 0 a 100%
                if CicloDeTrabajo>100:			#si se llegara a pasar de 100% se regresa a 100%
                    CicloDeTrabajo=100
                print ("Derecha")		#conjunto de PWM para que el robot gire a la derecha
            if j.get_axis(2) > 0.2:
                CicloDeTrabajo = j.get_axis(2)*(100)	#multiplica el valor del la palanca por 100 para conseguir el cilco de trabajo de 0 a 100%
                if CicloDeTrabajo>100:			#si se llegara a pasar de 100% se regresa a 100%
                    CicloDeTrabajo=100
                print ("Izquierda")	#conjunto de PWM para que el robot gire a la izquierda
except KeyboardInterrupt:						#se limpia todo el codigo si se interrumpe con Ctrl + C
	GPIO.cleanup()





