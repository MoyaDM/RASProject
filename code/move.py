import pygame

pygame.init()
j = pygame.joystick.Joystick(0)
j.init()

while(1):
    pygame.event.pump()

    e = j.get_axis(0)
    print (e)
