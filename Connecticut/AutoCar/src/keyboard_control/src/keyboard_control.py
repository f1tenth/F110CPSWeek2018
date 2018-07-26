#!/usr/bin/env python
import rospy
from ac_msgs.msg import drive_params
from std_msgs.msg import Bool
import pygame
from pygame.locals import *

# Initialise screen
pygame.init()
screen = pygame.display.set_mode((300,150))
pygame.display.set_caption('Multi-key capture window')

# Fill background
background = pygame.Surface(screen.get_size())
background = background.convert()
background.fill((250, 250, 250))

# Display some text
font = pygame.font.Font(None, 36)

# Blit everything to the screen
screen.blit(background, (0, 0))
pygame.display.flip()
rospy.init_node("keyboard_control")
#Publisher init
DriveParamPublisher = rospy.Publisher("drive_parameters", drive_params, queue_size=10)
EStopPublisher = rospy.Publisher("eStop", Bool, queue_size=10)

# Event loop
estopmsg = Bool()
msg = drive_params()
turn = 0
velocity = 0

#pygame.key.set_repeat(500, 500)
while 1:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            break
    key = pygame.key.get_pressed()  # checking pressed keys
    if key[pygame.K_q]: #Quit/Exit
        pygame.display.quit()
        break
    if key[pygame.K_UP] and key[pygame.K_DOWN]:
        velocity = 0
    elif key[pygame.K_UP]:
        velocity = 20
    elif key[pygame.K_DOWN]:
        velocity = -30
    else:
        velocity = 0
    if key[pygame.K_LEFT] and key[pygame.K_RIGHT]:
        turn = 0
    elif key[pygame.K_LEFT]:
        turn = -60
    elif key[pygame.K_RIGHT]:
        turn = 60
    else:
        turn = 0
    if key[pygame.K_SPACE]:
        estopmsg = True
        EStopPublisher.publish(estopmsg)
    if key[pygame.K_c]:
        turn = 0
    if (msg.angle != turn or msg.velocity != velocity):
        msg.angle = turn
        msg.velocity = velocity
        DriveParamPublisher.publish(msg)

    screen.blit(background, (0, 0))
pygame.display.flip()
