#! /usr/bin/env python
import pygame
import rospy
from ac_msgs.msg import drive_params
from std_msgs.msg import Bool
from pygame.locals import *

# Initialise screen
pygame.init()
screen = pygame.display.set_mode((300, 150))
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
rospy.init_node("MultiKeyboardControl")
# Publisher init
DriveParamPublisher = rospy.Publisher("drive_parameters", drive_params, queue_size=1)
EStopPublisher = rospy.Publisher("eStop", Bool, queue_size=1)

line_control = False


# callback function on occurance of line_control:drive_parameters (angle & velocity)
def callback(data):
    print "\t inside callback"
    global line_control
    global DriveParamPublisher
    msg = drive_params()
    msg.velocity = data.velocity
    msg.angle = data.angle
    print line_control
    if line_control == True:
        print "\t\t inside line control"
        DriveParamPublisher.publish(msg)


# set up listening to line controller
rospy.Subscriber("line_controller_drive_parameters", drive_params, callback)

# Event loop
estopmsg = Bool()
msg = drive_params()
turn = 0
velocity = 0

# pygame.key.set_repeat(500, 500)

while 1:
    for event in pygame.event.get():
        if (event.type == pygame.KEYDOWN or event.type == pygame.KEYUP) and event.key == pygame.K_l:
            line_control = True
        else:
            line_control = False
        if event.type == pygame.QUIT:
            break
    key = pygame.key.get_pressed()  # checking the state of keys (which ones are currently pressed)
    if key[pygame.K_q]:  # Quit/Exit
        pygame.display.quit()
        break
    if key[pygame.K_UP] and key[pygame.K_DOWN]:
        print "\t Both Up and Down keys pressed!!! \n"
        velocity = 0
    elif key[pygame.K_UP]:
        if velocity < 0:
            velocity = 10
        else:
            velocity = 10
    elif key[pygame.K_DOWN]:
        if velocity > 0:
            velocity = -10
        else:
            velocity = -10
    else:
        velocity = 0
    if key[pygame.K_LEFT] and key[pygame.K_RIGHT]:
        print "\t Both Left and Right keys pressed!!! \n"
        turn = 0
    elif key[pygame.K_LEFT]:
        turn = -100
    elif key[pygame.K_RIGHT]:
        turn = 100
    else:
        turn = 0
    if key[pygame.K_l]:
        line_control = True
    if key[pygame.K_F7]:
        estopmsg = True
        EStopPublisher.publish(estopmsg)
    if key[pygame.K_c]:
        turn = 0
    if (not line_control) and (msg.angle != turn or msg.velocity != velocity):
        msg.angle = turn
        msg.velocity = velocity
        DriveParamPublisher.publish(msg)

    screen.blit(background, (0, 0))
    pygame.display.flip()
