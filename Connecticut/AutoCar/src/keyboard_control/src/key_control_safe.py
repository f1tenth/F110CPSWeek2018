#! /usr/bin/env python
import rospy
from ac_msgs.msg import drive_params
from std_msgs.msg import Bool
import curses

currwindow = curses.initscr()
currwindow.keypad(1)
curses.cbreak()
curses.halfdelay(1)
curses.noecho()
rospy.init_node("KeyboardControl")
DriveParamPublisher = rospy.Publisher("drive_parameters", drive_params, queue_size=10)
EStopPublisher = rospy.Publisher("eStop", Bool, queue_size=10)
turn = 0
while(1):
    currwindow.refresh()
    currchar = currwindow.getch()
    msg = drive_params()
    if (currchar == curses.KEY_UP):
        msg.velocity = 12
    elif (currchar == curses.KEY_DOWN):
        msg.velocity = -12
    if (currchar == curses.KEY_LEFT):
        turn -= 2
    elif (currchar == curses.KEY_RIGHT):
        turn += 2
    if (currchar == -1):
        msg.velocity = 0
    if (currchar == 99):
        turn = 0
    if (currchar == 32):
        msg.velocity = 0
        turn = 0
        EStopPublisher.publish(False)
        break
    msg.angle = turn
    DriveParamPublisher.publish(msg)
curses.nocbreak()
curses.echo()
currwindow.keypad(0)
curses.endwin()