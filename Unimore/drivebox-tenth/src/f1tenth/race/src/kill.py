#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
import curses

stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)
rospy.init_node('kill_switch', anonymous=True)
em_pub = rospy.Publisher('eStop', Bool, queue_size=10)

stdscr.refresh()

key = ''
running = True
while key != ord('q'):
	key = stdscr.getch()
	stdscr.refresh()
	if key == curses.KEY_BACKSPACE:
		if running == True:
			em_pub.publish(True)
			stdscr.addstr(5, 20, "Emergency STOP!!!!!")
			running = False
		else:
			em_pub.publish(False)
			stdscr.addstr(5, 20, "Normal Operation :)")
			running = True

curses.endwin()
