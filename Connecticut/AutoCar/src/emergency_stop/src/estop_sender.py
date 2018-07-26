#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
import curses

"""
The aim of this program is to listen for the spacebar being pressed
Upon this press, the program publishes True to the eStop topic, stopping the car
"""
print("Emergency Stop Sender Initialized")
rospy.init_node("emergency_stop_sender")
pub = rospy.Publisher('eStop', Bool, queue_size=10)

stdscr = curses.initscr() # Initialize curses
curses.cbreak() # Don't require enter to be pressed before a read
curses.noecho() # Don't echo keys to screen
stdscr.keypad(1) # Enable curses key translation

stdscr.addstr("STARTED LISTENING: EMERGENCY NOT TRIGGERED")
stdscr.refresh()



# Main listen loop
while True:
    c = stdscr.getch()
    if(c == 32):
        pub.publish(True)
        stdscr.erase()
        stdscr.addstr("EMERGENCY TRIGGERED: ESTOP ACTIVATED", curses.A_BLINK)
        stdscr.refresh()
        curses.nocbreak()
        stdscr.keypad(0)
        curses.echo()
        sleep(5)
        curses.endwin()
        break
    else:
        pub.publish(False)
