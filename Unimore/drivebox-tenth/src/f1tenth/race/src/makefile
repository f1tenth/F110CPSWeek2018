#!/bin/bash
serial:
	rosrun rosserial_python serial_node.py /dev/ttyACM0

lidar:
	rosrun urg_node urg_node _ip_address:=192.168.0.10

control:
	rosrun race control.py

dist:
	rosrun race dist_finder.py

kill:
	rosrun race kill.py

talker:
	rosrun race talker.py
