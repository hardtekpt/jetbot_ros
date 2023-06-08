#!/usr/bin/env python
from __future__ import print_function
import rospy
import qwiic_micro_oled
import sys

from std_msgs.msg import String

screen = qwiic_micro_oled.QwiicMicroOled()

def printOLED(text):
    
	screen.clear(screen.PAGE)
	screen.print(text)
	screen.display()

def screen_listener(msg):
    	
	printOLED(msg.data)

# initialization
if __name__ == '__main__':
	
	rospy.init_node('jetbot_screen')

	rospy.loginfo("Screen connection status " + str(screen.connected))

	screen.begin()
	screen.clear(screen.ALL)
	screen.display()

	# setup ros node	
	rospy.Subscriber('~screen', String, screen_listener)

	# start running
	rospy.spin()