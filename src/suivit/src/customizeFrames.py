#!/usr/bin/env python
import rospy
import os

"""
The main purpose of this file is to customize the leader agent frame which helps user's to better understand the situation.
'customizeFrames' function call two ros commands to customize the look and feel of the leader agent frame
"""
def customizeFrames():
	os.system("rosparam set leader/background_r 150")
	os.system("rosservice call leader/clear")
	rospy.loginfo("Hi, i'm leader. I've different colour among all the agents")
	
if __name__ == '__main__' :
	try :
		customizeFrames()
	except rospy.ROSInterruptException:
		pass
