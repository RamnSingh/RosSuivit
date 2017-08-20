#!/usr/bin/env python

import rospy
import rospkg
from geometry_msgs.msg import Twist

"""
This file publishes the currebt position of the leader agent to the topic 'publishLeaderPosToLeaderPosHolder'
which then is collected by fllowers by using another node
"""

"""
It initializes the 'publishLeaderPosToLeaderPosHolder' which subscribes to the leader agent current position.
It also needs 'leaderAgent' parameter which is dynamically assigned by main.py file
"""
def publishLeaderPosToLeaderPosHolder():
	rospy.init_node("publishLeaderPosToLeaderPosHolder") # Creating nodes
	
	if rospy.has_param("leaderAgentNameSpace") :
		leaderAgentNameSpace = rospy.get_param("leaderAgentNameSpace")
		rospy.Subscriber(leaderAgentNameSpace + "/turtle1/cmd_vel", Twist, publishLeaderPos) #Subscring to the topic which gives the current position of the leader agent
		rospy.spin()
	else :
		rospy.logerr("'leaderAgent' param is not defined")

"""
It's a callback function which is invoked when the current position of leader agent is recieved by the a'publishLeaderPosToLeaderPosHolder' method
"""
def publishLeaderPos (pos) :
	publisher = rospy.Publisher("leaderPosHolder", Twist, queue_size = 1000)
	publisher.publish(pos)
	
if __name__ == '__main__':
    try:
        publishLeaderPosToLeaderPosHolder()
    except rospy.ROSInterruptException:
        pass
