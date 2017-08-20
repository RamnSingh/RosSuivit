#!/usr/bin/env python

import rospy
import rospkg
from geometry_msgs.msg import Twist

"""
This file publishes the leader agent current position to the followers
"""

"""
It creates a node called 'publishLeaderPosToFollowers' which subscribes 'leaderPosHolder' topic
to get the current position of leader agent in Twist type
"""
def publishLeaderPosToFollowers():
	rospy.init_node("publishLeaderPosToFollowers") # Creating node
	rospy.Subscriber("leaderPosHolder", Twist, publishLeaderPos) #Subscribing to 'leaderPosHolder' which holds the leader agent current position
	rospy.spin()

"""
It's simple callback which recieves the leader agent current position then broadcast it to all followers
"""
def publishLeaderPos (pos) :
	publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size = 1000)
	publisher.publish(pos)
	
if __name__ == '__main__':
    try:
        publishLeaderPosToFollowers()
    except rospy.ROSInterruptException:
        pass
