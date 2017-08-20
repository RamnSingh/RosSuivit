#!/usr/bin/env python

import rospy
import rospkg
import os
from random import randint
import xml.etree.cElementTree as ET

"""
This is the main function which will bootstrap the application (frames).
To run successfully this function 'numberOfAgents' param should  be transfered to it.

What it does :
1. Choose a leader frame ammong all frames to be created (by votes or rendomly)
2. Update a _suivit.launch file to create dynamic nodes (_suivit.launch has permission which is 0777)
3. At the end, it calls the newly updated _suivit.launch file
"""
def main():
	rospy.init_node("main")
	if rospy.has_param("numberOfAgents") : #Checking if the 'numberOfAgents' is supplied or not
		numberOfAgents = rospy.get_param("numberOfAgents")
		if numberOfAgents > 1 and numberOfAgents < 11 :
			leaderAgent = getLeader(numberOfAgents) #Selecting a leader
			rospy.loginfo("Leader agent is frame number : " + str(leaderAgent))
			updateLaunchFile(leaderAgent, numberOfAgents) #Updating _suivit.launch
			os.system("roslaunch suivit _suivit.launch") #executing updated _suivit.launch
		else :
			rospy.logerr("Number of agents passed must be between 1 and 11.")
	else :
		rospy.logerr("PLease ensure that you have set 'agents' argument required this program to run.")

"""
 It's main role is to update already created file which _suivit.launch in launch directory of the package.
 If the _suivit.launch file doesn't exist. In this case, create a launch file and nalme it _suivit.launch and give it 0777 permission.
 
 This functions takes two parameteres leader (already chosen leader) and totalNumberOfAgents (total number of frames to be created).
 Based on the arguments passed, it updates dynamically the _suivit.launch which then will be launched by main function of this file.
"""
def updateLaunchFile(leader, totalNumberOfAgents):
	rospack = rospkg.RosPack()
	packagePath = rospack.get_path("suivit")	#Getting the package directory path
	launchFilePath = packagePath + "/launch/_suivit.launch"; #Declaring the absolute path of _suivit.launch with the help of 'packagePath' variable
	
	open(launchFilePath, 'w').close() # Removing any existing content the the _suivit.launch
	
	"""
	In the next block of code, etree library is being used to structure the _suivit.launch 
	"""
	
	launchTag = ET.Element("launch") # root element for launch file
	
	# Creating master
	group = ET.SubElement(launchTag, "group", ns="leader")
	ET.SubElement(group, "node", pkg="turtlesim", type="turtlesim_node", name = "turtle" + str(leader), respawn="true")
	ET.SubElement(group, "node", pkg="turtlesim", type="turtle_teleop_key", name = "teleop", required="true")
	
	# Creating followers
	for frame in range (1, totalNumberOfAgents + 1):
		if frame != leader :
			ET.SubElement(launchTag, "node", pkg="turtlesim", type="turtlesim_node", name = "turtle" + str(frame), respawn="true")
	
	# Creating other required nodes
	ET.SubElement(launchTag, "param", name="leaderAgentNameSpace", type="str", value="leader")
	ET.SubElement(launchTag, "node", pkg="suivit", type="publishLeaderPosToLeaderPosHolder.py", name="publishLeaderPosToLeaderPosHolder", output="screen")
	ET.SubElement(launchTag, "node", pkg="suivit", type="publishLeaderPosToFollowers.py", name="publishLeaderPosToFollowers", output="screen")
	ET.SubElement(launchTag, "node", pkg="suivit", type="customizeFrames.py", name="customizeFrames")
	
	# Packing up all together and then writing to the _suivit.launch file
	tree = ET.ElementTree(launchTag)
	tree.write(launchFilePath)

"""
This function is a wrapper which wraps the all functions helping to choose a leader.
Basicaly it calls other functions, it is introduced to organise the code
"""
def getLeader(totalNumberOfAgents) :
	votes = doVoting(totalNumberOfAgents) #Making all agents votes for each other
	leaderInfo = getLeaderBasedOnVotes(votes)#Getting informations on chosen leader by voting as a tuple value
	
	isLeaderFound, leaderAgent = leaderInfo[0], leaderInfo[1]#Unpackaging tuple
	
	if isLeaderFound :
		rospy.loginfo("Leader found by voting")#If leader has been chosen, then log it to the screen.
	else : #if leader has not been chosen due to a tie
		rospy.loginfo("Voting didn't help to chose a leader")
		rospy.loginfo("Randomly chosing a leader")
		leaderAgent = randint(1, totalNumberOfAgents) #Randomly selecting any agent to be the leader agent
	
	return leaderAgent

"""
 It makes all agents votes for each others. It returns a list containig all votes.
"""
def doVoting (numberOfVoters) :
	votes = []
	for x in range (1, numberOfVoters + 1) :
		votes.append(randint(1, numberOfVoters))
	return votes

"""
 Based on the votings done in the 'doVoting' function, it let application choose a leader.
"""
def getLeaderBasedOnVotes (votes) :
	
	votesPerIndex = []
	"""
	This loop determine the how many votes an agent got
	"""
	for x in range (0, len(votes)) :
		voteForThisElement = 0;
		for y in range (0, len(votes)) :
			if votes[x] == votes[y] :
				voteForThisElement += 1
		votesPerIndex.append(voteForThisElement)

	
	maxNumberOfVotes = 0
	isUniqueMaxNumberOfVotes = True
	leader = 0;
	
	"""
	This loop returns the leader who got the higher number of votes by ensuring 
	that no other agent shares the same number of votes with this agent
	"""
	for x in range (0, len(votesPerIndex)):
		if votesPerIndex[x] > maxNumberOfVotes :
			maxNumberOfVotes = votesPerIndex[x]
			leader = x + 1
			isUniqueMaxNumberOfVotes = True
		elif votesPerIndex[x]  == maxNumberOfVotes :
			isUniqueMaxNumberOfVotes = False
	
	return (isUniqueMaxNumberOfVotes, leader)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
