#Author: Sean Wilson s0831408

"""epsilon-soft
	on-policy
	monte carlo"""

#==========IMPORTS===========
from __future__ import division
from Config import *
from ResultsPlotter import *
import motion
import time
import math
import random
import sys

#============INSTANCE VARIABLES===================
motionProxy = ALProxy("ALMotion", IP, PORT)
tts 		= ALProxy("ALTextToSpeech",IP,PORT)
tts.setVolume(0.6)	#Volume level for robot talk

epsilon = 1 					#e-greedy probability
episodeNum = 0 					#This increases with each episode

noEpisodes = 100				#The number of episodes (trials) in the test
strategyType = 0 				#This determines what type of distance aiming for

staticOptimalPolicyCount = 0 	#Increases when optimal policy doesn't change, resets when does
distancesList 	 = []
bestDistance 	 = []			#Hold best distance and policy that got it

#===============ENVIRONMENT SETUP=================

#To edit the environment (e.g. number of states, etc.) please edit this file.
from EnvironmentSetup import *

#Set size of bestDistance array. Depends on number of steps in episodes (+1 time)
for i in range(0, noTimeSteps+1):
	bestDistance.append(0)


#================================READY ROBOT====================================
def readyRobot():
	"""This will stiffen the motors and place Nao in the initial pose"""
	
	execfile("StiffnessOn.py") #Comment out this to test without robot moving (dummy values)
	execfile("PoseInit.py")


#=================================RELAX ROBOT===================================
def relaxRobot():
	"""This will make the robot sit and relax the motors"""
	
	execfile("StiffnessOff.py")


#======================ON POLICY FIRST-VISIT MONTE CARLO========================
def onPolicyFirstVisitMonteCarlo():
	"""This is the code for the policy evaluation and value function for
	on-policy monte carlo first-visit."""
	global epsilon
	global episodeNum
	global staticOptimalPolicyCount
	global strategyType
	global distancesList
	global bestDistance

	
	#---------------EPISODE GENERATION------------
	""" Using policy, generate episode by choosing keyframes
		Increase episode count and calculate epsilon"""
	
	episodeNum += 1
	epsilon = 1/(0.03*episodeNum)

	print "\n\n==========================================="
	print "==================== " + str(episodeNum) + " ===================="
	print "Greedy policy for episode " + str(episodeNum) + " is: ",
	print gPolicy
	print "(Epsilon is: " + str(epsilon) + ")\n"

	#Use e-soft to either exploit or explore
	for i in range(0, noTimeSteps):

		#Probability for explore/exploit
		randNum = random.random()

		#To explore must be bigger than 1 - greedy share: (1-e + e/|A(s)|)
		if(randNum > ((1-epsilon) + (epsilon/totalStatesPerTimeStep))):
			print "Exploring in state " + str(i+1)
			
			#Randomly choose next. Make sure don't choose exploit while exploring
			while(True):
				ePolicy[i] = random.randrange(0, totalStatesPerTimeStep)
				if (ePolicy[i] != gPolicy[i]):
					break

		#Exploit - choose greed next state
		else:
			print "Exploiting in state " + str(i+1)
			ePolicy[i] = gPolicy[i]

	
	#Print out episode info
	print "\nEpisode to be executed: "
	print ePolicy

	print "\nExecution times: "
	totalExT = 0
	for i in range(noTimeSteps):
		print str(stateTable[i][ePolicy[i]][3]) + ", ",
		totalExT += stateTable[i][ePolicy[i]][3]

	print "Total = " + str(totalExT)


	#------------------READY ROBOT----------------
	#Setup Nao for episode (comment out this while loop for continuous testing)
	while (True):
		userInput = raw_input("\nPrepare Nao for next episode? (y/n): ")

		if (userInput == 'n'):
			userInput2 = raw_input("Relax robot? (y/n): ")
			
			if (userInput2 == 'y'):
				relaxRobot()
		
		elif (userInput == 'y'):
			break

	tts.say("Preparing for episode" + str(episodeNum))
	readyRobot()
	execfile("PoseShiftRight.py")

	while (True):
		userInput = raw_input("Start episode? (y/n): ")
		if (userInput == 'y'):
			break
	
	#--------------GET KEY-FRAMES-----------------
	#If using ankle rotation, use (2) else use (1)
	rFoot = [0,0,0]			#(1)
	#rFoot = [0.25,0.7,0.2]	#(2)

	effectorList= ["LLeg"]
	space		= motion.SPACE_NAO
	axisMaskList= [63]
	isAbsolute	= False
	
	#Hold key-frames for each time-step
	pathList = [[]]
	#Holds the time point for key-frame (adds on to previous)
	timeList = [[]]

	#This hold the total execution time til the end of the last key-frame
	currentTime = 0

	#Interpolation uses start pose as origin, as such, record backswing as start
	backPose = [stateTable[0][ePolicy[0]][0],
				stateTable[0][ePolicy[0]][1],
				stateTable[0][ePolicy[0]][2],
				rFoot[0],rFoot[1],rFoot[2]]

	#Starts at the back phase, ends at front phases
	#Create the pathList and timeList needed for the SE3 interpolation
	for i in range(1, noTimeSteps):	
		next = [0,0,0,0,0,0]

		#Get each state from the current policy (normalise to origin)
		for j in range(0, 3):
			next[j] = stateTable[i][ePolicy[i]][j] - backPose[j]

		#Get time and update
		currentTime += stateTable[i][ePolicy[i]][3]
		timeList[0].append(currentTime)		

		#This moves the ankle back to original position at contact
		if i == (noTimeSteps-1):
			next[3] = 0 - backPose[3]
			next[4] = 0 - backPose[4]
			next[5] = 0 - backPose[5]

		pathList[0].append(next)


	#--------------EXECUTE EPISODE----------------
	#(1) Backswing phase
	path = [stateTable[0][ePolicy[0]][0],
			stateTable[0][ePolicy[0]][1],
			(stateTable[0][ePolicy[0]][2] - 0.02),
			rFoot[0],rFoot[1],rFoot[2]]
	motionProxy.positionInterpolation(effectorList[0], space, path, axisMaskList[0], stateTable[0][ePolicy[0]][3], isAbsolute)

	#(2) Interpolated forwardswing phase	
	motionProxy.positionInterpolations(effectorList, space, pathList, axisMaskList, timeList, isAbsolute)

	#(3) Return phase
	path = [-(stateTable[noTimeSteps-1][ePolicy[noTimeSteps-1]][0]),
		-(stateTable[noTimeSteps-1][ePolicy[noTimeSteps-1]][1]),
		(0.01-stateTable[noTimeSteps-1][ePolicy[noTimeSteps-1]][2]),
		0.0, 0.0, 0.0]
	motionProxy.positionInterpolation(effectorList[0], space, path, axisMaskList[0], 0.3, isAbsolute)

	#Return robot to initial pose
	execfile("PoseInit.py")


	#----------------GET REWARD-------------------
	"""The reward is calculated manually using the following formula:
		- For furthest kick: +2 per 10cm
		- For within range: +100 for middle, -10 per 10cm away"""

	#Get distance from episode kick
	inputDist = 0
	while True:
		try:
			inputDist = int(raw_input("Enter Distance of kick (cm), or -150 (bearing change or no kick), or -500 (fall): "))
			#Add to outcome list (if fell, don't add negative to this list)
			if inputDist < 0:
				distancesList.append(0)
			else:
				distancesList.append(inputDist) 
			break
		except ValueError:
			print "Invalid input"

	#Depending on the desired distance, calculate the reward value
	#(1) Max distance reward
	if (strategyType == 0) & (inputDist > 0):
		eReturn = round(inputDist/10)*2

		#Update if was best distance
		if inputDist > bestDistance[0]:
			tts.say("That was the best so far!")
			bestDistance[0] = inputDist
			for i in range(0, noTimeSteps):
				bestDistance[i+1] = ePolicy[i]

	#(2) Depending on how far from goal distance strategyType 1,2,3,4
	elif inputDist > 0:
		goalDist = strategyType * 100
		if (goalDist-10) <= inputDist <= (goalDist+10):
			tts.say("Perfect!")
			eReturn = 100
		elif (goalDist-20) <= inputDist <= (goalDist+20):
			eReturn = 80
		elif (goalDist-40) <= inputDist <= (goalDist+40):
			eReturn = 60
		elif (goalDist-60) <= inputDist <= (goalDist+60):
			eReturn = 40
		elif (goalDist-80) <= inputDist <= (goalDist+80):
			eReturn = 20
		else:
			eReturn = 0

		#Update if distance was closest
		if abs(goalDist - inputDist) < abs(goalDist - bestDistance[0]):
			tts.say("That was the best so far!")
			bestDistance[0] = inputDist
			for i in range(0, noTimeSteps):
				bestDistance[i+1] = ePolicy[i]

	#(3) Fall or no kick reward
	else:
		eReturn = inputDist


	#As this is MC - update all used state returns
	stateActionReturns[0][0][ePolicy[0]].append(eReturn)

	for i in range(0, noTimeSteps):
		stateReturns[i][ePolicy[i]].append(eReturn)

		if (i != 0):
			stateActionReturns[i][ePolicy[i-1]][ePolicy[i]].append(eReturn)

	
	#-------------POLICY EVALUATION---------------
	#Update value function and state-action value function
	stateActionValue[0][0][ePolicy[0]] = (sum(stateActionReturns[0][0][ePolicy[0]]) /
		len(stateActionReturns[0][0][ePolicy[0]]))

	for i in range(0, noTimeSteps):
		stateValue[i][ePolicy[i]] = (sum(stateReturns[i][ePolicy[i]]) /
			len(stateReturns[i][ePolicy[i]]))

		if (i != 0):
			stateActionValue[i][ePolicy[i-1]][ePolicy[i]] = (sum(stateActionReturns[i][ePolicy[i-1]][ePolicy[i]]) /
				len(stateActionReturns[i][ePolicy[i-1]][ePolicy[i]]))
	

	#------------POLICY IMPROVEMENT---------------
	#Set actions dictated by greed policy (set initial values to first in saValue table)
	
	#Record previous gPolicy for later reference
	previousGPolicy = gPolicy[:]
	

	#(1) update first time-step
	maxValue = stateActionValue[0][0][0]
	count = 0

	for saVal in stateActionValue[0][0]:
		if (saVal > maxValue):
			gPolicy[0] = count
			maxValue = saVal
		count += 1

	#(2) Update all other time-steps based on previous time-step
	for i in range(1, noTimeSteps):
		maxValue = stateActionValue[i][gPolicy[i-1]][0]
		count = 0

		#Follow from the first greedy choice through the table
		for saVal in stateActionValue[i][gPolicy[i-1]]:
				if (saVal > maxValue):
					gPolicy[i] = count
					maxValue = saVal
				count += 1

	#Check if gPolicy has changed
	if (previousGPolicy == gPolicy):
		staticOptimalPolicyCount += 1
	else:
		staticOptimalPolicyCount = 0

	#------------------OUTPUT---------------------
	#Print table and values
	print "\n---------State value function table--------"
	print stateValue
	print "\n-----State-action value function table-----"
	print stateActionValue
	print "\n-----------Best Kick Thus Far--------------"
	print "Distance: " + str(bestDistance[0]) + "cm,   from policy: [",
	for i in range(0, noTimeSteps):
		print str(bestDistance[i+1]),
	print "]"
	print "\n-------------------------------------------"


#===============================MAIN============================================

#Get the distance we wish to optimise for:
while True:
	try:
		strategyType = int(raw_input("What distance are we optimising for?\nEnter 0 for furthest possible, or 1-5 for respective distance in metres: "))
		break
	except ValueError:
		print "Invalid input"

while (episodeNum < noEpisodes):
	onPolicyFirstVisitMonteCarlo()

#Stand down and print state
tts.say("Test Complete")
print "=========================================="
print "Final read-outs:"
printCurrentState()
print "=========================================="
print "Optimal policy stable for " + str(staticOptimalPolicyCount) + " episodes."
print "Best distance (cm) and policy: ",
print bestDistance
print
print "=========================================="
print "Optimal policy: ",
print gPolicy
print
print "Optimal Policy key-frames:"
for i in range(0, noTimeSteps):
	print stateTable[i][gPolicy[i]]
print

print "=========================================="
print "Distance List"
print distancesList

#Plot heatmap of kick
resultsPlotter(noTimeSteps, noExTimesPerState, noStatesPerTimeStep, stateActionValue)

tts.say("Standing down")
relaxRobot()