#Author: Sean Wilson s0831408
from __future__ import division
from Config import *
import motion
import time
import math
import random
import sys

motionProxy = ALProxy("ALMotion", IP, PORT)

#=============================GET KEY FRAMES====================================
def getKeyFrames(kickDist):
	"""This takes an integer which corresponds to the kick type:
			(0) - Straight, as far as can
			(1) - Straight, 1 metre
			(2) - Straight, 2 metres
			(3) - Straight, 3 metres
			(4) - Straight, 4 metres
		Each kick defines the backswing, middle point, and contact point. The 
		both distribution phases and the leg return are the same for all kicks.
	"""
	if (kickDist == 0) | (kickDist > 4):
		keyFrames = [[[-0.05, 0.0, 0.05, 0, 0, 0],[0.018, 0.0, 0.04, 0, 0, 0],[0.1, 0.0, 0.03, 0, 0, 0]]]

		intTimes = [[0.1, 0.1, 0.1]]
		return keyFrames, intTimes

	elif kickDist == 1:
		keyFrames = [[[-0.02, 0.0, 0.05, 0, 0, 0],[0.018, 0.0, 0.02, 0, 0, 0],[0.1, 0.0, 0.05, 0, 0, 0]]]
		intTimes = [[0.15, 0.1, 0.25]]
		return keyFrames, intTimes

	elif kickDist == 2:
		keyFrames = [[[-0.02, 0.0, 0.06, 0, 0, 0],[0.018, 0.0, 0.03, 0, 0, 0],[0.1, 0.0, 0.06, 0, 0, 0]]]
		intTimes = [[0.1,0.15,0.15]]
		return keyFrames, intTimes

	elif kickDist == 3:
		keyFrames = [[[-0.05, 0.0, 0.04, 0, 0, 0],[0.018, 0.0, 0.02, 0, 0, 0],[0.1, 0.0, 0.03, 0, 0, 0]]]
		intTimes = [[0.1,0.2,0.1]] 
		return keyFrames, intTimes

	elif kickDist == 4:
		keyFrames = [[[-0.05, 0.0, 0.05, 0, 0, 0],[0.018, 0.0, 0.06, 0, 0, 0],[0.1, 0.0, 0.03, 0, 0, 0]]]
		intTimes = [[0.1,0.2,0.1]] 
		return keyFrames, intTimes


#=================================UPDATE Y======================================
def updateY(keyFrames,dispMove, angle, foot):

	inLimit = 0.02	 #Limit for moving foot towards centre of bot (metres)
	outLimit = 0.023 #Limit for moving foot away from centre of bot (metres)

	#Calculate angle move from degreees given (moves foot y-value in cm)
	if abs(angle) < 15:
		angleMove = 0
	elif abs(angle) < 25:
		angleMove = 0.01
	elif abs(angle) < 35:
		angleMove = 0.02
	else:
		angleMove = outLimit

	#If angled to left, need to move foot to right (this is a neg move in either foot)
	if (angle < 0):
		angleMove = -angleMove

	#Add angleMove and dispMove, ensure not over limit
	move = dispMove + angleMove

	#Ensure foot doesn't go out of range limits
	if (foot == 0) & (move > outLimit):
		move = outLimit
	elif (foot == 0) & (move < -inLimit):
		move = -inLimit
	elif (foot == 1) & (move > inLimit):
		move = inLimit
	elif (foot == 1) & (move < -outLimit):
		move = -outLimit

	#Take key-frames, add update all y-values for desired trajectory
	for i in range(len(keyFrames[0])):
		keyFrames[0][i][1] += move

	return keyFrames


#===============================EXECUTE KICK====================================
def executeKick(distance, angle, ballDisp):
	"""Things look a little complicated because we must split up the motion into
		backswing, frontswing, and return as we only want spline interpolation on
		front swing.
		Each motion starts where the last ends as it's orientation. As we 
		specify all key-frames from initial pose, This must be translated. 
		Simple, but looks a bit messy.
	"""
	
	footCentreDisp = 0.05 #The displacement from centre of bot to centre of foot

	#----------------------GET AND MODIFY KEY FRAMES----------------------------
	#Get the key-frames for the distance
	keyFrames, intTimes = getKeyFrames(distance)
	print keyFrames

	#Change displacement to which ever foot is to be used
	if ballDisp >= 0:
		effectorList = ["LLeg"]
		foot = 0
		dispMove = ballDisp - footCentreDisp
	else:
		effectorList = ["RLeg"]
		foot = 1
		dispMove = ballDisp + footCentreDisp

	#Change the y-value for angle and ball Disp
	keyFrames = updateY(keyFrames, dispMove, angle, foot)
	print keyFrames

	
	#-----------------------CALCULATE INTERPOLATION-----------------------------
	
	space		= motion.SPACE_NAO
	TO_RAD 		= math.pi/180
	axisMaskList= [63]
	isAbsolute	= False
	
	#Hold key-frames for each time-step
	pathList = [[]]
	#Holds the time point for key-frame (adds on to previous)
	timeList = [[]]

	#This hold the total execution time til the end of the last key-frame
	currentTime = 0

	#Interpolation uses start pose as origin, as such, record backswing as start
	backPose = keyFrames[0][0]

	#Starts at the back phase, ends at front phases
	#Create the pathList and timeList needed for the SE3 interpolation
	for i in range(1, 3):	
		next = [0,0,0,0,0,0]

		#Normalise each key-frame to origin - backswing
		for j in range(0, 3):
			next[j] = keyFrames[0][i][j] - backPose[j]

		#Get time and update
		currentTime += intTimes[0][i]
		timeList[0].append(currentTime)		

		pathList[0].append(next)

	#----------------------------EXECUTE EPISODE--------------------------------
	#(1) Distribute weight
	LeftArm  = [90,  25, -90, -10]		# shoulder pitch/roll, elbow yaw/roll
	LeftArm	 = [ x * TO_RAD for x in LeftArm]
	RightArm = [90, -25,  90,  10]
	RightArm = [ x * TO_RAD for x in RightArm]
	motionProxy.post.angleInterpolation(["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll"], LeftArm+RightArm, 0.6, True)
	
	if foot == 0:
		path = [0.0, -0.07, -0.01, 0.0, 0.0, 0.0]
	else:
		path = [0.0, 0.07, -0.01, 0.0, 0.0, 0.0]
	motionProxy.positionInterpolation("Torso", space, path, axisMaskList[0], 0.6, isAbsolute)

	path = [0.0, 0.0, 0.02, 0.0, 0.0, 0.0]
	motionProxy.positionInterpolation(effectorList[0], space, path, axisMaskList[0], 0.2, isAbsolute)


	#(2) Backswing phase (need to take the z off as this is done above)
	path = [keyFrames[0][0][0], keyFrames[0][0][1], (keyFrames[0][0][2] - 0.02), 0, 0, 0]
	motionProxy.positionInterpolation(effectorList[0], space, path, axisMaskList[0], intTimes[0][0], isAbsolute)

	#(3) Interpolated forwardswing phase	
	motionProxy.positionInterpolations(effectorList, space, pathList, axisMaskList, timeList, isAbsolute)

	#(4) Return phase (+0.01 to z just to make sure it clears the ground)
	path = [-keyFrames[0][2][0], -keyFrames[0][2][1], (0.01-keyFrames[0][2][2]), 0.0, 0.0, 0.0]
	motionProxy.positionInterpolation(effectorList[0], space, path, axisMaskList[0], 0.3, isAbsolute)

	#(5) Stand up again
	path = [0.0, 0.0, -0.01, 0.0, 0.0, 0.0]
	motionProxy.positionInterpolation(effectorList[0], space, path, axisMaskList[0], 0.2, isAbsolute)

	if foot == 0:
		path = [0.0, 0.07, 0.01, 0.0, 0.0, 0.0]
	else:
		path = [0.0, -0.07, 0.01, 0.0, 0.0, 0.0]
	motionProxy.positionInterpolation("Torso", space, path, axisMaskList[0], 0.6, isAbsolute)
	
	#In actual execution, this will slow things down, so remove it
	execfile("PoseInit.py")


#================================MAIN===========================================
""" This is currently just a method call with user inputs. This can easily be
	adapted to a given the variables from another module by commenting out the
	following section and using: 'from KickingEngine import *'.
	Then use the method	call executeKick(distance, angle, ballDisplacement) with
	the distances in metres (0 for furthest) and the angle in degrees.
"""

while True:
	try:
		distance = float(raw_input("Distance (metres): "))
		break
	except ValueError:
		print "Invalid input"

while True:
	try:
		angle = float(raw_input("Angle, +right,-left (degrees): "))
		break
	except ValueError:
		print "Invalid input"

while True:
	try:
		ballDisp = float(raw_input("Ball displacement from centre of robot, pos=left neg=right (metres): "))
		break
	except ValueError:
		print "Invalid input"

executeKick(distance, angle, ballDisp)