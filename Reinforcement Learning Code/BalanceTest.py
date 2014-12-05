from __future__ import division
from Config import *
import motion
import time
import math
import sys
#Author: Sean Wilson s0831408

#----------------------------------------------------------
#Test leg positions
TO_RAD = math.pi/180
motionProxy = ALProxy("ALMotion", IP, PORT)
memoryProxy = ALProxy("ALMemory", IP, PORT)
useSensors = False

space      = motion.SPACE_NAO
axisMask   = 63                     # control all the effector's axes
isAbsolute = False

times	= 1.5	#Time between points


#======================Print Pressure Sensor Readings===========================

def getBalance():
	# Get The Right Foot Force Sensor Values
	RFsrFL = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value")
	RFsrFR = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value")
	RFsrBL = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value")
	RFsrBR = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value")

	right = RFsrFR + RFsrBR
	left = RFsrFL + RFsrBL
	dif = abs(RfootR - RfootL)

	print("Right side [kg]: %.2f,  Left side [kg]: %.2f,  Difference: %.2f" % (right, left, dif))

#=================================MAIN==========================================

execfile("StiffnessOn.py") 	#Comment out this to test without robot moving (dummy values)
execfile("PoseInit.py")		#Stand
execfile("PoseShiftRight.py")#Shift weight to right leg

pose = [0,0,0,0,0,0]

while (True): 
	shiftY 	= 0.0	#pos -> left		neg -> right
	shiftZ 	= 0.0	#pos -> up			neg -> down

	userInput = raw_input("specify torso movement (y or z): ")
	if (userInput == 'y'):
			userInput = raw_input("move size: ")
			shiftY = float(userInput)
			pose[1] += shiftY
	elif (userInput == 'z'):
			userInput = raw_input("move size: ")
			shiftZ = float(userInput)
			pose[2] += shiftZ

	effector   = "Torso" 
	path       = [0, shiftY, shiftZ, 0, 0, 0]
	motionProxy.positionInterpolation(effector, space, path, axisMask, times, isAbsolute)

	print pose
	getBalance()

	userInput = raw_input("Reset robot? (y/n): ")
	if (userInput == 'y'):
		execfile("PoseInit.py")
		execfile("PoseShiftRight.py")
		pose = [0,0,0,0,0,0]

	userInput = raw_input("Move again? (y/n): ")
	if (userInput == 'n'):
		break
		
execfile("PoseInit.py")