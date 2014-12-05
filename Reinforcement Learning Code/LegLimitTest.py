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
	# Get The Left Foot Force Sensor Values
	LFsrFL = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value")
	LFsrFR = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value")
	LFsrBL = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value")
	LFsrBR = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value")

	print( "Left FSR [Kg] : %.2f %.2f %.2f %.2f" %  (LFsrFL, LFsrFR, LFsrBL, LFsrBR) )

	# Get The Right Foot Force Sensor Values
	RFsrFL = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value")
	RFsrFR = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value")
	RFsrBL = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value")
	RFsrBR = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value")

	print( "Right FSR [Kg] : %.2f %.2f %.2f %.2f" %  (RFsrFL, RFsrFR, RFsrBL, RFsrBR) )


#=================================MAIN==========================================

execfile("StiffnessOn.py") 	#Comment out this to test without robot moving (dummy values)
execfile("PoseInit.py")		#Stand
execfile("PoseShiftRight.py")#Shift weight to right leg

pose = [0,0,0,0,0,0]
while (True): 

	shiftX 	= 0.0	#pos -> forward		neg -> backward
	shiftY 	= 0.0	#pos -> left		neg -> right
	shiftZ 	= 0.0	#pos -> up			neg -> down
	rotX 	= 0.0	#pos -> roll acw	neg -> roll cw 
	rotY 	= 0.0	#pos -> pitch down	neg -> pitch up
	rotZ 	= 0.0	#pos -> twist out	neg -> twist in

	userInput = raw_input("specify which coord to change: ")
	if (userInput == 'x'):
		userInput = raw_input("move size: ")
		shiftX = float(userInput)
		pose[0] += shiftX
	elif (userInput == 'y'):
		userInput = raw_input("move size: ")
		shiftY = float(userInput)
		pose[1] += shiftY
	elif (userInput == 'z'):
		userInput = raw_input("move size: ")
		shiftZ = float(userInput)
		pose[2] += shiftZ
	elif (userInput == 'rx'):
		userInput = raw_input("move size: ")
		rotX = float(userInput)
		pose[3] += rotX
	elif (userInput == 'ry'):
		userInput = raw_input("move size: ")
		rotY = float(userInput)
		pose[4] += rotY
	elif (userInput == 'rz'):
		userInput = raw_input("move size: ")
		rotZ = float(userInput)
		pose[5] += rotZ

	effector   = "LLeg" 
	path       = [shiftX, shiftY, shiftZ, rotX, rotY, rotZ]
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
