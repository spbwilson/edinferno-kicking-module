#Author: Sean Wilson s0831408
from Config import *
import motion
import time
motionProxy = ALProxy("ALMotion", IP, PORT)

import math
import sys
TO_RAD = math.pi/180

#----------------------------------------------------------
#This is a a quick straight shot
#Time: 1.54 sec

print "# HY HP LSP LSR LEY LER LHYP LHR LHP LKP LAP LAR RHR RHP RKP RAP RAR RSP RSR REY RER DUR"
useSensors = False

space      = motion.SPACE_NAO
axisMask   = 63                     # control all the effector's axes
isAbsolute = False

#---------Lower the Torso and move to the side-------------
# Move the arms so they are straight downwards
times     	   = 0.6			# duration of interpolation in seconds
LeftArm  = [90,  25, -90, -10]	# shoulder pitch/roll, elbow yaw/roll
LeftArm = [ x * TO_RAD for x in LeftArm]

RightArm = [90, -25,  90,  10]
RightArm = [ x * TO_RAD for x in RightArm]

motionProxy.post.angleInterpolation(["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll"], LeftArm+RightArm, times, True) # "post" means non-blocking, so...

effector   = "Torso"
path       = [0.0, 0.07, -0.01, 0.0, 0.0, 0.0]
motionProxy.positionInterpolation(effector, space, path, axisMask, times, isAbsolute)

        
#------------Lift and swing right leg forward--------------
# Move right arm all the way back, left arm to the front (but not all the way)
times      = 0.17         
LeftArm  = [61,  25, -90, -10]
LeftArm = [ x * TO_RAD for x in LeftArm]

RightArm = [119, -25,  90,  10]
RightArm = [ x * TO_RAD for x in RightArm]

motionProxy.post.angleInterpolation(["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll"], LeftArm+RightArm, times, True)

effector   = "RLeg" 
path       = [0.135, 0.0, 0.06, 0.0, 0.0, 0]
motionProxy.positionInterpolation(effector, space, path, axisMask, times, isAbsolute)

#----------Move leg back to initial position---------------
# Move arms so they are facing straight down again
times      = 0.17               
LeftArm  = [90,  25, -90, -10]
LeftArm = [ x * TO_RAD for x in LeftArm]

RightArm = [90, -25,  90,  10]
RightArm = [ x * TO_RAD for x in RightArm]

motionProxy.post.angleInterpolation(["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll"], LeftArm+RightArm, times, True)

effector   = "RLeg"
path       = [-0.135, 0.0, -0.06, 0.0, 0.0, 0]
motionProxy.positionInterpolation(effector, space, path, axisMask, times, isAbsolute)
                 
#-----------------Raise torso again------------------------
times      = 0.6    
effector   = "Torso"                                       
path       = [0.0, -0.07, 0.01, 0.0, 0.0, 0.0]
motionProxy.positionInterpolation(effector, space, path, axisMask, times, isAbsolute)

from motion_PoseInit import *
