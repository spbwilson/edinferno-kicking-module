from Config import *
import motion
import time
import math
import sys
#Author: Sean Wilson s0831408

# This script will set the robot to a leaning position
# The robot's weight is centred over the right leg allowing for left kicks

TO_RAD = math.pi/180
motionProxy = ALProxy("ALMotion", IP, PORT)
useSensors = False

space      = motion.SPACE_NAO
axisMask   = 63                     # control all the effector's axes
isAbsolute = False

times = 0.8
LeftArm  = [90,  25, -90, -10]		# shoulder pitch/roll, elbow yaw/roll
LeftArm	 = [ x * TO_RAD for x in LeftArm]

RightArm = [90, -25,  90,  10]
RightArm = [ x * TO_RAD for x in RightArm]

motionProxy.post.angleInterpolation(["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll"], LeftArm+RightArm, times, True) # "post" means non-blocking, so...

effector   = "Torso"
path       = [0.0, -0.07, -0.01, 0.0, 0.0, 0.0]
motionProxy.positionInterpolation(effector, space, path, axisMask, 0.6, isAbsolute)

effector   = "LLeg"
path       = [0.0, 0.0, 0.02, 0.0, 0.0, 0.0] #0.055 (ankle rotation)
motionProxy.positionInterpolation(effector, space, path, axisMask, 0.2, isAbsolute)
