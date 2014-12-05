from Config import *
import motion
import time
motionProxy = ALProxy("ALMotion", IP, PORT)

import math
TO_RAD = math.pi/180

space      = motion.SPACE_NAO
axisMask   = 63                     # control all the effector's axes
isAbsolute = False

### Lower the Torso and move to the side
# Move the arms so they are straight downwards
times     	   = 1.5                  # duration of interpolation in seconds
LeftArm  = [90,  25, -90, -10]    # shoulder pitch/roll, elbow yaw/roll
LeftArm = [ x * TO_RAD for x in LeftArm]

RightArm = [90, -25,  90,  10]
RightArm = [ x * TO_RAD for x in RightArm]

motionProxy.post.angleInterpolation(["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll"], LeftArm+RightArm, times, True) # "post" means non-blocking, so...

# move torso at the same time
effector   = "Torso"
path       = [0.0, -0.08, -0.02, 0.0, 0.0, 0.0]
motionProxy.positionInterpolation(effector, space, path, axisMask, times, isAbsolute)

### Swing left leg back
# Move left arm to the front, right arm to the back
times      = 0.5     
effector   = "LLeg"         
LeftArm  = [61,  25, -90, -10]
LeftArm = [ x * TO_RAD for x in LeftArm]

RightArm = [119, -25,  90,  10]
RightArm = [ x * TO_RAD for x in RightArm]

motionProxy.post.angleInterpolation(["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll"], LeftArm+RightArm, times, True)

path       = [-0.10, 0.00, 0.04, 0.0, 0.0, 0.0]

motionProxy.positionInterpolation(effector, space, path,
                                      axisMask, times, isAbsolute)
                                      



### Swing left leg forward
# Move left arm all the way back, right arm to the front (but not all the way)
times      = 0.3              
LeftArm  = [119,  25, -90, -10]
LeftArm = [ x * TO_RAD for x in LeftArm]

RightArm = [61, -25,  90,  10]
RightArm = [ x * TO_RAD for x in RightArm]

motionProxy.post.angleInterpolation(["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll"], LeftArm+RightArm, times, True)

path       = [0.20, 0.0, 0, 0.0, 0.0, 0]

motionProxy.positionInterpolation(effector, space, path,
                                      axisMask, times, isAbsolute)

### Move leg back to initial position
# Move arms so they are facing straight down again
times      = 1.0                
LeftArm  = [90,  25, -90, -10]
LeftArm = [ x * TO_RAD for x in LeftArm]

RightArm = [90, -25,  90,  10]
RightArm = [ x * TO_RAD for x in RightArm]

motionProxy.post.angleInterpolation(["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll"], LeftArm+RightArm, times, True)

path       = [-0.10, 0.0, -0.04, 0.0, 0.0, 0]

motionProxy.positionInterpolation(effector, space, path,
                                      axisMask, times, isAbsolute)
                                      
# Raise torso again
effector   = "Torso"
times      = 1.0                                                       

path       = [0.0, 0.08, 0.02, 0.0, 0.0, 0.0]

motionProxy.positionInterpolation(effector, space, path, axisMask, times, isAbsolute)
from motion_PoseInit import *
