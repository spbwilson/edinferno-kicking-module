from Config import *
import math


# This file places the robot into the initial pose

motionProxy = ALProxy("ALMotion", IP, PORT)
TO_RAD = math.pi/180

# Angle values
kneeAngle  = 40
torsoAngle =  0
wideAngle  =  0

#----------------------------- prepare the angles -----------------------------
# Get the Number of Joints
NumJoints = len(motionProxy.getJointNames("Body"))

# Define The Initial Position
Head     = [0, 0]
LeftArm  = [120,  15, -90, -80]
LeftLeg  = [0,  wideAngle, -kneeAngle/2-torsoAngle, kneeAngle, -kneeAngle/2, -wideAngle]
RightLeg = [0, -wideAngle, -kneeAngle/2-torsoAngle, kneeAngle, -kneeAngle/2,  wideAngle]
RightArm = [120, -15,  90,  80]

# If we have hands, we need to add angles for wrist and hand
if (NumJoints == 26):
          LeftArm  += [0, 0]
          RightArm += [0, 0]

# Gather the joints together
pTargetAngles = Head + LeftArm + LeftLeg + RightLeg + RightArm

# Convert to radians
pTargetAngles = [ x * TO_RAD for x in pTargetAngles]

#------------------------------ send the commands -----------------------------

# Display the starting position
#print motionProxy.getSummary()

# We use the "Body" name to signify the collection of all joints
pNames = "Body"

# We set the fraction of max speed
pMaxSpeedFraction = 0.2

# Ask motion to do this with a blocking call
motionProxy.angleInterpolationWithSpeed(pNames, pTargetAngles, pMaxSpeedFraction)

# Display the end position
#print motionProxy.getSummary()

motionProxy.closeHand('RHand')
