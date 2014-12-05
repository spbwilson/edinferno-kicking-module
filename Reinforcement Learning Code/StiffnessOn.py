from Config import *
motionProxy = ALProxy("ALMotion", IP, PORT)

#set body stiffness to 1.0
names  = 'Body'
stiffness  = 1.0
time  = 1.0
motionProxy.stiffnessInterpolation(names, stiffness, time)

print "Body Stiffness set to", stiffness
