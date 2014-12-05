#Author: Sean Wilson s0831408

from __future__ import division
#=============ENVIRONMENT VARIABLES===============
"""Here we set up the grid test environment for testing:
The grid of actions is a representation of the x-plane for the foot end
effector. 
[0,0] is back bottom
[noTimeSteps][noStatesPerTimeStep*noExTimesPerState] is top forward leg position
We set the environment 
"""

#Limits of foot motion	(x-forward/backwards, y-left/right, z-up/down)
minX = -0.05#-0.08 MAX
maxX = 0.1
minY = 0
maxY = 0
minZ = 0.02
maxZ = 0.06
#This is the x-value for contact point, will be used as middle state (18mm)
midX = 0.018

"""#The following limits are used when ankle rotation is included:
minX = -0.12
maxX = 0.1
minY = 0
maxY = 0
minZ = 0.055
maxZ = 0.08
#This is the x-value for contact point, will be used as middle state (18mm)
midX = 0.02"""


#Limits of execution time
minExTime = 0.1
maxExTime = 0.3

#Set no states per time-step and number of time intervals
noTimeSteps = 3 #CAN ONLY HAVE 2 or 3 (linear or 3 point spline)
noStatesPerTimeStep = 5	
noExTimesPerState 	= 5 
totalStatesPerTimeStep = noStatesPerTimeStep * noExTimesPerState


#Calculate the middle value for y given midX
percentOfMax = (midX-minX) / (maxX-minX)
midY = ((maxY - minY) * percentOfMax) + minY

#Calculate step-size of z
zStep = (maxZ - minZ) / (noStatesPerTimeStep-1)

#Calculate time steps
if (noExTimesPerState > 1):
	exTimeStep = (maxExTime - minExTime) / (noExTimesPerState-1)
else:
	#Set the default time if only one time-step per state here
	exTimeStep = 0.3

stateTable 			= []
stateValue 			= []
stateReturns 		= []
stateActionValue 	= []
stateActionReturns 	= []
gPolicy 			= []
ePolicy 			= []


#===================INITIALISE THE TABLES TO SET SIZES==========================

def setupEnvironment():
	#Add the first state-action col and state-action return col. Needs to be double 
	#brackets as it is one time-step. Inner bracket is current state (only one start)
	stateActionValueCol 	= [[]]
	stateActionReturnsCol 	= [[]]

	#Create the first rows of the state-actions - they only have one start state
	for i in range(0, totalStatesPerTimeStep):
		stateActionValueCol[0].append(0.0)
		stateActionReturnsCol[0].append([])

	stateActionValue.append(stateActionValueCol)
	stateActionReturns.append(stateActionReturnsCol)


	#Create the	rest of the tables
	for timeStep in range(0, noTimeSteps):
		#Create the tables columns (a single time step)
		stateValueCol 		= []
		stateReturnsCol 	= []
		stateActionValueCol	= []
		stateActionReturnsCol = []

		for i in range(0, noStatesPerTimeStep):
			
			#For each time-state, add space for its value and returns
			for j in range(0, noExTimesPerState):
				stateValueCol.append(0.0)
				stateReturnsCol.append([])

				#For each state, add a transition to every possible next state
				stateActionValueState 	= []
				stateActionReturnsState = []
				for k in range(0, totalStatesPerTimeStep):
					stateActionValueState.append(0.0)
					stateActionReturnsState.append([])
				stateActionValueCol.append(stateActionValueState)
				stateActionReturnsCol.append(stateActionReturnsState)


		#Add this time step to final tables
		stateValue.append(stateValueCol)
		stateReturns.append(stateReturnsCol)

		#Also set the policy starts
		gPolicy.append(0)
		ePolicy.append(0)

		#Remember, action-value pairs start from one state (previouly added first)
		if timeStep != 0:
			stateActionValue.append(stateActionValueCol)
			stateActionReturns.append(stateActionReturnsCol)
	
	#-------------------------SET-UP STATE TABLE--------------------------------
	
	#Add back time-step	
	stateTableCol 		= []
	for i in range(0, noStatesPerTimeStep):
		for j in range(0, noExTimesPerState):
			stateTableCol.append([float(minX),
								float(minY),
								float(minZ + (zStep * i)),
								float(minExTime + (exTimeStep * j))])
	stateTable.append(stateTableCol)

	#Add mid time-step (if time-step count is 3)
	if noTimeSteps == 3:
		stateTableCol 		= []
		for i in range(0, noStatesPerTimeStep):
			for j in range(0, noExTimesPerState):
				stateTableCol.append([float(midX),
									float(midY),
									float(minZ + (zStep * i)),
									float(minExTime + (exTimeStep * j))])
		stateTable.append(stateTableCol)

	#Add Forward time-step
	stateTableCol 		= []
	for i in range(0, noStatesPerTimeStep):
		for j in range(0, noExTimesPerState):
			stateTableCol.append([float(maxX),
								float(maxY),
								float(minZ + (zStep * i)),
								float(minExTime + (exTimeStep * j))])
	stateTable.append(stateTableCol)

#==========================PRINT CURRENT STATE==================================
def printCurrentState():
	"""This method prints out all of the tables	for checking the workings"""

	print "==================POLICIES===================\n"
	print "Greedy Policy:"
	print gPolicy
	print

	print "Current Policy:"
	print ePolicy
	print

	print "=================STATE TABLE=================\n"
	print "State Table:"
	print stateTable
	print

	print "==============VALUE FUNCTION=================\n"
	print "Value Function:"
	print stateValue
	print

	print "==============STATE-RETURNS==================\n"
	print "State Returns:"
	print stateReturns
	print

	print "===========STATE-ACTION VALUE================\n"
	print "State-action Values:"
	print stateActionValue
	print

	print "==========STATE-ACTION RETURNS===============\n"
	print "State-action Returns:"
	print stateActionReturns
	print

setupEnvironment()
printCurrentState()
