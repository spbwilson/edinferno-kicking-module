Author: Sean Wilson (s0831408)
besiix@gmail.com

=========================KICKING ENGINE=======================
This folder hold all the code necessary to run the Python built
kicking engine. See dissertation for further explination.

INPUTS:
distance - must be a number representing desired distance in cm
displacment - the distance (cm) displacement (laterally) from the
	centre of the robot. (-left +right)
angle - the angle in degrees for the desired kick (-left +right)


==================REINFORCEMENT LEARING CODE==================
This hold all the required code to build a state-action environment
for the nao robot. The user must specify the distance to optimise.
There are two RL algorithms implemented:
- Qlearning
- Monte Carlo

There is also a limit module to incrementally move nao's joints.
Similarly there is a balance module to test for nao's balance point.

A Plotting algorithm is included for making heat maps of the 
average state-values resulting from the RL algorithms

We also include sample kicking motions for the benchmark used in
the dissertation, and the Edinferno basic straight kick