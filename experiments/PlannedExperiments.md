# Planned experiments for Ranger

## Motor model validation
I want to check the motor model for Ranger, to make sure that the paramters are reasonable. For example, I'm suspicious that the linear friction term is zero in Pranav's model.

#### Possible procedure:  (do for each motor)
- Generate a large data set of joint angles and motor currents.
	- Just run a series of step changes in the target angle, using the existing controllers. Randomly sample the target set-point from a uniform distrubition between the safe bounds of the actuator.
- Compute the joint velocity and acceleration by smoothing and then differentiation
- Use CMAES or some other optimizer to find the parameters that best match the data.
	- Need to know the joint inertia, or include it as a parameter, since we don't have direct torque sensing.
	- Minimize defects in the predicted vs measured acceleration.

## Motor current limits
The current limits in the motors are not simple. First, they saturate all commands at +- 8 amps. Then they shut off the motors if they exceed some safe operating temperature. This means that there is a hidden internal state that the simulator doesn't know about. One solution is to just assume that there is a time-dependent current limit. For practical purposes we can just limit the maximum current to a value that is safe for a period of a few seconds. The question is: what is that value?

#### Possible procedure:
- assume that 3 seconds is an acceptable time to sit at the max current
- let the motor cool off
- apply some small test current
- record when the motor shuts off, or 3 seconds, whichever happens first
- repeat with an increased current.
- Record the highest command current that the robot can safely maintain for 3 seconds. I expect it to be around 5 amps.