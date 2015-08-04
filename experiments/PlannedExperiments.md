# Planned experiments for Ranger

## Determine effective ankle joint inertia
There is a good model for the ankle-joint motor and transmission, which gives the torque on the joint as a function of the joint state and motor current. There is no model that relates the joint torque back to the joint acceleration.

#### Possible procedure:  (run for both inside and outside ankle motors)
- write some code for the ankle motors to track a sine-wave reference (thus angle rate should be a cosine)
- The amplitude of the wave should be roughly from the upper limit (flip-up) to lower limit (push-off).
- The frequency should be constant for each trial, with maybe 5 trials. The frequencies for each trial should be something like:
	- 0.500 Hz   
	- 0.748 Hz   
	- 1.118 Hz     
	- 1.672 Hz 
	- 2.500 Hz 
- For each trial, log the motor current, and joint angle.
- Smooth the data, and differentiate twice to find acceleration
- Use the motor model to find the torque expected at the joint
- Use least squares to fit:  (or something like it)
		M*ddq + C*dq + K*(q-qRef) = torque
		
