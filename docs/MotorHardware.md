# Motor Hardware

## Motor Current limits:
- The motor controllers will programatically saturate any current commands that are more than 8 amps.
- The motors are rated for 3.1 amps continuously
- The motor controllers continuously monitor the temperature. If the temperature becomes too high, they shut down the board. If this happens, an error message will be sent.
- The thermal model assumes that the robot is at room temperature when it starts to calibrate the sensor. If the motors are hot and you restart the robot, you may get funny behaviors.

## Motor Transmission limits:
- The planetary gearboxes in Ranger can output 4.5 Nm continuously, or 6.0 Nm maximum
- The hip gearbox ratio is 66:1
- The ankle gearbox ratio is 43:1, but there is an additional pulley transmission. The overall ratio is 34:1.
	- This means that the torque limits for the ankle joint are 3.6 Nm continuous and 4.8Nm peak, measured at the foot.
- Both gearboxes have a rated efficiency of 70%