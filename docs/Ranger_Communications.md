# Ranger Communications:
There are two ways to communicate with the robot while it is running. The first is with a serial port, and the second is over bluetooth wireless. The serial port should be about 4 times faster and have more consistent rate, but you'll need to be connected to a laptop with a wire.

## Serial Port:
0. open `mb_software_setup.h`
0. ensure that lines 4-7 read like this:
		//#define DEBUG
		#define BT_HSU1_SERIAL_PORT //Data collection via serial port (comment line below)
		//#define BT_HSU2_BLUETOOTH //Data collection via blue-tooth (comment earlier line)
0. compile program and flash to robot
0. open the labview data acquisition program: `RoboDAQ7.vi`
0. Check that you have the right serial port (guess and check)
0. Set the Baud Rate to be 921600
0. Run the labview program and it should work!

## Blue Tooth:
0. open `mb_software_setup.h`
0. ensure that lines 4-7 read like this:
		//#define DEBUG
		//#define BT_HSU1_SERIAL_PORT //Data collection via serial port (comment line below)
		#define BT_HSU2_BLUETOOTH //Data collection via blue-tooth (comment earlier line)
0. compile program and flash to robot
0. open the labview data acquisition program: `RoboDAQ7.vi`
0. Check that you have the right serial port (guess and check)
0. Set the Baud Rate to be 460800
0. Run the labview program and it should work!
