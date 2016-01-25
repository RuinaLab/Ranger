# How to set up parameters for Ranger

All parameters for Ranger must be updated and set using a series of spreadsheets. A Matlab script then runs through the spreadsheets and automatically generates header files. The header files should never be changed manually.

These files are located in the directory:
RangerCode\Ranger\Control\Ranger_Configuration\

## Header and source files that are automatically generated:
- board_id.h
- error_id.h
- can_id.h
- init_txlist.h
- init_rxlist.h
- io_data.c
- error_messages.h
- csr_routing_table.c

## ranger_parser.m
This is the Matlab script that automatically generates the code above, using the three tables below for input.

## board_table.csv
This table assigns enums for communication. For practical purposes, it should never be changed.

## error_table.csv
Holds information for doing error handling. Do not delete any old entries (lines 1-114), but you can add new ones if you are careful.

## can_table.csv
This is the important one to modify! But be careful. Notice that the CAN IDs are divided into sections by board. Only edit parameters related to the main brain. We do not want to recompile code to the motor control or UI boards. In other words, only edit lines after 191.

#### columns in can_table.csv
1. Parameter name. This should be accessible in the main code using set_io_float() and get_io_float().
2. Default value - this is hard coded into the automatically generated code. Must run the parsing script (ranger_parser.m) for them to take effect!
3. While running, where does this parameter get generated in the code? This is used for the CAN router, and the numbers can be found in board_table.csv. 
4. While running, where should this parameter be sent? This is used for the CAN router, and the numbers can be found in board_table.csv.
5. Comments

If you want to be able to adjust parameters while the robot is running, then you must set the source to 0 and the destination to 1. In other words: the parameter is set by the labview board and received by the main brain.