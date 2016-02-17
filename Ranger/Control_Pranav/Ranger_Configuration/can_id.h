//
//CAN_ID Table for Cornell Ranger Robot
//Automatically generated by ranger_parser.m
//	 use the ranger_parser.m MATLAB script to make any changes to can_id.h, board_id.h, error_id.h, init_txlist.h, init_rxlist.h, io_data.c, error_messages.h or csr_routing_table.c
//	 the script is located in ...\SVN\Trunk\Ranger\Control\Ranger_Configuration
//

#ifndef __H_CAN_ID__
#define __H_CAN_ID__ 

typedef enum can_ids{
	ID_TIMESTAMP = 0,	 // 0 2 1 // Timestamp // time since turning on of the robot in milliseconds
	ID_ERROR_MB = 1,	 // 0 1 0 // 
	ID_ERROR_CSR = 2,	 // 0 2 1 // 
	ID_ERROR_MCH = 3,	 // 0 3 1 // 
	ID_ERROR_MCFO = 4,	 // 0 4 1 // 
	ID_ERROR_MCFI = 5,	 // 0 5 1 // 
	ID_ERROR_MCSO = 6,	 // 0 6 1 // 
	ID_ERROR_MCSI = 7,	 // 0 7 1 // 
	ID_ERROR_UI = 8,	 // 0 8 1 // 
	ID_MB_STATUS = 9,	 // 0 1 3;4;5;6;7;8;9 // 
	ID_MCH_MOTOR_VELOCITY = 10,	 // 0 3 1 // angular rate (rad/sec) of hip motor
	ID_MCH_MOTOR_CURRENT = 11,	 // 0 3 1 // hip motor current
	ID_MCH_MOTOR_POSITION = 12,	 // 0 3 1 // angle in radians of the hip motor
	ID_MCH_MOTOR_TARGET_CURRENT = 13,	 // 0 1 3 // 
	ID_MCH_ANGLE = 14,	 // 0 3 1 // angle in radians of the hip joint (difference between this and the hip motor angle is the gear back lash)
	ID_MCH_BATT_POWER = 15,	 // 0 3 1 // power consumed by the hip board. 
	ID_MCH_SHUTDOWN = 16,	 // 0 1 3 // 1 = turn off the hip motor
	ID_MCH_SLEEP = 17,	 // 0 1 3 // 
	ID_MCH_COMMAND_CURRENT = 18,	 // 0 1 3 // desired current to the hip motor = ID_MCH_COMMAND_CURRENT+ ID_MCH_STIFFNESS*hip_angle + ID_MCH_DAMPNESS*hip_angle_rate 
	ID_MCH_STIFFNESS = 19,	 // 0 1 3 // see above (ID_MCH_COMMAND_CURRENT)
	ID_MCH_DAMPNESS = 20,	 // 0 1 3 // see above (ID_MCH_COMMAND_CURRENT)
	ID_MCH_ANG_RATE = 21,	 // 0 3 1 // slightly filtered derivative of ID_MCH_ANGLE
	ID_MCH_STATUS = 22,	 // 0 3 1 // 
	ID_MCH_EXECUTION_TIME = 23,	 // 0 3 1 // time taken to execute all the functions in one row of the scheduler (software_setup.c in hip board) ( should be less than the tick time of the board; eg 0.5 ms for hip motor board)
	ID_MCH_MAX_EXECUTION_TIME = 24,	 // 0 3 1 // maximum recorded value of the ID_MCH_EXECUTION_TIME since the start of the robot
	ID_MCH_EMPTY_TX1 = 25,	 // 0 1 3 // 
	ID_MCH_EMPTY_TX2 = 26,	 // 0 1 3 // 
	ID_MCH_BATT_CURRENT = 27,	 // 0 3 1 // current in the hip board
	ID_MCH_BATT_VOLTAGE = 28,	 // 0 3 1 // 
	ID_MCH_HBRIDGE_TEMP = 29,	 // 0 3 1 // 
	ID_MCH_EMPTY_RX1 = 30,	 // 0 3 1 // 
	ID_MCH_EMPTY_RX2 = 31,	 // 0 3 1 // 
	ID_MCH_EMPTY_RX3 = 32,	 // 0 3 1 // 
	ID_MCFO_MOTOR_POSITION = 33,	 // 0 4 1 // can_id for Outer Feet board -- check similar can_id; starting with MCH
	ID_MCFO_MOTOR_VELOCITY = 34,	 // 0 4 1 // can_id for Outer Feet board -- check similar can_id; starting with MCH
	ID_MCFO_MOTOR_CURRENT = 35,	 // 0 4 1 // can_id for Outer Feet board-- check similar can_id; starting with MCH
	ID_MCFO_MOTOR_TARGET_CURRENT = 36,	 // 0 1 4 // can_id for Outer Feet board -- check similar can_id; starting with MCH
	ID_MCFO_RIGHT_ANKLE_ANGLE = 37,	 // 0 4 1 // ankle joint angle in radians (zero when foot is turned all the way up)
	ID_MCFO_RIGHT_LS = 38,	 // 0 4 1 // 1 = right foot is turned all the way up
	ID_MCFO_LEFT_LS = 39,	 // 0 4 1 // 1 = left foot is turned all the way up
	ID_MCFO_RIGHT_HS = 40,	 // 0 4 1 // 1 = right foot is touching the ground; 0= foot in air
	ID_MCFO_LEFT_HS = 41,	 // 0 4 1 // 1 = left foot is touching the ground; 0= foot in air
	ID_MCFO_SHUTDOWN = 42,	 // 0 1 4 // can_id for Outer Feet board -- check similar can_id; starting with MCH
	ID_MCFO_SLEEP = 43,	 // 0 1 4 // can_id for Outer Feet board -- check similar can_id; starting with MCH
	ID_MCFO_BATT_POWER = 44,	 // 0 4 1 // can_id for Outer Feet board -- check similar can_id; starting with MCH
	ID_MCFO_COMMAND_CURRENT = 45,	 // 0 1 4 // can_id for Outer Feet board -- check similar can_id; starting with MCH
	ID_MCFO_STIFFNESS = 46,	 // 0 1 4 // can_id for Outer Feet board -- check similar can_id; starting with MCH
	ID_MCFO_DAMPNESS = 47,	 // 0 1 4 // can_id for Outer Feet board -- check similar can_id; starting with MCH
	ID_MCFO_STATUS = 48,	 // 0 4 1 // can_id for Outer Feet board -- check similar can_id; starting with MCH
	ID_MCFO_LEFT_HEEL_SENSE = 49,	 // 0 4 1 // analog value from foot contact sensor (high when foot is touching the ground; low when foot is in air)
	ID_MCFO_RIGHT_HEEL_SENSE = 50,	 // 0 4 1 // analog value from foot contact sensor (high when foot is touching the ground; low when foot is in air) 
	ID_MCFO_EXECUTION_TIME = 51,	 // 0 4 1 // can_id for Outer Feet board -- check similar can_id; starting with MCH
	ID_MCFO_MAX_EXECUTION_TIME = 52,	 // 0 4 1 // can_id for Outer Feet board -- check similar can_id; starting with MCH
	ID_MCFO_HS_STATE = 53,	 // 0 1 4 // ??? HS means heel strike
	ID_MCFO_EMPTY_TX2 = 54,	 // 0 1 4 // 
	ID_MCFO_BATT_CURRENT = 55,	 // 0 4 1 // can_id for Outer Feet board -- check similar can_id; starting with MCH
	ID_MCFO_BATT_VOLTAGE = 56,	 // 0 4 1 // can_id for Outer Feet board -- check similar can_id; starting with MCH
	ID_MCFO_HBRIDGE_TEMP = 57,	 // 0 4 1 // can_id for Outer Feet board -- check similar can_id; starting with MCH
	ID_MCFO_OUTER_HS = 58,	 // 0 4 1 // ??? HS means heel strike
	ID_MCFO_RIGHT_ANKLE_RATE = 59,	 // 0 4 1 // slightly filtered derivative of the ID_MCFO_RIGHT_ANKLE_ANGLE
	ID_MCFO_EMPTY_RX3 = 60,	 // 0 4 1 // 
	ID_MCFI_MOTOR_VELOCITY = 61,	 // 0 5 1 // can_id for Inner Feet board -- check similar can_id; starting with MCH
	ID_MCFI_MOTOR_CURRENT = 62,	 // 0 5 1 // can_id for Inner Feet board -- check similar can_id; starting with MCH
	ID_MCFI_MOTOR_POSITION = 63,	 // 0 5 1 // can_id for Inner Feet board -- check similar can_id; starting with MCH
	ID_MCFI_MOTOR_TARGET_CURRENT = 64,	 // 0 1 5 // can_id for Inner Feet board -- check similar can_id; starting with MCH
	ID_MCFI_MID_ANKLE_ANGLE = 65,	 // 0 5 1 // can_id for Inner Feet board -- check similar can_id; starting with MCFO
	ID_MCFI_RIGHT_LS = 66,	 // 0 5 1 // can_id for Inner Feet board -- check similar can_id; starting with MCFO
	ID_MCFI_LEFT_LS = 67,	 // 0 5 1 // can_id for Inner Feet board -- check similar can_id; starting with MCFO
	ID_MCFI_RIGHT_HS = 68,	 // 0 5 1 // can_id for Inner Feet board -- check similar can_id; starting with MCFO
	ID_MCFI_LEFT_HS = 69,	 // 0 5 1 // can_id for Inner Feet board -- check similar can_id; starting with MCFO
	ID_MCFI_BATT_POWER = 70,	 // 0 5 1 // can_id for Inner Feet board -- check similar can_id; starting with MCH
	ID_MCFI_SHUTDOWN = 71,	 // 0 1 5 // can_id for Inner Feet board -- check similar can_id; starting with MCH
	ID_MCFI_SLEEP = 72,	 // 0 1 5 // can_id for Inner Feet board -- check similar can_id; starting with MCH
	ID_MCFI_COMMAND_CURRENT = 73,	 // 0 1 5 // can_id for Inner Feet board -- check similar can_id; starting with MCH
	ID_MCFI_STIFFNESS = 74,	 // 0 1 5 // can_id for Inner Feet board -- check similar can_id; starting with MCH
	ID_MCFI_DAMPNESS = 75,	 // 0 1 5 // can_id for Inner Feet board -- check similar can_id; starting with MCH
	ID_MCFI_STATUS = 76,	 // 0 5 1 // can_id for Inner Feet board -- check similar can_id; starting with MCH
	ID_MCFI_LEFT_HEEL_SENSE = 77,	 // 0 5 1 // can_id for Inner Feet board -- check similar can_id; starting with MCFO
	ID_MCFI_RIGHT_HEEL_SENSE = 78,	 // 0 5 1 // can_id for Inner Feet board -- check similar can_id; starting with MCFO
	ID_MCFI_EXECUTION_TIME = 79,	 // 0 5 1 // can_id for Inner Feet board -- check similar can_id; starting with MCH
	ID_MCFI_MAX_EXECUTION_TIME = 80,	 // 0 5 1 // can_id for Inner Feet board -- check similar can_id; starting with MCH
	ID_MCFI_HS_STATE = 81,	 // 0 1 5 // can_id for Inner Feet board -- check similar can_id; starting with MCFO
	ID_MCFI_EMPTY_TX2 = 82,	 // 0 1 5 // 
	ID_MCFI_BATT_CURRENT = 83,	 // 0 5 1 // can_id for Inner Feet board -- check similar can_id; starting with MCH
	ID_MCFI_BATT_VOLTAGE = 84,	 // 0 5 1 // can_id for Inner Feet board -- check similar can_id; starting with MCH
	ID_MCFI_HBRIDGE_TEMP = 85,	 // 0 5 1 // can_id for Inner Feet board -- check similar can_id; starting with MCH
	ID_MCFI_INNER_HS = 86,	 // 0 5 1 // can_id for Inner Feet board -- check similar can_id; starting with MCFO
	ID_MCFI_ANKLE_RATE = 87,	 // 0 5 1 // can_id for Inner Feet board -- check similar can_id; starting with MCFO
	ID_MCFI_EMPTY_RX3 = 88,	 // 0 5 1 // 
	ID_COL_STATUS = 89,	 // 0 9 1 // IDs for the currently non existent color sensing; camera board tried on Ranger in 2010
	ID_COL_FRONT_WHITE = 90,	 // 0 9 1 // see above
	ID_COL_FRONT_RED = 91,	 // 0 9 1 // see above
	ID_COL_FRONT_GREEN = 92,	 // 0 9 1 // see above
	ID_COL_FRONT_BLUE = 93,	 // 0 9 1 // see above
	ID_COL_EXECUTION_TIME = 94,	 // 0 9 1 // see above
	ID_COL_MAX_EXECUTION_TIME = 95,	 // 0 9 1 // see above
	ID_ERROR_COL = 96,	 // 0 9 1 // see above
	ID_COL_EMPTY2 = 97,	 // 0 9 1 // see above
	ID_COL_EMPTY3 = 98,	 // 0 9 1 // see above
	ID_MCSO_LEFT_ANKLE_ANGLE = 99,	 // 0 6 1;4 // IDs for the currently non existent outer steering motor controller board tried on ranger before 2010
	ID_MCSO_LEFT_ANKLE_RATE = 100,	 // 0 6 1 // see above
	ID_MCSO_EMPTY_TX1 = 101,	 // 0 1 6 // see above
	ID_MCSO_EMPTY_TX2 = 102,	 // 0 1 6 // see above
	ID_MCSO_STATUS = 103,	 // 0 6 1 // see above
	ID_MCSO_COLOR_BACK_WHITE = 104,	 // 0 6 1 // see above
	ID_MCSO_COLOR_BACK_RED = 105,	 // 0 6 1 // see above
	ID_MCSO_COLOR_BACK_GREEN = 106,	 // 0 6 1 // see above
	ID_MCSO_COLOR_BACK_BLUE = 107,	 // 0 6 1 // see above
	ID_MCSO_EXECUTION_TIME = 108,	 // 0 6 1 // see above
	ID_MCSO_MAX_EXECUTION_TIME = 109,	 // 0 6 1 // see above
	ID_MCSI_MOTOR_CURRENT = 110,	 // 0 7 1 // can_id for steering board -- check similar can_id; starting with MCH
	ID_MCSI_MOTOR_TARGET_CURRENT = 111,	 // 0 1 7 // can_id for steering board -- check similar can_id; starting with MCH
	ID_MCSI_STEER_ANGLE = 112,	 // 0 7 1 // can_id for steering board -- check similar can_id; starting with MCH
	ID_MCSI_BATT_POWER = 113,	 // 0 7 1 // can_id for steering board -- check similar can_id; starting with MCH
	ID_MCSI_SHUTDOWN = 114,	 // 0 1 7 // can_id for steering board -- check similar can_id; starting with MCH
	ID_MCSI_SLEEP = 115,	 // 0 1 7 // can_id for steering board -- check similar can_id; starting with MCH
	ID_MCSI_COMMAND_ANG = 116,	 // 0 1 7 // steering angle commanded to the steering board
	ID_MCSI_PROP_COEFF = 117,	 // 0 1 7 // (march 2013) position coefficient in the PI controller currently used for low-level steering (in the Steering board)
	ID_MCSI_INT_COEFF = 118,	 // 0 1 7 // (march 2013) integral coefficient in the PI controller currently used for low-level steering (in the Steering board)
	ID_MCSI_STATUS = 119,	 // 0 7 1 // can_id for steering board -- check similar can_id; starting with MCH
	ID_MCSI_EXECUTION_TIME = 120,	 // 0 7 1 // can_id for steering board -- check similar can_id; starting with MCH
	ID_MCSI_MAX_EXECUTION_TIME = 121,	 // 0 7 1 // can_id for steering board -- check similar can_id; starting with MCH
	ID_MCSI_EMPTY_TX1 = 122,	 // 0 1 7 // 
	ID_MCSI_EMPTY_TX2 = 123,	 // 0 1 7 // 
	ID_MCSI_BATT_CURRENT = 124,	 // 0 7 1 // can_id for steering board -- check similar can_id; starting with MCH
	ID_MCSI_BATT_VOLTAGE = 125,	 // 0 7 1 // can_id for steering board -- check similar can_id; starting with MCH
	ID_MCSI_HBRIDGE_TEMP = 126,	 // 0 7 1 // can_id for steering board -- check similar can_id; starting with MCH
	ID_MCSI_EMPTY_RX1 = 127,	 // 0 7 1 // 
	ID_MCSI_EMPTY_RX2 = 128,	 // 0 7 1 // 
	ID_MCSI_EMPTY_RX3 = 129,	 // 0 7 1 // 
	ID_UI_SET_LCD_QUAD_1 = 130,	 // 0 1 8 // data displayed in the 1st quadrant of the LCD display ????
	ID_UI_SET_LCD_QUAD_2 = 131,	 // 0 1 8 // 
	ID_UI_SET_LCD_QUAD_3 = 132,	 // 0 1 8 // 
	ID_UI_SET_LCD_QUAD_4 = 133,	 // 0 1 8 // 
	ID_UI_SET_LED_1 = 134,	 // 0 1 8 // ids for the LEDs in the UI board. 
	ID_UI_SET_LED_2 = 135,	 // 0 1 8 // 
	ID_UI_SET_LED_3 = 136,	 // 0 1 8 // 
	ID_UI_SET_LED_4 = 137,	 // 0 1 8 // 
	ID_UI_SET_LED_5 = 138,	 // 0 1 8 // 
	ID_UI_SET_LED_6 = 139,	 // 0 1 8 // 
	ID_UI_SET_BUZZER_FREQ = 140,	 // 0 1 8 // produces the sound! (used to play alma mater)
	ID_UI_SET_BUZZER_AMPL = 141,	 // 0 1 8 // produces the sound! (used to play alma mater)
	ID_UI_ROLL = 142,	 // 0 8 1 // (euler angle from imu) for-aft angle of the robot from gravity vector
	ID_UI_PITCH = 143,	 // 0 8 1 // (euler angle from imu)
	ID_UI_YAW = 144,	 // 0 8 1 // (euler angle from imu)
	ID_UI_ANG_RATE_X = 145,	 // 0 8 1 // angular velocity in rad/sec (for-aft) from imu  (see the axes marked on the imu)
	ID_UI_ANG_RATE_Y = 146,	 // 0 8 1 // imu angle rate (not exactly equal to thederivative of ID_UI_PITCH)
	ID_UI_ANG_RATE_Z = 147,	 // 0 8 1 // imu rangle rate
	ID_UI_BUTTONS = 148,	 // 0 8 1 // 
	ID_UI_RC_0 = 149,	 // 0 8 1 // most left i/o pin on the UI board (used for steering command from RC; right joystick)
	ID_UI_RC_1 = 150,	 // 0 8 1 // (used to receive signal from RC; left joystick)
	ID_UI_RC_2 = 151,	 // 0 8 1 // (used to receive signal from RC; signal coming from a knob on the right of RC transmitter; used to switch between the camera and the RC)
	ID_UI_RC_3 = 152,	 // 0 8 1 // most right i/o pin on the UI board; camera gives its output here. 
	ID_UI_EXECUTION_TIME = 153,	 // 0 8 1 // can_id for the UI board -- check similar can_id; starting with MCH
	ID_UI_MAX_EXECUTION_TIME = 154,	 // 0 8 1 // can_id for the UI board -- check similar can_id; starting with MCH
	ID_UI_AUDIO = 155,	 // 0 1 8 // 
	ID_UI_EMPTY_TX2 = 156,	 // 0 1 8 // 
	ID_UI_STATUS = 157,	 // 0 8 1 // 
	ID_UI_EMPTY_RX4 = 158,	 // 0 8 1 // 
	ID_UI_EMPTY_RX5 = 159,	 // 0 8 1 // 
	ID_CSR_CAN1_LOAD = 160,	 // 0 2 0 // 
	ID_CSR_CAN2_LOAD = 161,	 // 0 2 0 // 
	ID_CSR_CAN3_LOAD = 162,	 // 0 2 0 // 
	ID_CSR_CAN4_LOAD = 163,	 // 0 2 0 // 
	ID_CSR_MCU_POWER = 164,	 // 0 2 0 // 
	ID_CSR_EMPTY_TX1 = 165,	 // 0 1 2 // 
	ID_CSR_EMPTY_TX2 = 166,	 // 0 1 2 // 
	ID_CSR_MCU_VOLTAGE = 167,	 // 0 2 1 // 
	ID_CSR_MCU_CURRENT = 168,	 // 0 2 1 // 
	ID_CSR_EMPTY_RX4 = 169,	 // 0 2 1 // 
	ID_CSR_EMPTY_RX5 = 170,	 // 0 2 1 // 
	ID_MB_EXECUTION_TIME = 171,	 // 0 1 0 // Time required to finish the schedule
	ID_ERROR_LABVIEW = 172,	 // 0 1 0 // Buffered errors for transmission to LabView/PC
	ID_ERROR_LCD = 173,	 // 0 1 8 // Buffered errors for transmission to Ranger LCD
	ID_MB_SYSTEM_INIT_FSM_STATE = 174,	 // 0 1 0 // System_init_fsm state number
	ID_MB_HIP_FSM_STATE = 175,	 // 0 1 0 // hip fsm state number
	ID_MB_FOOT_INNER_FSM_STATE = 176,	 // 0 1 0 // inner foot fsm state number
	ID_MB_FOOT_OUTER_FSM_STATE = 177,	 // 0 1 0 // outer foot fsm state number
	ID_MB_STEERING_FSM_STATE = 178,	 // 0 1 0 // steering fsm state number
	ID_MB_UI_FSM_STATE = 179,	 // 0 1 0 // ui fsm state number
	ID_MB_TOP_FSM_STATE = 180,	 // 0 1 0 // top fsm state number
	ID_MB_LEG_FSM_STATE = 181,	 // 0 1 0 // leg fsm state
	ID_MB_ROCK_HIP_FSM_STATE = 182,	 // 0 1 0 // rock hip fsm state
	ID_MB_ROCK_FI_FSM_STATE = 183,	 // 0 1 0 // rock foot inner fsm state
	ID_MB_ROCK_FO_FSM_STATE = 184,	 // 0 1 0 // rock foot outer fsm state
	ID_MB_EMPTY_FSM3 = 185,	 // 0 1 0 // Unused FSM ID
	ID_E_INNER_ANG_RATE = 186,	 // 0 1 0 // inner leg angular rate from estimator (world frame) rad/s; positive if falling forward
	ID_E_OUTER_ANG_RATE  = 187,	 // 0 1 0 // outer leg angle from estimator (world frame); rad; measured from vertical; positive if leaning forward
	ID_E_INNER_ANGLE = 188,	 // 0 1 0 // not being used (it's free)
	ID_E_OUTER_ANGLE = 189,	 // 0 1 0 // not being used (it's free)
	ID_E_T_AFTER_HS = 190,	 // 0 1 0 // time in ms since the last Heel strike
	ID_E_LI_ABSANG = 191,	 // 0 1 0 // inner leg angle from estimator (world frame); rad; measured from vertical; positive if leaning forward
	ID_E_LO_ABSANG = 192,	 // 0 1 0 // outer leg angle from estimator (world frame); rad; measured from vertical; positive if leaning forward
	ID_E_H_RATE = 193,	 // 0 1 0 // estimated hip angular rate  (filtered version of ID_MCH_ANG_RATE)
	ID_E_FI_ABSANG = 194,	 // 0 1 0 // estimated inner foot absolute angle; rad; not measured from vertical; positive if turned forward
	ID_E_FO_ABSANG = 195,	 // 0 1 0 // estimated outer foot absolute angle; rad; not measured from vertical; positive if turned forward
	ID_E_SWING_F_HEIGHT = 196,	 // 0 1 0 // estimated swing foot height from level ground
	ID_E_LEG_STATE = 197,	 // 0 1 0 // estimated swing leg state
	ID_E_MIDSTANCE_LEGRATE = 198,	 // 0 1 0 // velocity of stance leg when robot is upright
	ID_E_MIDSTANCE_HIPRATE = 199,	 // 0 1 0 // velocity of hip when robot is upright (positive if robot is walking fwd)
	ID_E_MIDSTANCE_HIPANG = 200,	 // 0 1 0 // angle of hip when robot is upright (positive if swing leg is in front of stance leg)
	ID_E_MIDSTANCE_TIME = 201,	 // 0 1 0 // time for midstance measured from heelstrike
	ID_E_STEP_LEGANG = 202,	 // 0 1 0 // stance leg angle at heelstrike
	ID_E_STEP_LEGRATE = 203,	 // 0 1 0 // stance leg rate at heelstrike
	ID_E_STEP_HIPANG = 204,	 // 0 1 0 // hip angle at heelstrike (positive for fwd step)
	ID_E_STEP_TIME = 205,	 // 0 1 0 // step time
	ID_E_STEP_NO = 206,	 // 0 1 0 // number of steps since the start (of entering the walk state in the ui_fsm)
	ID_E_TOTAL_DISTANCE = 207,	 // 0 1 0 // total distance walked since the start in meters
	ID_E_SS0_LEGANG = 208,	 // 0 1 0 // leg angle at beginning of swing phase
	ID_E_SS0_LEGRATE = 209,	 // 0 1 0 // leg rate at beginning of swing phase
	ID_E_H_MOTOR_RATE = 210,	 // 0 1 0 // hip motor rate; filtered version of ID_MCH_MOTOR_VELOCITY
	ID_E_EXEC_TIME = 211,	 // 0 1 0 // time taken by estimator to run
	ID_E_INT_LO_ABSANG_RESET = 212,	 // 0 1 0 // outer foot reset at hs
	ID_E_TIME_TO_HS = 213,	 // 0 1 0 // estimated time for next hs to happen (not tested/functioning)
	ID_E_SLOPE = 214,	 // 0 1 0 // estimated ground slope (not tested/ functioning)
	ID_E_SWING_LEG = 215,	 // 0 1 0 // 0 = outer leg is in swing; 1 = inner leg is in swing
	ID_E_F_PP_MISSED = 216,	 // 0 1 0 // 0 for outer and 1 for inner prepush missed
	ID_E_F_PP2HS_TIME = 217,	 // 0 1 0 // prepush to heelstrike time
	ID_E_F_PP_TIME = 218,	 // 0 1 0 // prepush time in ms
	ID_E_F_AP_TIME = 219,	 // 0 1 0 // afterpush time in ms
	ID_E_H_PS_TIME = 220,	 // 0 1 0 // pre-swing time in ms
	ID_E_H_AM_TIME = 221,	 // 0 1 0 // pre-swing time in ms
	ID_E_ANK2ANK_HT = 222,	 // 0 1 0 // height of swing ankle with respect to the stance ankle (meters)
	ID_E_DBL_STANCE = 223,	 // 0 1 0 // estimation of double/single stance state: 1 = in double stance; 0 = in single stance
	ID_E_TEST3 = 224,	 // 0 1 0 // dummy for testing
	ID_E_TEST4 = 225,	 // 0 1 0 // dummy for testing
	ID_E_TEST5 = 226,	 // 0 1 0 // dummy for testing
	ID_E_TEST6 = 227,	 // 0 1 0 // dummy for testing
	ID_E_TEST7 = 228,	 // 0 1 0 // dummy for testing
	ID_E_TEST8 = 229,	 // 0 1 0 // dummy for testing
	ID_E_TEST9 = 230,	 // 0 1 0 // dummy for testing
	ID_E_TEST10 = 231,	 // 0 1 0 // dummy for testing
	ID_E_TEST11 = 232,	 // 0 1 0 // dummy for testing
	ID_E_TEST12 = 233,	 // 0 1 0 // dummy for testing
	ID_FSM_RESET = 234,	 // 0 1 0 // resets the fsm so that the robot is easier to start walking
	ID_C_H_OUTER_STIFFNESS = 235,	 // 2 0 1 // stiffness to add in outer feet to make legs symmetric (was 1)
	ID_C_H_OUTER_DAMPNESS = 236,	 // 0 0 1 // dampness to add in outer feet to make legs symmetric 
	ID_A_H_PM_A0 = 237,	 // 0 0 1 // pre-mid constant current part
	ID_A_H_PM_A1 = 238,	 // 0 0 1 // pre-mid linear current part not used
	ID_C_H_PM_H_RATE = 239,	 // 3 0 1 // gain on velocity control
	ID_P_H_PM_H_TRATE = 240,	 // 2 0 1 // target hip velocity in premid
	ID_P_H_PM_TIME = 241,	 // 300 0 1 // not used
	ID_C_HI_SH_H_ANG = 242,	 // 30 0 1 // 
	ID_C_HI_SH_H_RATE = 243,	 // 3 0 1 // 
	ID_C_H_EH_H_ANG = 244,	 // 30 0 1 // 
	ID_C_H_EH_H_RATE = 245,	 // 4 0 1 // 
	ID_P_HI_SH_H_TANG = 246,	 // 0.35 0 1 // 
	ID_P_H_EH_H_TANG = 247,	 // 0.35 0 1 // 
	ID_P_H_EHOLD_ANG = 248,	 // 0.2 0 1 // 
	ID_P_H_EHOLD_ANG2 = 249,	 // 0.3 0 1 // 
	ID_P_H_EH_L_TRATE = 250,	 // 0.4 0 1 // target legrate below which ehold should kick in (was 0.5)
	ID_P_H_EH_L_TANG = 251,	 // 0.2 0 1 // target mid-stance angle for ehold
	ID_P_H_PM_F_ANG = 252,	 // 0 0 1 // feet absolute angle before pre-mid starts
	ID_P_H_PM_F_RATE = 253,	 // 5 0 1 // ankle swing rate before pre-mid starts
	ID_D_H_ON = 254,	 // 1 0 1 // after-mid discrete controller
	ID_DA_H_AM_A1 = 255,	 // 0 1 0 // value of current put in by discrete controller
	ID_DA_H_AM_A2 = 256,	 // 0 1 0 // value of current put in by discrete controller
	ID_DA2_H_AM_H_ANG1 = 257,	 // 0 1 0 // 
	ID_DA2_H_AM_H_ANG2 = 258,	 // 0 1 0 // 
	ID_DA2_H_AM_H_RATE1 = 259,	 // 0 1 0 // 
	ID_DA2_H_AM_H_RATE2 = 260,	 // 0 1 0 // 
	ID_DA3_H_AM_H_ANG1 = 261,	 // 0 1 0 // 
	ID_DA3_H_AM_H_ANG2 = 262,	 // 0 1 0 // 
	ID_DA3_H_AM_L_ABSRATE1 = 263,	 // 0 1 0 // 
	ID_DA3_H_AM_L_ABSRATE2 = 264,	 // 0 1 0 // 
	ID_DA3_H_AM_H_RATE1 = 265,	 // 0 1 0 // 
	ID_DA3_H_AM_H_RATE2 = 266,	 // 0 1 0 // 
	ID_DP_H_AM_H_ANG = 267,	 // 0 0 1 // 
	ID_DP_H_AM_L_ABSRATE = 268,	 // 0.72 0 1 // 
	ID_DP_H_AM_H_RATE = 269,	 // 1.66 0 1 // 
	ID_DP_H_AM_H_DRATE = 270,	 // 2.3 0 1 // 
	ID_DC_H_AM_H_ANG = 271,	 // 1 0 1 // 
	ID_DC_H_AM_L_ABSRATE = 272,	 // 0 0 1 // 
	ID_DC_H_AM_H_RATE = 273,	 // 1 0 1 // 
	ID_DC_H_AM_H_DRATE = 274,	 // 2 0 1 // 
	ID_DP_H_AM_TIME1 = 275,	 // 150 0 1 // time 1 for discrete control
	ID_DP_H_AM_TIME2 = 276,	 // 300 0 1 // time 2 for discrete control
	ID_DC2_H_AM_H_ANG1 = 277,	 // -1 0 1 // 
	ID_DC2_H_AM_H_ANG2 = 278,	 // -0.5 0 1 // 
	ID_DC2_H_AM_H_RATE1 = 279,	 // -0.3 0 1 // 
	ID_DC2_H_AM_H_RATE2 = 280,	 // -0.2 0 1 // 
	ID_DC3_H_AM_H_ANG1 = 281,	 // 2 0 1 // 
	ID_DC3_H_AM_H_ANG2 = 282,	 // 1 0 1 // 
	ID_DC3_H_AM_L_ABSRATE1 = 283,	 // 0 0 1 // 
	ID_DC3_H_AM_L_ABSRATE2 = 284,	 // 0 0 1 // 
	ID_DC3_H_AM_H_RATE1 = 285,	 // 0 0 1 // 
	ID_DC3_H_AM_H_RATE2 = 286,	 // 0 0 1 // 
	ID_D_H2_ON = 287,	 // 0 0 1 // ss0 discrete controller
	ID_DA_H_PM_A1 = 288,	 // 0 1 0 // value of current put in by discrete controller
	ID_DA_H_PM_A2 = 289,	 // 0 1 0 // value of current put in by discrete controller
	ID_DA_H_PM_L_ABSANG1 = 290,	 // 0 1 0 // 
	ID_DA_H_PM_L_ABSANG2 = 291,	 // 0 1 0 // 
	ID_DA_H_PM_L_ABSRATE1 = 292,	 // 0 1 0 // 
	ID_DA_H_PM_L_ABSRATE2 = 293,	 // 0 1 0 // 
	ID_DP_H_PM_L_ABSANG = 294,	 // 0 0 1 // 
	ID_DP_H_PM_L_ABSRATE = 295,	 // 0 0 1 // 
	ID_DC_H_PM_L_ABSANG1 = 296,	 // 0 0 1 // 
	ID_DC_H_PM_L_ABSANG2 = 297,	 // 0 0 1 // 
	ID_DC_H_PM_L_ABSRATE1 = 298,	 // 0 0 1 // 
	ID_DC_H_PM_L_ABSRATE2 = 299,	 // 0 0 1 // 
	ID_H_TEST1 = 300,	 // 0 0 1 // dummy for testing hip
	ID_H_TEST2 = 301,	 // 0 0 1 // dummy for testing hip
	ID_H_TEST3 = 302,	 // 0 0 1 // dummy for testing hip
	ID_H_TEST4 = 303,	 // 0 0 1 // dummy for testing hip
	ID_H_TEST5 = 304,	 // 0 0 1 // dummy for testing hip
	ID_P_F_PP_OUTER_BIAS = 305,	 // 1 0 1 // 
	ID_C_F_ST_F_ANG = 306,	 // 4 0 1 // 
	ID_C_F_ST_F_RATE = 307,	 // 0.2 0 1 // 
	ID_C_F_PP_F_ANG = 308,	 // 10 0 1 // 
	ID_C_F_PP_F_ANG2 = 309,	 // 8 0 1 // 
	ID_C_F_PP_F_RATE = 310,	 // 0 0 1 // spiky data. so set to zero
	ID_C_F_AP_F_ANG = 311,	 // 8 0 1 // 
	ID_C_F_AP_F_RATE = 312,	 // 0 0 1 // 
	ID_A_F_PP_A0 = 313,	 // 0 0 1 // current for prepush
	ID_C_F_FU_F_ANG = 314,	 // 1.5 0 1 // 
	ID_C_F_FU_F_RATE = 315,	 // 0.1 0 1 // 
	ID_C_F_FD_F_ANG = 316,	 // 1.5 0 1 // 
	ID_C_F_FD_F_RATE = 317,	 // 0.1 0 1 // 
	ID_C_FO_SST_F_ANG = 318,	 // 4 0 1 // 
	ID_C_FO_SST_F_RATE = 319,	 // 0.2 0 1 // 
	ID_C_F_SST_F_ANG = 320,	 // 30 0 1 // 
	ID_C_F_SST_F_RATE = 321,	 // 0.3 0 1 // 
	ID_P_F_PP_L_ABSANG = 322,	 // 0.12 0 1 // was 0.07 for barton record walk
	ID_P_F_PP_F_ABSANG = 323,	 // 1.7 0 1 // absolute feet angle for push-off
	ID_P_FO_PP_THEIGHT = 324,	 // 0.01 0 1 // push-off trigger for outer foot; changed from 0.012 by noopetr on Feb 19th 2013
	ID_P_FI_PP_THEIGHT = 325,	 // 0.01 0 1 // push-off trigger for inner foot; changed from by noopetr on Feb 19th 2013
	ID_P_F_AP_F_ANG = 326,	 // 0 0 1 // 
	ID_P_F_FU_H_ANG = 327,	 // 0.1 0 1 // was 0.13 for barton record walk
	ID_P_F_FU_F_TANG = 328,	 // 0.2 0 1 // 
	ID_P_F_ST_F_TANG = 329,	 // 1.85 0 1 // based on pranav's suggestions this number was changed from 2 to 1.85; anoop on March 01; 2013
	ID_P_F_PP_F_TANG = 330,	 // 0.2 0 1 // was 0.5 for barton record walk
	ID_P_F_PP_F_TANG2 = 331,	 // 0.3 0 1 // 
	ID_P_F_PP_FRAC_ANG = 332,	 // 0.5 0 1 // angle turned for prepush completion
	ID_P_F_AP_F_TANG = 333,	 // 0 0 1 // 
	ID_P_F_FD_F_TANG = 334,	 // 2 0 1 // 
	ID_P_F_SST_F_TANG = 335,	 // 2 0 1 // 
	ID_P_FO_SST_F_TANG = 336,	 // 2 0 1 // 
	ID_P_F_FU_F_CABL_STRETCH = 337,	 // 0.07 0 1 // cable stretch
	ID_P_F_AP_TIME = 338,	 // 30 0 1 // maximum time for after-push in ms
	ID_P_F_PP_F_LRATE = 339,	 // 0.5 0 1 // rate below which emergency prepush should occur
	ID_P_F_PP_F_HRATE = 340,	 // 0.8 0 1 // rate above which pre-push should be cut-off
	ID_P_F_PP_TIME = 341,	 // 10 0 1 // trigger for double-stance push-off for missed prepush
	ID_D_F_ON = 342,	 // 1 0 1 // foot discrete control on
	ID_DA_F_PP_L_ABSRATE = 343,	 // 0 1 0 // 
	ID_DA_F_PP_L_ABSRATE2 = 344,	 // 0 1 0 // 
	ID_DP_F_PP_L_ABSRATE = 345,	 // 0.65 0 1 // 
	ID_DC_F_PP_L_ABSRATE = 346,	 // 5 0 1 // gain on dampness in pp control
	ID_DC_F_PP_L_ABSRATE2 = 347,	 // 0.5 0 1 // gain on angle in pp control
	ID_F_TEST1 = 348,	 // 0 0 1 // dummy for testing feet
	ID_F_TEST2 = 349,	 // 0.5 0 1 // dummy for testing feet
	ID_F_TEST3 = 350,	 // 0 0 1 // dummy for testing feet
	ID_F_TEST4 = 351,	 // 0 0 1 // dummy for testing feet
	ID_F_TEST5 = 352,	 // 0 0 1 // dummy for testing feet 
	ID_F_TEST6 = 353,	 // 0 0 1 // dummy for testing feet -- (Zurich; March 2013) Andy's control parameter A  (~= -B ~= -3)
	ID_F_TEST7 = 354,	 // 0 0 1 // dummy for testing feet -- (Zurich; March 2013) Andy's control parameter B    (~= -A ~= 3)
	ID_F_TEST8 = 355,	 // 0 0 1 // dummy for testing feet -- (Zurich; March 2013) Andy's control parameter C      (~=1)
	ID_P_R_ROCK_TIMER = 356,	 // 0 1 0 // Total time in rock mode
	ID_P_R_FO_KP = 357,	 // 20 0 1 // Proportional gain for the outer foot controller
	ID_P_R_FI_KP = 358,	 // 20 0 1 // Proportional gain for the inner foot controller
	ID_P_R_FO_TANG = 359,	 // 2 0 1 // 
	ID_P_R_FI_TANG = 360,	 // 2 0 1 // 
	ID_P_R_FO_INIT_ANGLE = 361,	 // 2.2 0 1 // Angle for  initial stance
	ID_P_R_FI_INIT_ANGLE = 362,	 // 2.2 0 1 // Angle for initial stance
	ID_P_R_FO_END_ANGLE = 363,	 // 2 0 1 // Angle after initial stance
	ID_P_R_FI_END_ANGLE = 364,	 // 2 0 1 // Angle after initial stance
	ID_P_R_FO_PUSH_OFFSET = 365,	 // 0.5 0 1 // Angle below nominal for an outer foot push off
	ID_P_R_FI_PUSH_OFFSET = 366,	 // 0.85 0 1 // Angle below nominal for an inner foot push off
	ID_P_R_FI_CLEAR_OFFSET = 367,	 // 1.5 0 1 // Angle above nominal to clear the inner feet
	ID_P_R_H_THRESHOLD_ANG = 368,	 // 0.16 0 1 // Hip threshold angle to switch to walk mode
	ID_P_R_CONTACT_THRESHOLD = 369,	 // 1000 0 1 // Threshold value for an individual foot pressure sensor
	ID_P_R_WAIT_TIME = 370,	 // 5000 0 1 // Time to wait before robot starts rocking
	ID_P_R_FI_PUSH_TIME = 371,	 // 700 0 1 // Time inner foot is pushing before changing states
	ID_P_R_FO_PUSH_TIME = 372,	 // 800 0 1 // Time outer foot is pushing before changing states
	ID_P_R_ROCK_TO_WALK = 373,	 // 0 0 1 // Boolean value to switch from Rock to Walk
	ID_P_S_NULL_H_TANG = 374,	 // 0.1 0 1 // hip trigger angle
	ID_P_S_ILST_S_MAXANG = 375,	 // 2 0 1 // (March 2013): position term coefficient in the steering board motor controller; transmitted to the steering board through ID_MCSI_STIFFNESS
	ID_P_S_ILSW_S_MAXANG = 376,	 // 2000 0 1 // (March 2013): integral term coefficient in the steering board motor controller; transmitted to the steering board through ID_MCSI_DAMPNESS
	ID_C_S_NULL_S_ANG = 377,	 // 0 0 1 // gain of 0.002 works well
	ID_C_S_NULL_S_RATE = 378,	 // 0 0 1 // 
	ID_P_S_MAX_STEER_ANG = 379,	 // 0.1 0 1 // max steering angle (max steering angle command sent to the Steering board)
	ID_P_S_ILSW_STEER_FRAC = 380,	 // 0 0 0 // a fraction of desired steer angle by which inner legs should turn in inner swing phase; a number between 0 and 1
	ID_P_S_ILST_TWITCH_TIME = 381,	 // 1 0 0 // twice this time is the time for the twitch to take place in stance
	ID_D_S_NULL_S_DANG = 382,	 // 0 1 0 // desired steer angle; updated once per two steps
	ID_P_STEER_DEADBAND = 383,	 // 0.02 0 1 // dead band for steering; if the steering command (a value between -1 and 1) is smaller than this threshold; it is set to 0
	ID_P_UI_PP2HS_TTIME = 384,	 // 70 0 1 // set led if time is more that this time in ms
	ID_NAV_CAM_USED = 385,	 // 0 1 0 // says whether Camera Board has control authority: 0 = no (RC in use);  1 = yes
	ID_NAV_WALK = 386,	 // 0 1 0 // walk/stop command from RC/Camera: 1 = walk; 0 = stop
	ID_NAV_RC_STEER = 387,	 // 0 1 0 // normalized steering command from RC; a number in [-1;1]; where -1 = max right; 1 = max left
	ID_NAV_CAM_STEER = 388,	 // 0 1 0 // normalized steering command from Camera Board; a number in [-1;1]; where -1 = max right; 1 = max left
	ID_NAV_CAM_EMERG = 389,	 // 0 1 0 // emergency signal from the Camera Board; 1 = emergency is on; 0 = emergency is off
	ID_T_TEST_04 = 390,	 // 0 1 0 // 
	ID_T_TEST_05 = 391,	 // 0 1 0 // 
	ID_T_TEST_06 = 392,	 // 0 1 0 // 
	ID_T_TEST_07 = 393,	 // 0 1 0 // 
	ID_T_TEST_08 = 394,	 // 0 1 0 // 
	ID_T_TEST_09 = 395,	 // 0 1 0 // 
	ID_T_TEST_10 = 396,	 // 0 1 0 // 
	ID_T_TEST_11 = 397,	 // 0 1 0 // 
	ID_T_TEST_12 = 398,	 // 0 1 0 // 
	ID_T_TEST_13 = 399,	 // 0 1 0 // 
	ID_T_TEST_14 = 400,	 // 0 1 0 // 
	ID_T_TEST_15 = 401,	 // 0 1 0 // 
	ID_T_TEST_16 = 402,	 // 0 1 0 // 
	ID_T_TEST_17 = 403,	 // 0 1 0 // 
	ID_T_TEST_18 = 404,	 // 0 1 0 // 
	ID_T_TEST_19 = 405,	 // 0 1 0 // 
	ID_ABSOLUTE_COMP_TIME = 406,	 // 0 0 1 // 
	ID_GBRL_DELTA = 407,	 // 0.02 0 1 // Percent change for parameter perturbation
	ID_GBRL_MAX_STEPS = 408,	 // 8 0 1 // Number of steps per average reward computation
	ID_GBRL_REWARD = 409,	 // 0 1 0 // Current value of the GBRL reward function (usually Cost of Transport)
	ID_GBRL_POWER = 410,	 // 0 1 0 // Current estimated power consumption
	ID_GBRL_VELOCITY = 411,	 // 0 1 0 // Current estimated velocity of Ranger
	ID_GBRL_COT = 412,	 // 0 1 0 // Current estimated Cost of Transport
	ID_GBRL_CURRENT_PARAM = 413,	 // 0 1 0 // Current parameter being perturbed
	ID_GBRL_STEP_TIME = 414,	 // 0 1 0 // Time of the previous step
	ID_GBRL_STEP_LENGTH = 415,	 // 0 1 0 // Length of the previous step
	ID_GBRL_STEP_VELOCITY = 416,	 // 0 1 0 // Avg. Velocity of the previous step
	ID_GBRL_STABILITY = 417,	 // 0 1 0 // Estimated stability of Ranger
	ID_LV_START,	 //    // 
	ID_LV_CH_0,	 //    // 
	ID_LV_CH_1,	 //    // 
	ID_LV_CH_2,	 //    // 
	ID_LV_CH_3,	 //    // 
	ID_LV_CH_4,	 //    // 
	ID_LV_CH_5,	 //    // 
	ID_LV_CH_6,	 //    // 
	ID_LV_CH_7,	 //    // 
	ID_LV_CH_8,	 //    // 
	ID_LV_CH_9,	 //    // 
	ID_LV_CH_10,	 //    // 
	ID_LV_CH_11,	 //    // 
	ID_LV_CH_12,	 //    // 
	ID_LV_CH_13,	 //    // 
	ID_LV_CH_14,	 //    // 
	ID_LV_CH_15,	 //    // 
	ID_LV_CH_16,	 //    // 
	ID_LV_CH_17,	 //    // 
	ID_LV_CH_18,	 //    // 
	ID_LV_CH_19,	 //    // 
	ID_LV_CH_20,	 //    // 
	ID_LV_CH_21,	 //    // 
	ID_LV_CH_22,	 //    // 
	ID_LV_CH_23,	 //    // 
	ID_LV_CH_24,	 //    // 
	ID_LV_CH_25,	 //    // 
	ID_LV_CH_26,	 //    // 
	ID_LV_CH_27,	 //    // 
	ID_LV_CH_28,	 //    // 
	ID_LV_CH_29,	 //    // 
	ID_LV_CH_30,	 //    // 
	ID_LV_CH_31,	 //    // 
	ID_LV_CH_32,	 //    // 
	ID_LV_CH_33,	 //    // 
	ID_LV_CH_34,	 //    // 
	ID_LV_CH_35,	 //    // 
	ID_LV_CH_36,	 //    // 
	ID_LV_CH_37,	 //    // 
	ID_LV_CH_38,	 //    // 
	ID_LV_CH_39,	 //    // 
	ID_LV_CH_40,	 //    // 
	ID_LV_CH_41,	 //    // 
	ID_LV_CH_42,	 //    // 
	ID_LV_CH_43,	 //    // 
	ID_LV_CH_44,	 //    // 
	ID_LV_CH_45,	 //    // 
	ID_LV_CH_46,	 //    // 
	ID_LV_CH_47,	 //    // 
	ID_LV_VERSION,	 //    // 
	ID_LAST
} CAN_ID;


#endif
