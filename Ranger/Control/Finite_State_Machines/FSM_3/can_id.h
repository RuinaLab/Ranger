//
//	
//  CAN_ID Table for Cornell Ranger Robot
//
//	Nicolas Williamson - October 2009
//
//

#ifndef __H_CAN_ID__
#define __H_CAN_ID__ 

typedef enum can_ids{

  // *** CAN IDs *** //

  // All Boards
  ID_TIMESTAMP = 0, // 0 10 dest_num 
  
  // Errors
  ID_ERROR_MCH  = 1, // 0 1 dest_num
  ID_ERROR_MCFO = 2, // 0 2 dest_num
  ID_ERROR_MCFI = 3, // 0 3 dest_num
  ID_ERROR_MCSO = 4, // 0 4 dest_num
  ID_ERROR_MCSI = 5, // 0 5 dest_num
  ID_ERROR_UI   = 6, // 0 6 dest_num
  ID_ERROR_CSR  = 7,  // 0 10  dest_num
  
  // Hip Motor Controller Board (MCH) Num = 1
  ID_MCH_MOTOR_VELOCITY = 10, // 0 1 dest_num
  ID_MCH_MOTOR_CURRENT = 11, // 0 1 dest_num
  ID_MCH_MOTOR_POSITION = 12, // 0 1 dest_num
  ID_MCH_MOTOR_TARGET_CURRENT = 13, // 0 0 dest_num
  ID_MCH_ANGLE = 14, // 0 1 dest_num
  ID_MCH_BATT_POWER = 15, // 0 1 dest_num
  ID_MCH_ANG_RATE = 16,   // 0 1 dest_num
  ID_MCH_TEST_MULTIPLIER = 17, // 0 1 dest_num
  
  // Outer Ankle MC Board (MCFO)
  ID_MCFO_MOTOR_VELOCITY = 20, // 0 2 dest_num
  ID_MCFO_MOTOR_CURRENT = 21, // 0 2 dest_num
  ID_MCFO_MOTOR_POSITION = 22, // 0 2 dest_num
  ID_MCFO_MOTOR_TARGET_CURRENT = 23, // 0 0 dest_num
  ID_MCFO_RIGHT_ANKLE_ANGLE = 24, // 0 2 dest_num
  ID_MCFO_RIGHT_LS = 25, // 0 2 dest_num
  ID_MCFO_LEFT_LS = 26, // 0 2 dest_num
  ID_MCFO_RIGHT_HS = 27, // 0 2 dest_num
  ID_MCFO_LEFT_HS = 28, // 0 2 dest_num
  ID_MCFO_BATT_POWER = 29, // 0 2 dest_num
  
  // Inner Ankle MC Board (MCFI)
  ID_MCFI_MOTOR_VELOCITY = 30, // 0 3 dest_num
  ID_MCFI_MOTOR_CURRENT = 31, // 0 3 dest_num
  ID_MCFI_MOTOR_POSITION = 32, // 0 3 dest_num
  ID_MCFI_MOTOR_TARGET_CURRENT = 33, // 0 0 dest_num
  ID_MCFI_MID_ANKLE_ANGLE = 34, // 0 3 dest_num
  ID_MCFI_RIGHT_LS = 35, // 0 3 dest_num
  ID_MCFI_LEFT_LS = 36, // 0 3 dest_num
  ID_MCFI_RIGHT_HS = 37, // 0 3 dest_num
  ID_MCFI_LEFT_HS = 38, // 0 3 dest_num
  ID_MCFI_BATT_POWER = 39, // 0 3 dest_num
  
  // Outer Steering MC Board (MCSO)
  ID_MCSO_MOTOR_POSITION = 40, // 0 4 dest_num
  ID_MCSO_MOTOR_CURRENT = 41, // 0 4 dest_num
  ID_MCSO_MOTOR_TARGET_CURRENT = 42, // 0 0 dest_num
  ID_MCSO_LEFT_ANKLE_ANGLE = 43, // 0 4 dest_num
  ID_MCSO_BATT_POWER = 44, // 0 4 dest_num
  ID_MCSO_STEER_ANGLE = 45, // 0 4 dest_num
  
  // User Interface Board (UI)
  ID_UI_SET_LCD_QUAD_1 = 50, // 0 0 dest_num
  ID_UI_SET_LCD_QUAD_2 = 51, // 0 0 dest_num
  ID_UI_SET_LCD_QUAD_3 = 52, // 0 0 dest_num
  ID_UI_SET_LCD_QUAD_4 = 53, // 0 0 dest_num
  ID_UI_SET_LED_1 = 54, // 0 0 dest_num
  ID_UI_SET_LED_2 = 55, // 0 0 dest_num
  ID_UI_SET_LED_3 = 56, // 0 0 dest_num
  ID_UI_SET_LED_4 = 57, // 0 0 dest_num
  ID_UI_SET_LED_5 = 58, // 0 0 dest_num
  ID_UI_SET_LED_6 = 59, // 0 0 dest_num
  ID_UI_SET_BUZZER_FREQ = 60, // 0 0 dest_num
  ID_UI_SET_BUZZER_AMPL = 61, // 0 0 dest_num
  ID_UI_RADIO_FREQ = 62, // 0 6 dest_num
  ID_UI_ROLL = 63, // 0 6 dest_num
  ID_UI_PITCH = 64, // 0 6 dest_num 
  ID_UI_YAW = 65, // 0 6 dest_num
  ID_UI_ANG_RATE_X = 66, // 0 6 dest_num
  ID_UI_ANG_RATE_Y = 67, // 0 6 dest_num
  ID_UI_ANG_RATE_Z = 68, // 0 6 dest_num
  ID_UI_BUTTONS = 69, // 0 6 dest_num
 
  // Inner Steering MC Board (MCSI)
  ID_MCSI_MOTOR_POSITION = 70, // 0 5 dest_num
  ID_MCSI_MOTOR_CURRENT = 71, // 0 5 dest_num
  ID_MCSI_MOTOR_TARGET_CURRENT = 72, // 0 1 dest_num
  ID_MCSI_STEER_ANGLE = 73, // 0 5 dest_num
  ID_MCSI_BATT_POWER = 74, // 0 5 dest_num
  
  // Motor Safety: Shutdown and sleep
  ID_MCH_SHUTDOWN = 80, // 0 0 dest_num
  ID_MCH_SLEEP = 81, // 0 0 dest_num
  ID_MCFO_SHUTDOWN = 82, // 0 0 dest_num
  ID_MCFO_SLEEP = 83, // 0 0 dest_num
  ID_MCFI_SHUTDOWN = 84, // 0 0 dest_num
  ID_MCFI_SLEEP = 85, // 0 0 dest_num
  ID_MCSO_SHUTDOWN = 86, // 0 0 dest_num
  ID_MCSO_SLEEP = 87, // 0 0 dest_num
  ID_MCSI_SHUTDOWN = 88, // 0 0 dest_num
  ID_MCSI_SLEEP = 89, // 0 0 dest_num
  
  // Startup Communications
  ID_MB_STATUS = 90, // 0 0 dest_num
  ID_MCH_STATUS = 91, // 0 1 dest_num
  ID_MCFO_STATUS = 92, // 0 2 dest_num
  ID_MCFI_STATUS = 93, // 0 3 dest_num
  ID_MCSO_STATUS = 94, // 0 4 dest_num
  ID_MCSI_STATUS = 95, // 0 5 dest_num
  ID_UI_STATUS = 96, // 0 6 dest_num
  
  //Compliant Commands
  ID_MCH_COMMAND_CURRENT = 100, // 0 1 dest_num
  ID_MCFO_COMMAND_CURRENT = 101, // 0 1 dest_num
  ID_MCFI_COMMAND_CURRENT = 102, // 0 1 dest_num
  ID_MCSO_COMMAND_CURRENT = 103, // 0 1 dest_num
  ID_MCSI_COMMAND_CURRENT = 104, // 0 1 dest_num
  ID_MCH_STIFFNESS = 105,// 0 1 dest_num
  ID_MCFO_STIFFNESS = 106,// 0 1 dest_num
  ID_MCFI_STIFFNESS = 107,// 0 1 dest_num
  ID_MCSO_STIFFNESS = 108,// 0 1 dest_num
  ID_MCSI_STIFFNESS = 109,// 0 1 dest_num
  ID_MCH_DAMPNESS = 110,// 0 1 dest_num
  ID_MCFO_DAMPNESS = 111,// 0 1 dest_num
  ID_MCFI_DAMPNESS = 112,// 0 1 dest_num
  ID_MCSO_DAMPNESS = 113,// 0 1 dest_num
  ID_MCSI_DAMPNESS = 114,// 0 1 dest_num
  
//  ID_MCFO_DUMMY_1 = 115,// 0 2 dest_num
//  ID_MCFO_DUMMY_2 = 116,// 0 2 dest_num
// ID_MCFO_DUMMY_3 = 117,// 0 2 dest_num
//  ID_MCFO_DUMMY_4 = 118,// 0 2 dest_num

  ID_CSR_CAN1_LOAD    = 115,   // 0 10 dest_num
  ID_CSR_CAN2_LOAD    = 116,   // 0 10 dest_num
  ID_CSR_CAN3_LOAD    = 117,   // 0 10 dest_num
  ID_CSR_CAN4_LOAD    = 118,   // 0 10 dest_num
  ID_CSR_MCU_POWER    = 119,   // 0 10 dest_num
  
  // Max possible CAN_ID (CID) value
  //ID_MAX_CAN = 2031,


  // *** Non-CAN Control Parameters and Data *** //
  ID_MB_EXECUTION_TIME        = 120,  //0 7 dest_num Time required to finish the schedule
  ID_ERROR_MB                 = 121,    //0 7 dest_num  Errors generated by main brain
  ID_ERROR_LABVIEW            = 122,  //0  7 dest_num Buffered errors for transmission to LabView/PC
  ID_ERROR_LCD                = 123,  //0  7 dest_num Buffered errors for transmission to Ranger LCD
  ID_Absolute_Comp_Time	      = 124,  //0 0 dest_num Absolute Computer Time for use with video sync
  
  ID_MB_SYSTEM_INIT_FSM_STATE = 130,  //0 7 dest_num  System_init_fsm state number
  ID_MB_HIP_FSM_STATE         = 131,  //0 7 dest_num  hip fsm state number
  ID_MB_FOOT_INNER_FSM_STATE  = 132,  //0 7 dest_num  hip fsm state number
  ID_MB_FOOT_OUTER_FSM_STATE  = 133,  //0 7 dest_num  hip fsm state number
  ID_MB_STEERING_FSM_STATE  = 134,  //0 7 dest_num  hip fsm state number



  ID_EST_INNER_ANG_RATE =   160,  //0 1 dest_num  inner leg angular rate from estimator (world frame)
  ID_EST_OUTER_ANG_RATE =   161,  //0 1 dest_num  outer leg angular rate from estimator (world frame)
  ID_EST_INNER_ANGLE =      162,  //0 1 dest_num  inner leg angle from estimator (world frame)
  ID_EST_OUTER_ANGLE =      163,  //0 1 dest_num  outer leg angle from estimator (world frame)
  ID_E_T_AFTER_HS = 164,          //0 1 dest_num
  ID_E_LI_ABSANG = 165,           //0 1 dest_num
  ID_E_LO_ABSANG = 166,           //0 1 dest_num
  ID_E_H_RATE             = 167,    //0 1 dest_num estimated hip angular rate
  ID_E_FI_ABSANG          = 168,    //0 1 dest_num estimated inner foot absolute angle
  ID_E_FO_ABSANG          = 169,    //0 1 dest_num estimated outer foot absolute angle
  ID_E_SWING_F_HEIGHT     = 170,    //0 1 dest_num estimated swing foot height from level ground
  
   // Hip state machines
  ID_A_H_PM_A0 = 200, //0 0 dest_num
  ID_A_H_PM_A1 = 201, //0 0 dest_num
  
  ID_C_HI_SH_H_ANG = 202, //0 0 dest_num
  ID_C_HI_SH_H_RATE = 203, //0 0 dest_num
  ID_C_H_EH_H_ANG = 204, //0 0 dest_num
  ID_C_H_EH_H_RATE = 205, //0 0 dest_num

  ID_P_HI_SH_H_TANG = 206, //0 0 dest_num
  ID_P_H_EH_H_TANG = 207, //0 0 dest_num
  ID_P_H_EHOLD_ANG = 208, //0 0 dest_num

  ID_A_H_TEST_A0 = 209, //0 0 dest_num

  //Steer state machine
  ID_P_S_NULL_S_TANG = 240, //0 0 dest_num
  ID_C_S_NULL_S_ANG = 241, //0 0 dest_num
  ID_C_S_NULL_S_RATE = 242, //0 0 dest_num
  ID_P_S_ILST_S_MAXANG = 243, //0 0 dest_num
  ID_P_S_ILSW_S_MAXANG = 244, //0 0 dest_num
  ID_D_S_NULL_S_DANG = 245, //0 1 dest_num
  ID_P_S_TEMP_S_TANG = 246, //0 0 dest_num temporary desired steer angle

  // Foot state machines
  ID_C_F_ST_F_ANG = 251, //0 0 dest_num
  ID_C_F_ST_F_RATE = 252, //0 0 dest_num
  ID_C_F_PP_F_ANG = 253, //0 0 dest_num
  ID_C_F_PP_F_RATE = 254, //0 0 dest_num
  ID_C_F_FU_F_ANG = 255, //4 0 dest_num
  ID_C_F_FU_F_RATE = 256, //0.2 0 dest_num
  ID_C_F_FD_F_ANG =  257, //4 0 dest_num
  ID_C_F_FD_F_RATE = 258, //0.2 0 dest_num
  ID_C_F_SST_F_ANG = 259, //0 0 dest_num
  ID_C_F_SST_F_RATE = 260, //0 0 dest_num

  ID_P_F_PP_L_ABSANG = 261, //0 0 dest_num
  ID_P_F_FU_H_ANG = 262, //0.1 0 dest_num
  ID_P_F_FU_F_TANG = 263, //0.2 0 dest_num
  ID_P_F_ST_F_TANG = 264, //0 0 dest_num
  ID_P_F_PP_F_TANG = 265, //0 0 dest_num
  ID_P_F_FD_F_TANG = 266, //1.8 0 dest_num
  ID_P_F_SST_F_TANG = 267, //0 0 dest_num
  ID_P_F_S_H_ANG = 268, //0.05 0 dest_num
  
  //UI Radio Control Values
  ID_UI_RC_0 = 300,   //0 6 dest_num
  ID_UI_RC_1 = 301,   //0 6 dest_num
  ID_UI_RC_2 = 302,   //0 6 dest_num
  ID_UI_RC_3 = 303,   //0 6 dest_num
   
  //LabView Parameter Channels
  ID_LV_START,  //0 0 1
  ID_LV_CH_0,
  ID_LV_CH_1,
  ID_LV_CH_2,
  ID_LV_CH_3,
  ID_LV_CH_4,
  ID_LV_CH_5,
  ID_LV_CH_6,
  ID_LV_CH_7,
  ID_LV_CH_8,
  ID_LV_CH_9,
  ID_LV_CH_10,
  ID_LV_CH_11,
  ID_LV_CH_12,
  ID_LV_CH_13,
  ID_LV_CH_14,
  ID_LV_CH_15,
  ID_LV_CH_16,
  ID_LV_CH_17,
  ID_LV_CH_18,
  ID_LV_CH_19,
  ID_LV_CH_20,
  ID_LV_CH_21,
  ID_LV_CH_22,
  ID_LV_CH_23,
  ID_LV_CH_24,
  ID_LV_CH_25,
  ID_LV_CH_26,
  ID_LV_CH_27,
  ID_LV_CH_28,
  ID_LV_CH_29,
  ID_LV_CH_30,
  ID_LV_CH_31,
  ID_LV_CH_32,
  ID_LV_CH_33,
  ID_LV_CH_34,
  ID_LV_CH_35,
  ID_LV_CH_36,
  ID_LV_CH_37,
  ID_LV_CH_38,
  ID_LV_CH_39,
  ID_LV_CH_40,
  ID_LV_CH_41,
  ID_LV_CH_42,
  ID_LV_CH_43,
  ID_LV_CH_44,
  ID_LV_CH_45,
  ID_LV_CH_46,
  ID_LV_CH_47,

  ID_LV_VERSION,
  
  // Max Defined ID Value (in use)
  ID_LAST

} CAN_ID;

      
#endif
