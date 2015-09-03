/*
	microstrain_imu.h
	
	Nicolas Williamson - Dec. 2009
*/

#ifndef __H_MSIMU__
#define __H_MSIMU__

typedef enum msimu_commands {
  MSIMU_RAW_ACCEL_ANG_RATE = 0xC1, //Raw Accelerometer and Angular Rate Sensor Outputs
  MSIMU_ACCEL_ANG_RATE = 0xC2, //Acceleration & Angular Rate
  MSIMU_DELTA_ANGLE_DELTA_VEL = 0xC3, //DeltaAngle & DeltaVelocity
  MSIMU_SET_CONTINUOUS_MODE = 0xC4, //Set Continuous Mode
  MSIMU_ORIENT_MATRIX = 0xC5, //Orientation Matrix
  MSIMU_ATTITUDE_UPDATE_MATRIX = 0xC6, //Attitude Update Matrix
  MSIMU_MAGNET_VECTOR = 0xC7, //Magnetometer Vector
  MSIMU_ACCEL_ANG_RATE_ORIENT_MATRIX = 0xC8, //Acceleration, Angular Rate & Orientation Matrix
  MSIMU_WRITE_ACCEL_BIAS_CORRECTION = 0xC9, //Write Accelerometer Bias Correction
  MSIMU_WRITE_GYRO_BIAS_CORRECTION = 0xCA, //Write Gyro Bias Correction
  MSIMU_ACCEL_ANG_RATE_MAGNET_VEC = 0xCB, //Acceleration, Angular Rate & Magnetometer Vector
  MSIMU_ACCEL_ANG_RATE_MAG_VEC_ORIENT_MATRIX = 0xCC, //Accel, Ang Rate & Mag Vectors & Orientation Matrix... jeeeeez the whole shebang!
  MSIMU_CAPTURE_GYRO_BIAS = 0xCD, //Capture Gyro Bias
  MSIMU_EULER_ANGLES = 0xCE, //Euler Angles
  MSIMU_EULER_ANGS_ANG_RATE = 0xCF, //Euler Angles and Angular Rates
  MSIMU_TRANSFER_QUANT_NON_VOLATILE_MEM = 0xD0, //Transfer Quantity to Non-Volatile Memory
  MSIMU_TEMPERATURES = 0xD1, //Temperatures
  MSIMU_GYRO_STAB_ACCEL_ANG_RATE_MAG_VEC = 0xD2, //Gyro Stabilized Acceleration, Angular Rate & Magnetometer Vector
  MSIMU_DELTA_ANG_DELTA_VEL_MAG_VEC = 0xD3, //DeltaAngle & DeltaVelocity & Magnetometer Vectors
  MSIMU_WRITE_WORD_EEPROM = 0xE4, //Write Word to EEPROM
  MSIMU_READ_WORD_EEPROM = 0xE5, //Read Word from EEPROM
  MSIMU_READ_FIRMWARE_VERSION_NUM = 0xE9, //Read Firmware Version Number
  MSIMU_STOP_CONTINUOUS_MODE = 0xFA, //Stop Continuous Mode (no reply)
  MSIMU_BUILT_IN_TEST = 0xFB //Built-in-Test
} MSIMU_COMMAND;

/******** Public Functions ********/
void msimu_update(void);
void msimu_init(MSIMU_COMMAND command);
float msimu_get_data_float(int index);
int msimu_get_data_int(int index);
void msimu_update(void);

/******** Private Functions ********/
void msimu_send_all(unsigned char *bytes, unsigned int length);
void msimu_send_byte(unsigned char byte);
void msimu_send_polled(MSIMU_COMMAND command);
void msimu_send_continuous(MSIMU_COMMAND command);
void msimu_isr(void) __irq;
int msimu_get_length(MSIMU_COMMAND command);
void msimu_parse_buffer(void);


#endif /* __H_MSIMU__ */

