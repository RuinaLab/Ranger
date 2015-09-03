#ifndef __MICROSTRAIN_IMU_h__
#define __MICROSTRAIN_IMU_h__



void ms_imu_isr(void) __irq;
void ms_imu_init(void);
void ms_imu_parse_buffer(void);

void ms_imu_send_char_array(const unsigned char *array, unsigned short int length);







#endif	//__MICROSTRAIN_IMU_h__
