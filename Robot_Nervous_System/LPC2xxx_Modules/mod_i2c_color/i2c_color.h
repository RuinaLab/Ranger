#ifndef __MOD_I2C_COLOR_H__
#define __MOD_I2C_COLOR_H__

//Gain Adjustment Guessing
//Capacitor 00H to 0FH. A higher capacitance value will result in lower sensor output.
#define CAP_ALL_VAL 0x15
#define CAP_RED_VAL CAP_ALL_VAL
#define CAP_BLUE_VAL CAP_ALL_VAL
#define CAP_GREEN_VAL CAP_ALL_VAL
#define CAP_CLEAR_VAL CAP_ALL_VAL
//Integration Time. 0 to 4095 (0xfff). A higher value in integration time will generally result in higher sensor 
//digital value if the capacitance gain registers have the same value.
#define INT_ALL_VAL 3095
#define INT_RED_VAL INT_ALL_VAL
#define INT_BLUE_VAL INT_ALL_VAL
#define INT_GREEN_VAL INT_ALL_VAL
#define INT_CLEAR_VAL INT_ALL_VAL
//Max/Min Accecptable Value for Gain Optimization
#define MAX_OK_VALUE 950
#define MIN_OK_VALUE 50
#define MIN_INT_TIME 2
#define MAX_INT_TIME 2047
//Operation Timeout Length in Number of Calls limited to max value of unsigned int
#define MAX_CALLS 1000

//Averager Length (2^x)
#define AVERAGER_LENGTH 2
#define SHIFT 4


/******************************************/
// ADJD-S371-QR999 I2C Addressing
/******************************************/
// Address of Color Sensor is static / 7-bits witout RW bit in LSB is 0x74h
//RW bit = 0 means master will write data to slave
#define SLAVE_WRITE_ADDR 0xE8
//RW bit = 1 means master will read data from slave
#define SLAVE_READ_ADDR 0xE9

/******************************************/
// Registers addresses on ADJD-S371-QR999 Color Sensor
/******************************************/
#define CTRL 0x0
#define CONFIG 0x1 
#define CAP_RED 0x06
#define CAP_GREEN 0x07
#define CAP_BLUE 0x08
#define CAP_CLEAR 0x09
#define INT_RED_LO 0x0A
#define INT_RED_HI 0x0B
#define INT_GREEN_LO 0x0C
#define INT_GREEN_HI 0x0D
#define INT_BLUE_LO 0x0E
#define INT_BLUE_HI 0x0F
#define INT_CLEAR_LO 0x10
#define INT_CLEAR_HI 0x11
#define DATA_RED_LO 0x40
#define DATA_RED_HI 0x41
#define DATA_GREEN_LO 0x42
#define DATA_GREEN_HI 0x43
#define DATA_BLUE_LO 0x44
#define DATA_BLUE_HI 0x45
#define DATA_CLEAR_LO 0x46
#define DATA_CLEAR_HI 0x47
#define OFFSET_RED 0x48
#define OFFSET_GREEN 0x49
#define OFFSET_BLUE 0x4A
#define OFFSET_CLEAR 0x4B

/******************************************/
// Possible I2C status codes
/******************************************/
#define I2C_START_TRANSMITTED 0x08
#define I2C_REPEATED_START_TRANSMITTED 0x10
#define I2C_SLA_AND_W_ACKED 0x18
#define I2C_SLA_AND_W_NOT_ACKED 0x20
#define I2C_DATA_ACKED 0x28
#define I2C_DATA_NOT_ACKED 0x30
#define I2C_RX_SLA_AND_W_ACKED 0x40
#define I2C_RX_SLA_AND_W_NOT_ACKED 0x48
#define I2C_RX_DATA_REC_ACK_RET 0x50
#define I2C_RX_DATA_REC_NOT_ACK_RET 0x58

//Critical Functions
void i2c_color_update(void);
void i2c_color_init(void);

//High Level Functions
unsigned char find_sensor(void);
unsigned char gain_optimization(unsigned short int);
unsigned char request_receive_colors(void);

//Data Manipulation Functions
unsigned long int get_absolute_reading(unsigned long int);
void avg_colors(void);

//External Data Capture Functions
float i2c_get_white_data(void);
float i2c_get_red_data(void);
float i2c_get_green_data(void);
float i2c_get_blue_data(void);

//i2c operation functions
unsigned char i2c_send_data(unsigned char, unsigned char);
unsigned char i2c_check_output(unsigned char);
void i2c_send_START(void);
void i2c_send_STOP(void);
void i2c_clear_SI(void);
void i2c_clear_STA(void);
void i2c_hardware_reset(void);

#endif  //__MOD_I2C_COLOR__
