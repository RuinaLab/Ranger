/**

	@file qec.h
	@author Nicolas Williamson and Thomas Craig 
  @date June 2009

*/

#ifndef __QEC_H__
#define __QEC_H__

//the resolution of the encoder can be 2 or 4
// - QEC_2X: counts interrupts from only one channel; uses state of both channels to infer direciton
// - QEC_4X: counts interrupts from both channels
/** The available resolutions for an encoder. */
typedef enum qec_resolution {
	QEC_NULL = 0, /**< Don't use this encoder. */
	QEC_2X = 2, /**< Use interrupts from only channel A; use state of both channels to infer direction. */
	QEC_4X = 4 /**< Use interrupts and state from both channel A and channel B. */
} QEC_RESOLUTION;

/** Possible IDs for the encoders. */
typedef enum qec_nums {
	QEC_1 = 0, /**< The encoder on J3. */
	QEC_2 /**< The encoder on J9. */
} QEC_ID;

//the data associated with an encoder
/** The data necessary for an encoder. */
typedef struct qec_data{
	volatile int current_pos; /**< the absolute position of the encoder. */
  int prev_pos;
  float velocity;
  volatile int prev_chA;
  volatile int prev_chB;
	QEC_RESOLUTION resolution; /**< The resolution of the encoder. */
} QEC_DATA;

#define QEC_2_PI (6.283185307179586476925286766559f)

// Functions
//Public
int qec_get_abs_pos(QEC_ID id); //returns the absolute position of the encoder
void qec_update_velocity(void);
float qec_get_velocity(QEC_ID id); //calculates and returns the velocity of an encoder
void qec_init(QEC_RESOLUTION enc_res_1, QEC_RESOLUTION enc_res_2, VOID_VOID_F function); //initializes the qec module with the given data structs
int qec_is_stopped(QEC_ID id);
//Private
void qec_isr(void); //__irq; //interrupt called when a new edge or timer overflow is detected
void qec_init_data(volatile QEC_DATA* data, QEC_RESOLUTION resolution);//initializes a qec data struct
//updates the given encoder with given flag and time information
void qec_update(volatile QEC_DATA* data, int flag_chA, int flag_chB, int chA, int chB);

#endif // __QEC_H__

