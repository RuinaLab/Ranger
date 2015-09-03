/**

@file i2c_color.c
Contains i2c communication procedure
Runs gain optimization
Returns r,g and b values from color sensor

Example Hardware and Register setup for the LEDs:
@code
// ***********************************************
// Heartbeat Init Section
// ***********************************************
//On-board LED initialization
PINSEL2 &= ~(1<<3);   // set trace port for GPIO use (bit 3 = 0)
FIO1DIR |= (1<<23);   // set P1.23 to be output (Green LED)
FIO1DIR |= (1<<24);   // set P1.24 to be output (Red LED)
FIO1DIR |= (1<<25);   // set P1.25 to be output (Blue LED)
MCU_LED_ALL_OFF; 
@endcode

@author Michael Digman
@date December 2010

*/

#include <includes.h>

//define global variables
unsigned long int i2c_clear_data;
unsigned long int i2c_red_data;
unsigned long int i2c_blue_data;
unsigned long int i2c_green_data; 
unsigned long int i2c_clear_data_avg;
unsigned long int i2c_red_data_avg;
unsigned long int i2c_blue_data_avg;
unsigned long int i2c_green_data_avg;
short int state_color_update;
short int step;
short int state_send_data;
short int state_request_receive_colors;
unsigned long timestampo = 0;
unsigned short int integration_time = INT_ALL_VAL;

void i2c_color_update(void){
  unsigned char debug = 0;

  switch(state_color_update){
    case 0: //1) check for color sensor
      /*debug = find_sensor();
      //if(debug == 0xFF){ //sensor is found
        //state_color_update = 1; //move to next state
        //step = 0;  //reset current step
      }*/
	  state_color_update = 1;
      break;
    case 1: //2) perform gain optimization
      debug = gain_optimization(integration_time);
      if(debug == 0xf0){ //gain complete
        state_color_update = 2;
        step = 0;
        state_request_receive_colors = 0;
      }
      break;
    case 2: //3) return currently visible colors
      debug = request_receive_colors();
      if( debug == 0xfe ){ //color request cycle complete, restart!
         state_request_receive_colors = 0;
         //1) are any of the visible colors over the maximum value or below minimum?
         if( i2c_clear_data >= MAX_OK_VALUE ||
             i2c_red_data >= MAX_OK_VALUE ||
             i2c_blue_data >= MAX_OK_VALUE ||
             i2c_green_data >= MAX_OK_VALUE ){ //readjust gain optimization

           if(integration_time >= MIN_INT_TIME){
            integration_time = integration_time>>1; //divide by 2
            state_color_update = 4;
           } else { //the integration time cannot be fixed, just collect data
            avg_colors();
           }
         } else if ( i2c_clear_data <= MIN_OK_VALUE ||
             i2c_red_data <= MIN_OK_VALUE ||
             i2c_blue_data <= MIN_OK_VALUE ||
             i2c_green_data <= MIN_OK_VALUE ) {

           if(integration_time <= MAX_INT_TIME){
            integration_time = integration_time<<1; //mult by 2
            state_color_update = 4;
           } else { //the integration time cannot be fixed, just collect data
            avg_colors();
           }
                 
         }else { //if the colors readings are great, average them!
          avg_colors(); 
         }
      }  
      break;
    case 3: //4) wait until timer says to resample
      if( T0TC < timestampo ){
        state_color_update = 2;
      }
      break;
    case 4: //5) CALL GAIN OPTIMIZATION
      debug = gain_optimization(integration_time);
      if(debug == 0xf0){ //gain complete
        state_color_update = 2;
        step = 0;
        state_request_receive_colors = 0;
      }
      break;
  }
}

void avg_colors(void){
    //remove dependance on integration time
    i2c_clear_data = get_absolute_reading(i2c_clear_data);
    i2c_red_data = get_absolute_reading(i2c_red_data);
    i2c_blue_data = get_absolute_reading(i2c_blue_data);
    i2c_green_data = get_absolute_reading(i2c_green_data);
            
    //average
    i2c_clear_data_avg = i2c_clear_data_avg - (i2c_clear_data_avg>>SHIFT) + (i2c_clear_data);    
    i2c_red_data_avg = i2c_red_data_avg - (i2c_red_data_avg>>SHIFT) + (i2c_red_data);
    i2c_blue_data_avg = i2c_blue_data_avg - (i2c_blue_data_avg>>SHIFT) + (i2c_blue_data);           
    i2c_green_data_avg = i2c_green_data_avg - (i2c_green_data_avg>>SHIFT) + (i2c_green_data);
    timestampo = T0TC;

    //debug overrides
    //i2c_clear_data_avg = integration_time<<SHIFT;

    //get more colors!
    state_color_update = 3;
}

unsigned long int get_absolute_reading(unsigned long int reading){
    return (reading<<12)/integration_time;
}

unsigned char i2c_check_output(unsigned char expected_status){
  unsigned char debug = (((unsigned char)I2CONSET)&0x8)>>3;
  
  if(debug == 1 && I2STAT == expected_status) { 
      debug = 0xff;
  } else { 
    debug = I2STAT; 
  }// we have a problem! 
  
  return debug;
}

void i2c_send_START(void){ I2CONSET = 0x20; } //set STA bit high to signify START signal
void i2c_send_STOP(void) { I2CONSET = 0x10; }
void i2c_clear_SI(void) { I2CONCLR = 0x08; } //set SIC high
void i2c_clear_STA(void) { I2CONCLR = 0x20; } //set STAC high
void i2c_hardware_reset(void) { 
  I2CONCLR = 0x6C; // sets AA to 0, SI to 0 STA to 0, I2EN to 0
  I2CONSET = 0x40; //sets i2onset to -10000--
}

unsigned char find_sensor(void){
	unsigned char debug = 0;
	switch (step) {
		case 0: 
			i2c_send_START(); //send a start message
			step = 1; //move to the next state
			break;
		case 1: //wait for start SI
      debug = i2c_check_output(I2C_START_TRANSMITTED);
      if(debug == 0xff){//if start command is successfully transmitted
  			//place slave address on line.
  			I2DAT = SLAVE_WRITE_ADDR;
  			//clear SI 
        i2c_clear_SI();
  			//move to the next state
  			step = 2;
       }
			break;
		case 2: //wait ACK to be returned on slave address transmission
			debug = i2c_check_output(I2C_SLA_AND_W_ACKED);
      if(debug == 0xff){
          i2c_send_STOP();
					debug = 0xFF; //flag that it worked
      }
			break;
	}
	return debug;
}

unsigned char i2c_get_data(unsigned char addr){
  static unsigned int counter = 0;
  unsigned char debug = 0;
  switch (state_send_data) {
  		case 0: //send a start message
			i2c_hardware_reset(); //reset
			i2c_send_START();
			state_send_data = state_send_data+1; //move to the next state
			counter = 0;
			break;
  		case 1: //wait for start SI
  			debug = i2c_check_output(I2C_START_TRANSMITTED);
			if(debug == 0xff){//if start command is successfully transmitted
				I2DAT = SLAVE_WRITE_ADDR;
				i2c_clear_SI();
				i2c_clear_STA();
				state_send_data = state_send_data+1;
				counter = 0;
			}else if(counter > MAX_CALLS){//interference problem, restart transmission from 0
				i2c_send_STOP();
				state_send_data = 0; 
			}else{counter=counter+1;}
  			break;
		case 2: //ensure that slave address has been acked
			debug = i2c_check_output(I2C_SLA_AND_W_ACKED);
			if(debug == 0xff){
				I2DAT = addr;
				i2c_clear_SI();
				state_send_data = state_send_data+1;
				counter = 0;
			}else if(counter > MAX_CALLS){//interference problem, restart transmission from 0
				i2c_send_STOP();
				state_send_data = 0; 
			}else{counter=counter+1;}
			break;
		case 3:
			debug = i2c_check_output(I2C_DATA_ACKED);
			if(debug == 0xff){ //address has been sent properly!
				//ENTER MASTER RX MODE
				//RESEND START
				i2c_send_START();
				i2c_clear_SI();
				state_send_data = state_send_data+1;
				counter = 0;
			}else if(counter > MAX_CALLS){//interference problem, restart transmission from 0
				i2c_send_STOP();
				state_send_data = 0; 
			}else{counter=counter+1;}
			break;
		case 4: 
			debug = i2c_check_output(I2C_REPEATED_START_TRANSMITTED);
			if(debug == 0xff){//if start command is successfully transmitted
				I2DAT = SLAVE_READ_ADDR;
				i2c_clear_STA();
				i2c_clear_SI();            
				state_send_data = state_send_data+1;
				counter = 0;
			}else if(counter > MAX_CALLS){//interference problem, restart transmission from 0
				i2c_send_STOP();
				state_send_data = 0; 
			}else{counter=counter+1;}
		case 5:
			debug = i2c_check_output(I2C_RX_SLA_AND_W_ACKED);
			if(debug == 0xff){
				I2DAT = addr;
				i2c_clear_SI();
				state_send_data = state_send_data+1;
				counter = 0;
			}else if(counter > MAX_CALLS){//interference problem, restart transmission from 0
				i2c_send_STOP();
				state_send_data = 0; 
			}else{counter=counter+1;}
			break;
		case 6:
			debug = i2c_check_output(I2C_RX_DATA_REC_NOT_ACK_RET);
			if(debug == 0xff){
				i2c_send_STOP();
				i2c_clear_SI();
				debug = 0xee; //inform caller to read from i2dat
				counter = 0;
			}else if(counter > MAX_CALLS){//interference problem, restart transmission from 0
				i2c_send_STOP();
				state_send_data = 0; 
			}else{counter=counter+1;}
			break;
    }
    return debug;
}

unsigned char i2c_send_data(unsigned char data, unsigned char addr){
  static unsigned int counter = 0;
  unsigned char debug = 0;
  switch (state_send_data) {
  		case 0: //send a start message
			i2c_hardware_reset(); //reset
			i2c_send_START();
			state_send_data = state_send_data+1; //move to the next state
			counter = 0;
  			break;
  		case 1: //wait for start SI
  			debug = i2c_check_output(I2C_START_TRANSMITTED);
			if(debug == 0xff){//if start command is successfully transmitted
				I2DAT = SLAVE_WRITE_ADDR;
				i2c_clear_SI();
				i2c_clear_STA();
				state_send_data = state_send_data+1;
				counter = 0;
  			}else if(counter > MAX_CALLS){//interference problem, restart transmission from 0
				i2c_send_STOP();
				state_send_data = 0; 
			}else{counter=counter+1;}
  			break;
		case 2: //ensure that slave address has been acked
			debug = i2c_check_output(I2C_SLA_AND_W_ACKED);
			if(debug == 0xff){
				I2DAT = addr;
				i2c_clear_SI();
				state_send_data = state_send_data+1;
				counter = 0;
			}else if(counter > MAX_CALLS){//interference problem, restart transmission from 0
				i2c_send_STOP();
				state_send_data = 0; 
			}else{counter=counter+1;}
			break;
		case 3:
			debug = i2c_check_output(I2C_DATA_ACKED);
			if(debug == 0xff){ //address has been sent properly!
				I2DAT = data;
				i2c_clear_SI();
				state_send_data = state_send_data+1;
				counter = 0;
			}else if(counter > MAX_CALLS){//interference problem, restart transmission from 0
				i2c_send_STOP();
				state_send_data = 0; 
			}else{counter=counter+1;}
			break;
		case 4:
			debug = i2c_check_output(I2C_DATA_ACKED);
			if(debug == 0xff){ //data has been sent properly!
				i2c_send_STOP();
				debug = 0xee;
				counter = 0;
			}else if(counter > MAX_CALLS){//interference problem, restart transmission from 0
				i2c_send_STOP();
				state_send_data = 0; 
			}else{counter=counter+1;}
			break;
    }
    return debug;
}

unsigned char gain_optimization(unsigned short int integration_time_value){
  //**** STEP 1 **** 
  //Write sensor gain registers, CAP_RED, CAP_GREEN, 
  //CAP_BLUE and CAP_CLEAR to select the number of 
  //capacitor. The values must range from 00H to 0FH. A 
  //higher capacitance value will result in lower sensor 
  //output
  unsigned char debug = 0;
  switch (step) {
    case 0:
      debug = i2c_send_data(CAP_RED_VAL, CAP_RED);
      if(debug==0xee){
        state_send_data = 0;
        step = step + 1;
      }
      break;
    case 1:
      debug = i2c_send_data(CAP_BLUE_VAL, CAP_BLUE);
      if(debug==0xee){
        state_send_data = 0;
        step = step + 1;
      }
      break;
    case 2:
      debug = i2c_send_data(CAP_GREEN_VAL, CAP_GREEN);
      if(debug==0xee){
        state_send_data = 0;
        step = step + 1;
      }
      break;
    case 3:
      debug = i2c_send_data(CAP_CLEAR_VAL, CAP_CLEAR);
      if(debug==0xee){
        state_send_data = 0;
        step = step + 1;
      }
      break;

  /* STEP 2: Write sensor gain registers, INT_RED, INT_GREEN, INT_
  BLUE and INT_CLEAR to select the integration time. 
  The integration time registers is a 12-bit registers, 
  the values is range from 0 to 4095. A higher value in 
  integration time will generally result in higher sensor 
  digital value if the capacitance gain registers have the 
  same value(*/

   case 4: 
      debug = i2c_send_data(integration_time_value&0xff, INT_RED_LO);
      if(debug==0xee){
        state_send_data = 0;
        step = step + 1;
      }
      break;
   case 5: 
      debug = i2c_send_data((integration_time_value&0xf00)>>8, INT_RED_HI);
      if(debug==0xee){
        state_send_data = 0;
        step = step + 1;
      }
      break;  
   case 6: 
      debug = i2c_send_data(integration_time_value&0xff, INT_BLUE_LO);
      if(debug==0xee){
        state_send_data = 0;
        step = step + 1;
      }
      break;
   case 7: 
      debug = i2c_send_data((integration_time_value&0xf00)>>8, INT_BLUE_HI);
      if(debug==0xee){
        state_send_data = 0;
        step = step + 1;
      }
      break;
   case 8: 
      debug = i2c_send_data(integration_time_value&0xff, INT_GREEN_LO);
      if(debug==0xee){
        state_send_data = 0;
        step = step + 1;
      }
      break;
   case 9: 
      debug = i2c_send_data((integration_time_value&0xf00)>>8, INT_GREEN_HI);
      if(debug==0xee){
        state_send_data = 0;
        step = step + 1;
      }
      break;
   case 10: 
      debug = i2c_send_data(integration_time_value&0xff, INT_CLEAR_LO);
      if(debug==0xee){
        state_send_data = 0;
        step = step + 1;
      }
      break;
   case 11: 
      debug = i2c_send_data((integration_time_value&0xf00)>>8, INT_CLEAR_HI);
      if(debug==0xee){
        state_send_data = 0;
        step = step + 1;
      }
      break;

  /* STEP 3: Acquire sensor digital values by writing 01H to CTRL 
  register (address 00H). Then read CTRL register. When 
  the value is 00H, the sensor digital values are read 
  from the sample data registers (address  40H to 47H). 
  If these sensor digital values are not optimum, do 
  another iteration loop consisting of step 2, 3 and 4*/

    case 12: 
      debug = request_receive_colors();
      if( debug == 0xfe) { //all colors received
        //initalize average
        i2c_red_data_avg = get_absolute_reading(i2c_red_data)<<SHIFT;
        //i2c_red_data_avg =0;
        i2c_blue_data_avg = get_absolute_reading(i2c_blue_data)<<SHIFT;
        //i2c_blue_data_avg =0;
        i2c_clear_data_avg = get_absolute_reading(i2c_clear_data)<<SHIFT;
        //i2c_clear_data_avg =0;
        i2c_green_data_avg = get_absolute_reading(i2c_green_data)<<SHIFT;
        //i2c_green_data_avg =0;
        state_send_data = 0;
        state_request_receive_colors = 0;

        //send end code
        debug = 0xf0;
      }
      break;
   }
   return debug;
}

unsigned char request_receive_colors(void) {
  unsigned char debug = 0;
  switch(state_request_receive_colors){
    case 0: //set control to 0x1 
        debug = i2c_send_data(0x01, CTRL);
        if(debug==0xee){
          state_send_data = 0;
          state_request_receive_colors = state_request_receive_colors +1;
        }
        break;
      case 1: //read from control, is it 00?
        debug = i2c_get_data(CTRL);
        if(debug == 0xee) { //is control reading 00?
          if(I2DAT == 0){
            //yes! read the colors
            state_send_data = 0;
            state_request_receive_colors = state_request_receive_colors +1;
          } else {
            state_send_data = 0; 
          }
        }
        break;
     case 2: //read from red low
        debug = i2c_get_data( DATA_RED_LO);
        if(debug == 0xee) {
          i2c_red_data = I2DAT;
          state_send_data = 0;
          state_request_receive_colors = state_request_receive_colors +1;
        }
        break;
     case 3: //read from red high
        debug = i2c_get_data( DATA_RED_HI);
        if(debug == 0xee) {
          i2c_red_data = ((I2DAT&0x3)<<8)|i2c_red_data; //merge colors
          state_send_data = 0;
          state_request_receive_colors = state_request_receive_colors +1;
        }
        break;
     case 4: //read from blue low
        debug = i2c_get_data( DATA_BLUE_LO);
        if(debug == 0xee) {
          i2c_blue_data = I2DAT;
          state_send_data = 0;
          state_request_receive_colors = state_request_receive_colors +1;
        }
        break;
     case 5: //read from blue high
        debug = i2c_get_data( DATA_BLUE_HI);
        if(debug == 0xee) {
          i2c_blue_data = ((I2DAT&0x3)<<8)|i2c_blue_data; //merge colors
          state_send_data = 0;
          state_request_receive_colors = state_request_receive_colors +1;
        }
        break;
     case 6: //read from green low
        debug = i2c_get_data( DATA_GREEN_LO);
        if(debug == 0xee) {
          i2c_green_data = I2DAT;
          state_send_data = 0;
          state_request_receive_colors = state_request_receive_colors +1;
        }
        break;
     case 7: //read from green high
        debug = i2c_get_data( DATA_GREEN_HI);
        if(debug == 0xee) {
          i2c_green_data = ((I2DAT&0x3)<<8)|i2c_green_data; //merge colors
          state_send_data = 0;
          state_request_receive_colors = state_request_receive_colors +1;
        }
        break;
      case 8: //read from clear low
        debug = i2c_get_data( DATA_CLEAR_LO);
        if(debug == 0xee) {
          i2c_clear_data = I2DAT;
          state_send_data = 0;
          state_request_receive_colors = state_request_receive_colors +1;
        }
        break;
     case 9: //read from clear high
        debug = i2c_get_data( DATA_CLEAR_HI);
        if(debug == 0xee) {
          i2c_clear_data = ((I2DAT&0x3)<<8)|i2c_clear_data; //merge colors
          state_send_data = 0;
          debug = 0xfe;
        }
        break;
     }
     return debug;
}

//on implementation of public functions 
//use abstraction in data_nexus i2c_color section
//as it is directly called by the can_setup function
float i2c_get_white_data(void){
	return (float) (i2c_clear_data_avg>>SHIFT);
}
float i2c_get_red_data(void){
	return (float) (i2c_red_data_avg>>SHIFT);
}
float i2c_get_blue_data(void){
	return (float) (i2c_blue_data_avg>>SHIFT);
  //return i2c_get_red_data()/i2c_get_green_data();
}
float i2c_get_green_data(void){
	return (float) (i2c_green_data_avg>>SHIFT);
}

void i2c_color_init(void){
	//set internal state to 0
	state_color_update = 0;
	step = 0;
  state_send_data = 0;
  state_request_receive_colors = 0;
  timestampo = 0;

	//PINSEL0 &=~(3<<4);
	//PINSEL0 &=~(3<<6);
	//PINSEL0 |=1<<4;
	//PINSEL0 |=1<<6;

	//set up appropriate data rate
	//I2SCLH defines the number of pclk cycles for SCL high, I2SCLL defines the number of pclk cycles for SCL low.
	//see p 175 for more details
	//I2SCLH = 300;
	//I2SCLL = 300;

	//turn i2onset
	//make i2conset look like MSB: - 1 0 0 0 0 - - :LSB
	//I2CONCLR = 0; // sets AA to 0, SI to 0 STA to 0, I2EN to 0
	//I2CONSET = 1<<6; //sets i2onset to -10000--
}

