#include <includes.h>

//Global variables for detection of peak CAN frame rate
unsigned short dn_peak_frames_can1 = 0;
unsigned short dn_peak_frames_can2 = 0;
unsigned short dn_peak_frames_can3 = 0;
unsigned short dn_peak_frames_can4 = 0;

// CAN receive frame count global variables:
extern volatile unsigned short can_rx_frame_count_1;
extern volatile unsigned short can_rx_frame_count_2;
extern volatile unsigned short can_rx_frame_count_3;
extern volatile unsigned short can_rx_frame_count_4;

// CAN transmit frame count global variables:
extern volatile unsigned short can_tx_frame_count_1;
extern volatile unsigned short can_tx_frame_count_2;
extern volatile unsigned short can_tx_frame_count_3;
extern volatile unsigned short can_tx_frame_count_4;


///////////////////////////////////////////////////////////////////////////////////////////////////
// Define ring buffers for data flow in the CAN-SSP router board

//Set up ring name and buffer for CAN-SSP
//router board transmit (Error codes, battery voltage, system power, bus loading, etc.)
const unsigned short router_tx_frame_buf_len = 8;
CAN_RING router_tx_ring;
CAN_FRAME router_tx_frame_buf[router_tx_frame_buf_len];

///////////////////////////////////////////////////////////////////////////////////////////////////

void router_data_nexus_init(void)
{
  //Initialize ring buffer for router board transmit (error codes, voltage, power, CAN bus load, etc.)
  can_ring_init(&router_tx_ring, router_tx_frame_buf, router_tx_frame_buf_len);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Take incoming data from SSP receive buffer and CAN receive buffer.
//Call csr_route_frame to distribute it to the correct locations
void route_frames(void)
{
  CAN_FRAME frame;

  //pop one CAN frame pointer from ssp; route if available
  if (!csr_pop_ssp_frame(&frame))
  {
    csr_route_frame(&frame);
  }

  //pop one CAN frame from CAN receive buffer; route if available
  //if (!can_ring_pop(&can_rx_ring, &frame))
  if (!csr_can_rx_pop_frame(&frame))
  {
// **** TEST CODE ****  line below is for bench testing with interconnected CAN buses.
 //   frame.addr += 500;  // Put received CAN addresses outside the range of the routing table to prevent endless loops.
  
    csr_route_frame(&frame);
  }
    
  //pop one CAN frame from router board transmit buffer; route if available
  if (!can_ring_pop(&router_tx_ring, &frame))
  {
    csr_route_frame(&frame);
  }
}

// Build error frame, and push it onto the router board ring buffer for transmission to main brain
 void router_error_transmit(void)
 {
   CAN_FRAME frame;
   
   //Build CSR error frame
   frame.addr = ID_ERROR_CSR;
   frame.dlc = 8;
   frame.rtr = 0;
   frame.chan = CHAN_ROUTER;
   frame.payload.w.w1 = error_get_info();
   frame.payload.w.w2 = error_get_time();

  //Push frame onto router board (csr) ring buffer
   can_ring_push(&router_tx_ring, &frame);
 }

 // Build CAN bus load frame, and push it onto the router board ring buffer for transmission to main brain
 void router_can1_load_transmit(void)
 {
   CAN_FRAME frame;
   
   //Build CAN 1 bus load frame
   frame.addr = ID_CSR_CAN1_LOAD;
   frame.dlc = 8;
   frame.rtr = 0;
   frame.chan = CHAN_ROUTER;
   frame.payload.f.f1 = (float)dn_peak_frames_can1;
   frame.payload.w.w2 = csr_elapsed_ms();

   //reset peak frames variable
   dn_peak_frames_can1 = 0;

  //Push frame onto router board (csr) ring buffer
   can_ring_push(&router_tx_ring, &frame);
 }

  // Build CAN bus load frame, and push it onto the router board ring buffer for transmission to main brain
 void router_can2_load_transmit(void)
 {
   CAN_FRAME frame;
   
   //Build CAN 2 bus load frame
   frame.addr = ID_CSR_CAN2_LOAD;
   frame.dlc = 8;
   frame.rtr = 0;
   frame.chan = CHAN_ROUTER;
   frame.payload.f.f1 = (float)dn_peak_frames_can2;
   frame.payload.w.w2 = csr_elapsed_ms();

   //reset peak frames variable
   dn_peak_frames_can2 = 0;

  //Push frame onto router board (csr) ring buffer
   can_ring_push(&router_tx_ring, &frame);
 }

   // Build CAN bus load frame, and push it onto the router board ring buffer for transmission to main brain
 void router_can3_load_transmit(void)
 {
   CAN_FRAME frame;
   
   //Build CAN 3 bus load frame
   frame.addr = ID_CSR_CAN3_LOAD;
   frame.dlc = 8;
   frame.rtr = 0;
   frame.chan = CHAN_ROUTER;
   frame.payload.f.f1 = (float)dn_peak_frames_can3;
   frame.payload.w.w2 = csr_elapsed_ms();

   //reset peak frames variable
   dn_peak_frames_can3 = 0;

  //Push frame onto router board (csr) ring buffer
   can_ring_push(&router_tx_ring, &frame);
 }

   // Build CAN bus load frame, and push it onto the router board ring buffer for transmission to main brain
 void router_can4_load_transmit(void)
 {
   CAN_FRAME frame;
   
   //Build CAN 4 bus load frame
   frame.addr = ID_CSR_CAN4_LOAD;
   frame.dlc = 8;
   frame.rtr = 0;
   frame.chan = CHAN_ROUTER;
   frame.payload.f.f1 = (float)dn_peak_frames_can4;
   frame.payload.w.w2 = csr_elapsed_ms();

   //reset peak frames variable
   dn_peak_frames_can4 = 0;

  //Push frame onto router board (csr) ring buffer
   can_ring_push(&router_tx_ring, &frame);
 }

 // Build timestamp frame, and push it onto the router board ring buffer for transmission to main brain
 void router_timestamp_transmit(void)
 {
   CAN_FRAME frame;
   
   //Build timestamp frame
   frame.addr = ID_TIMESTAMP;
   frame.dlc = 8;
   frame.rtr = 0;
   frame.chan = CHAN_ROUTER;
   frame.payload.w.w2 = csr_elapsed_ms();
   frame.payload.f.f1 = (float)frame.payload.w.w2;

  //Push frame onto router board (csr) ring buffer
   can_ring_push(&router_tx_ring, &frame);
 }

void router_update_can_errors(void)
 {
  CAN_FRAME frame;

  can_get_error_1(&frame);
  if (frame.payload.w.w1)
  {
    b10a_blink_red_can_led(CHAN_CAN1, 50);
  }

  can_get_error_2(&frame);
  if (frame.payload.w.w1)
  {
    b10a_blink_red_can_led(CHAN_CAN2, 50);
  }

  can_get_error_3(&frame);
  if (frame.payload.w.w1)
  {
    b10a_blink_red_can_led(CHAN_CAN3, 50);
  }

  can_get_error_4(&frame);
  if (frame.payload.w.w1)
  {
    b10a_blink_red_can_led(CHAN_CAN4, 50);
  }

 }
 
// **********************************************************************************************************
// Router battery voltage, current, and power measurements. This is also the electronics system power,
// including all electronics and sensors except the motors and motor drivers.
//***********************************************************************************************************
//Battery voltage and current calibration constants
 
#define CSR_BATTERY_CURRENT_OFFSET   (-6)
#define CSR_BATTERY_CURRENT_GAIN     (0.000501)
 
#define CSR_BATTERY_VOLTAGE_OFFSET   (-5)
#define CSR_BATTERY_VOLTAGE_GAIN     (0.03046f)

//Measures battery current at the CSR, creates frame, sends to router.
void csr_send_battery_current(void)
{
  CAN_FRAME frame;
  float battery_current;
  long new_current; 
  
  new_current = ADDR0 & (0x3FF << 6);  //ADC reading is in 10 bits from 6 to 15 inclusive
  battery_current = (float)((new_current >> 6) - CSR_BATTERY_CURRENT_OFFSET) * CSR_BATTERY_CURRENT_GAIN;
  
  //Build frame
  frame.addr = ID_CSR_MCU_CURRENT;
  frame.dlc = 8;
  frame.rtr = 0;
  frame.chan = CHAN_ROUTER;
  frame.payload.f.f1 = battery_current;
  frame.payload.w.w2 = csr_elapsed_ms();


  //Push frame onto router board (csr) ring buffer
  can_ring_push(&router_tx_ring, &frame);
}

//Measures battery voltage at the CSR, creates frame, sends to router.
void csr_send_battery_voltage(void)
{
  CAN_FRAME frame;
  float battery_voltage;
  long new_voltage; 
  
  new_voltage = ADDR1 & (0x3FF << 6);  //ADC reading is in 10 bits from 6 to 15 inclusive
 // battery_voltage = (float)(new_voltage);
  battery_voltage = (float)((new_voltage >> 6) - CSR_BATTERY_VOLTAGE_OFFSET) * CSR_BATTERY_VOLTAGE_GAIN;
  
  //Build frame
  frame.addr = ID_CSR_MCU_VOLTAGE;
  frame.dlc = 8;
  frame.rtr = 0;
  frame.chan = CHAN_ROUTER;
  frame.payload.f.f1 = battery_voltage;
  frame.payload.w.w2 = csr_elapsed_ms();


  //Push frame onto router board (csr) ring buffer
  can_ring_push(&router_tx_ring, &frame);
}

//Measures battery power at the CSR, creates frame, sends to router.
void csr_send_battery_power(void)
{
  CAN_FRAME frame;
  float battery_power;
  long new_voltage;
  long new_current; 
  
  new_current = ADDR0 & (0x3FF << 6);  //ADC reading is in 10 bits from 6 to 15 inclusive
  new_voltage = ADDR1 & (0x3FF << 6);  //ADC reading is in 10 bits from 6 to 15 inclusive
  
  //Calculate battery power, using only one (I think) floating-point multiply so as to reduce execution time.
  battery_power = (float)(((new_voltage >> 6) - CSR_BATTERY_VOLTAGE_OFFSET) * ((new_current >> 6) - CSR_BATTERY_CURRENT_OFFSET)) 
    * (CSR_BATTERY_VOLTAGE_GAIN * CSR_BATTERY_CURRENT_GAIN);
  
  //Build frame
  frame.addr = ID_CSR_MCU_POWER;
  frame.dlc = 8;
  frame.rtr = 0;
  frame.chan = CHAN_ROUTER;
  frame.payload.f.f1 = battery_power;
  frame.payload.w.w2 = csr_elapsed_ms();


  //Push frame onto router board (csr) ring buffer
  can_ring_push(&router_tx_ring, &frame);
}
 
//***********************************************************************************************************
  #define DN_CAN_MAX 20   // At 3 megabits per second, full-load CAN traffic is about 30 frames/millisecond
                          // A red light will blink if loading is above 2/3 = 20 frames/mS. To be adjusted.

 void router_update_can_loading(void)
 {
  static unsigned short i = DN_CAN_MAX;
  unsigned short count;

  count = can_tx_frame_count_1 + can_rx_frame_count_1;
  if (count > dn_peak_frames_can1)
  {
    dn_peak_frames_can1 = count;    //Save maximum CAN load level
  }
  can_tx_frame_count_1 = 0;
  can_rx_frame_count_1 = 0;

  count = can_tx_frame_count_2 + can_rx_frame_count_2;
  if (count > dn_peak_frames_can2)
  {
    dn_peak_frames_can2 = count;    //Save maximum CAN load level
  }
  can_tx_frame_count_2 = 0;
  can_rx_frame_count_2 = 0;

  count = can_tx_frame_count_3 + can_rx_frame_count_3;
  if (count > dn_peak_frames_can3)
  {
    dn_peak_frames_can3 = count;    //Save maximum CAN load level
  }
  can_tx_frame_count_3 = 0;
  can_rx_frame_count_3 = 0;

  count = can_tx_frame_count_4 + can_rx_frame_count_4;
  if (count > dn_peak_frames_can4)
  {
    dn_peak_frames_can4 = count;    //Save maximum CAN load level
  }
  can_tx_frame_count_4 = 0;
  can_rx_frame_count_4 = 0;

  //Make green CAN LEDs blink for time proportional to CAN loading percentage of DN_CAN_MAX
  //Red LEDs blink proportionally to loading in excess of DN_CAN_MAX
  if (!i)
  {
    b10a_blink_green_can_led(CHAN_CAN1, dn_peak_frames_can1);
    b10a_blink_green_can_led(CHAN_CAN2, dn_peak_frames_can2);
    b10a_blink_green_can_led(CHAN_CAN3, dn_peak_frames_can3);
    b10a_blink_green_can_led(CHAN_CAN4, dn_peak_frames_can4);

    //Blink red LEDS for time proportional to excess load. May need finetuning... Could also put error_occurred calls here.
    if (dn_peak_frames_can1 > DN_CAN_MAX) {b10a_blink_red_can_led(CHAN_CAN1, dn_peak_frames_can1 - DN_CAN_MAX);}
    if (dn_peak_frames_can2 > DN_CAN_MAX) {b10a_blink_red_can_led(CHAN_CAN2, dn_peak_frames_can2 - DN_CAN_MAX);}
    if (dn_peak_frames_can2 > DN_CAN_MAX) {b10a_blink_red_can_led(CHAN_CAN3, dn_peak_frames_can3 - DN_CAN_MAX);}
    if (dn_peak_frames_can4 > DN_CAN_MAX) {b10a_blink_red_can_led(CHAN_CAN4, dn_peak_frames_can4 - DN_CAN_MAX);}

    // Restart interval counter
    i = DN_CAN_MAX;
  }
  --i;  //Decrement interval counter variable

 }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
