#include <includes.h>

// NOTE: Errors disabled - error_send in sched

extern int global_running; 
// **********************  Variables  ************************** 
unsigned int song_on = 0;
unsigned int sync_led_on = 0;
SONG_NOTE alma_mater[26] = {
  {NOTE_DOTTED, NOTE_DOTTED, NOTE_Ab5},
  {NOTE_EIGHTH, NOTE_EIGHTH, NOTE_Bb5},
  {NOTE_DOTTED, NOTE_DOTTED, NOTE_C5},
  {NOTE_EIGHTH, NOTE_EIGHTH, NOTE_Bb5},
  {NOTE_QUARTER, NOTE_QUARTER, NOTE_Ab5},
  {NOTE_QUARTER, NOTE_QUARTER, NOTE_F4},
  {NOTE_QUARTER, NOTE_QUARTER, NOTE_F4},
  {NOTE_QUARTER, NOTE_QUARTER, NOTE_Eb4},
  {NOTE_DOTTED, NOTE_DOTTED, NOTE_Bb5},
  {NOTE_EIGHTH, NOTE_EIGHTH, NOTE_Ab5},
  {NOTE_QUARTER, NOTE_QUARTER, NOTE_G4},
  {NOTE_QUARTER, NOTE_QUARTER, NOTE_Ab5},
  {NOTE_WHOLE, 3*NOTE_QUARTER, NOTE_Bb5},
  
  {NOTE_DOTTED, NOTE_DOTTED, NOTE_Ab5},
  {NOTE_EIGHTH, NOTE_EIGHTH, NOTE_Bb5},
  {NOTE_DOTTED, NOTE_DOTTED, NOTE_C5},
  {NOTE_EIGHTH, NOTE_EIGHTH, NOTE_Bb5},
  {NOTE_QUARTER, NOTE_QUARTER, NOTE_Ab5},
  {NOTE_QUARTER, NOTE_QUARTER, NOTE_F4},
  {NOTE_QUARTER, NOTE_QUARTER, NOTE_F4},
  {NOTE_QUARTER, NOTE_QUARTER, NOTE_Eb4},
  {NOTE_DOTTED, NOTE_DOTTED, NOTE_Bb5},
  {NOTE_EIGHTH, NOTE_EIGHTH, NOTE_C5},
  {NOTE_QUARTER, NOTE_QUARTER, NOTE_Db4},
  {NOTE_QUARTER, NOTE_QUARTER, NOTE_G5},
  {NOTE_WHOLE, 3*NOTE_QUARTER, NOTE_Ab5}
};

SONG_NOTE buzzer_mario[74] = {
	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_E5},//intro
	{NOTE_QUARTER,NOTE_EIGHTH,NOTE_E5},
	{NOTE_QUARTER,NOTE_EIGHTH,NOTE_E5},
	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_C5},
	{NOTE_QUARTER,NOTE_QUARTER,NOTE_E5},
	{NOTE_HALF,NOTE_QUARTER,NOTE_G5},
	{NOTE_HALF,NOTE_QUARTER,NOTE_G4},//end intro
	
	{NOTE_DOTTED,NOTE_DOTTED,NOTE_C5},
	{NOTE_DOTTED,NOTE_EIGHTH,NOTE_G4},
	{NOTE_DOTTED,NOTE_DOTTED,NOTE_E4},
	{NOTE_QUARTER,NOTE_QUARTER,NOTE_A5},
	{NOTE_QUARTER,NOTE_QUARTER,NOTE_B5},
	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_Bb5},
	{NOTE_QUARTER,NOTE_QUARTER,NOTE_A5},
	{NOTE_SIXTH,NOTE_SIXTH,NOTE_G4},
	{NOTE_SIXTH,NOTE_SIXTH,NOTE_E5},
	{NOTE_SIXTH,NOTE_SIXTH,NOTE_G5},
	{NOTE_QUARTER,NOTE_QUARTER,NOTE_A5},
	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_F5},
	{NOTE_QUARTER,NOTE_EIGHTH,NOTE_G5},
	{NOTE_QUARTER,NOTE_QUARTER,NOTE_E5},
	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_C5},
	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_D5},
	{NOTE_DOTTED,NOTE_DOTTED,NOTE_B5},//repeat
	
	{NOTE_DOTTED,NOTE_DOTTED,NOTE_C5},
	{NOTE_DOTTED,NOTE_EIGHTH,NOTE_G4},
	{NOTE_DOTTED,NOTE_DOTTED,NOTE_E4},
	{NOTE_QUARTER,NOTE_QUARTER,NOTE_A5},
	{NOTE_QUARTER,NOTE_QUARTER,NOTE_B5},
	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_Bb5},
	{NOTE_QUARTER,NOTE_QUARTER,NOTE_A5},
	{NOTE_SIXTH,NOTE_SIXTH,NOTE_G4},
	{NOTE_SIXTH,NOTE_SIXTH,NOTE_E5},
	{NOTE_SIXTH,NOTE_SIXTH,NOTE_G5},
	{NOTE_QUARTER,NOTE_QUARTER,NOTE_A5},
	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_F5},
	{NOTE_QUARTER,NOTE_EIGHTH,NOTE_G5},
	{NOTE_QUARTER,NOTE_QUARTER,NOTE_E5},
	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_C5},
	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_D5},
	{5*NOTE_EIGHTH,NOTE_DOTTED,NOTE_B5},//repeat
	
	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_G5},
	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_Fs5},
	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_F5},
	{NOTE_QUARTER,NOTE_QUARTER,NOTE_Ds5},
	{NOTE_QUARTER,NOTE_EIGHTH,NOTE_E5},
	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_Gs4},
	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_A5},
	{NOTE_QUARTER,NOTE_EIGHTH,NOTE_C5},
	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_A5},
	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_C5},
	{NOTE_DOTTED,NOTE_EIGHTH,NOTE_D5},
	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_G5},
	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_Fs5},
	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_F5},
	{NOTE_QUARTER,NOTE_QUARTER,NOTE_Ds5},
	{NOTE_QUARTER,NOTE_EIGHTH,NOTE_E5},
	{NOTE_QUARTER,NOTE_QUARTER,NOTE_C6},
	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_C6},
	{3*NOTE_QUARTER,NOTE_HALF,NOTE_C6},
	
	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_G5},
	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_Fs5},
	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_F5},
	{NOTE_QUARTER,NOTE_QUARTER,NOTE_Ds5},
	{NOTE_QUARTER,NOTE_EIGHTH,NOTE_E5},
	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_Gs4},
	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_A5},
	{NOTE_QUARTER,NOTE_EIGHTH,NOTE_C5},
	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_A5},
	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_C5},
	{NOTE_DOTTED,NOTE_EIGHTH,NOTE_D5},
	{NOTE_DOTTED,NOTE_QUARTER,NOTE_Eb5},
	{NOTE_DOTTED,NOTE_DOTTED,NOTE_D5},
	{NOTE_HALF,NOTE_HALF,NOTE_C5}
	};

// *******************************************
// Scheduler Variables
// *******************************************
VOID_VOID_F schedule[]={
  //task_every_row, can_tx_pitch, dn_sched_done,NULL,
  //task_every_row, can_tx_yaw, dn_sched_done,NULL,
  task_every_row, can_tx_roll, dn_sched_done,NULL,
  task_every_row, can_tx_ang_rate_x, dn_sched_done, NULL,
  //task_every_row, can_tx_ang_rate_y, dn_sched_done,NULL,
  //task_every_row, can_tx_ang_rate_z, dn_sched_done,NULL,
  task_every_row, run_occasionally, dn_sched_done, NULL,
  NULL
};

/********************* TASKS ***********************/
void task_every_row(void) 
{
  static int song_count = 0;
  static int ui_count = 0;
  can_rx_dispatch_all();  
  hb_beat();          
  mcu_led_update(); 
  msimu_update();
  error_update();
  can_tx2(); // transmit all the can ids

  if (button_pushed(4))	{
    ui_count++;
    if (ui_count == 100){
      sync_led_on = !sync_led_on;
      ui_count = 0;
    }
  }
  
	if(sync_led_on) {ui_sync_led_flash();}
  else {ui_sync_led_off();}
  //update the song if it's playing
  song_update();   
}

//A simple function to run other functions at a slow, not necessarily well-timed rate.
//Add additional else if statements below for each new function that needs to run.
//Be sure that the index i values are continuous from 0 to (number of functions - 1),
//because it will skip any functions after a gap.
void run_occasionally(void)
{
  static short i = 0;
  if (i == 0){error_send_next();}           //Send error from error buffer over the CAN bus
  else if (i == 1){can_tx_buttons();}       //transmit button status
  else if (i == 2){can_tx_rc0();}           //transmit rc info
  else if (i == 3){can_tx_rc1();}           //Send motor controller input power over the CAN bus
  else if (i == 4){can_tx_status();}        //Send board operation status code to main brain
  else if (i == 5){can_tx_exec_time();}     //Send schedule execution time over the CAN bus
  else if (i == 6){can_tx_max_exec_time();} //Send maximum schedule execution time over the CAN bus 
  else if (i == 7){can_tx_pitch();}     // for the time being otherwise they were above
  else if (i == 8){can_tx_roll();}
  else if (i == 9){can_tx_yaw();} //
  else if (i == 10){can_tx_ang_rate_y();}     //
  else if (i == 11){can_tx_ang_rate_z();} //
  else if (i == 12){can_tx_rc2();}
  else if (i == 13){can_tx_rc3();}
  
  else{i = -1;}
  ++i; 
}

// **********************  Software Initialization  **************************
void init_software(void){

  // *********************************************
  // Standard board initialization.  Do not modify. 
  // *********************************************
  asched_init(schedule,1);
  error_init(dn_error_transmit, asched_get_timestamp, BOARD_UI);	//* dn_error_transmit
  hb_init(9, dn_blue_heartbeat, asched_get_timestamp); //heartbeat

  // *********************************************
  // Put user initialization code below here.  
  // *********************************************
  ui_led_init();  
  ui_sync_led_init(asched_get_timestamp); //sync led
  lcd_init(asched_tick);
  msimu_init(MSIMU_EULER_ANGS_ANG_RATE);
  rcx_init(1,1,1,1);
  
  song_set(alma_mater, 26);

}

void init_values(void)
{ //startup calibrations and the like

}





