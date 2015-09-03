/**
	@file button.c
	
  Simple button sensor readings for the UI board.
  
  Example hardware/pin setup:
  @code
  // *******************************************************************************
	// Initialize Buttons
	// *******************************************************************************
	PINSEL0 &= ~(3<<24); //Button 1: P0.12 as GPIO
	PINSEL0 &= ~(3<<26); //Button 2: P0.13 as GPIO
	PINSEL1 &= ~(3<<24); //Button 3: P0.28 as GPIO
	PINSEL1 &= ~(3<<28); //Button 5: P0.30 as GPIO
	PINSEL2 &= ~(1<<3);  //Buttons 0 & 4: P1.18 and P1.16 GPIO (P1.16-25 GPIO)
	//set pins as inputs
	FIO0DIR &= ~(1<<12); 
	FIO0DIR &= ~(1<<13);
	FIO0DIR &= ~(1<<28);
	FIO0DIR &= ~(1<<30);
	FIO1DIR &= ~(1<<16);
	FIO1DIR &= ~(1<<18);
	@endcode
  
	@author Nicolas Williamson 
  @date July 2009
  
*/

#include <includes.h>

/**
  Checks if a button is pushed.
  @param button The number of the button to check. Buttons are numbered 0 to 5.
  @return The state of the button; 1 = pressed, 0 = not pressed.
*/
unsigned int button_pushed(unsigned int button)
{
	unsigned int button_state = 0;
	unsigned int p0_state = FIO0PIN;
	unsigned int p1_state = FIO1PIN;
	
	if (button > 5){
		error_occurred(ERROR_BUTTON_OOB);
	} else {
		switch (button){
			case 0: button_state = p1_state & (1<<18);
					break;
			case 1: button_state = p0_state & (1<<12);
					break;
			case 2: button_state = p0_state & (1<<13);
					break;
			case 3: button_state = p0_state & (1<<28);
					break;
			case 4: button_state = p1_state & (1<<16);
					break;
			case 5: button_state = p0_state & (1<<30);
					break;
		}
	}
	return button_state;
}

unsigned int button_0(void){return button_pushed(0);} /**< Return the state of button 0. @return 1 = pressed, 0 = not pressed. */
unsigned int button_1(void){return button_pushed(1);} /**< Return the state of button 1. @return 1 = pressed, 0 = not pressed. */
unsigned int button_2(void){return button_pushed(2);} /**< Return the state of button 2. @return 1 = pressed, 0 = not pressed. */
unsigned int button_3(void){return button_pushed(3);} /**< Return the state of button 3. @return 1 = pressed, 0 = not pressed. */
unsigned int button_4(void){return button_pushed(4);} /**< Return the state of button 4. @return 1 = pressed, 0 = not pressed. */
unsigned int button_5(void){return button_pushed(5);} /**< Return the state of button 5. @return 1 = pressed, 0 = not pressed. */

