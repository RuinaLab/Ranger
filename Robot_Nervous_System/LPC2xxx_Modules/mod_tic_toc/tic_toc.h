/*

  Utility module useful in debugging to see how long
  different functions take to execute.
  
  Nicolas Williamson - June 2009
  
*/


/* SOFTWARE HEADER SETUP - Put this into software_setup.h
 
 #define TT_CLOCK T1TC //uses timer 1 (or 0) to measure clock cycles
 
*/

// Function Declarations
void tic(void);
void toc(void);
unsigned int tt_get_time_elapsed(void);

