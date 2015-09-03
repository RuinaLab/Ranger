#ifndef  __UART_INT_H__
#define  __UART_INT_H__

typedef void(*UARTI_CALLBACK_PTR)(void);

void uarti_tx_set_empty_callback(UARTI_CALLBACK_PTR callback);
int uarti_tx_buf(char * buf, int buflen);
void uarti_tx_refill(void);
__irq void uarti_isr(void);
void uarti_print_int(int i);
void uarti_print_int2(int i1, int i2);
void uarti_print_float(float f);
void uarti_print_2float(float f1, float f2);

/* Put into hardware_setup.c

	// ***********************************************
	// UART Init Section
	// ***********************************************
	
	//Set P0.0 to TXD0 and P0.1 to RXD0 
	PINSEL0 &= ~(0xF);
	PINSEL0 |= 0x5;
	
	U0LCR = (1<<7);//DLAB = 1 to enable baud rate changes
	//Rate config from Jason for 115.2k, with 1+0.3 frac divider and pclk = 60MHz
	U0DLL = 25;
	U0DLM = 0;
	U0FDR = 3;//DIVADDVAL = 3
	U0FDR |= 10<<4;//MULTVAL = 10;
	//
	
	U0LCR = (0<<7)|(3<<0);//DLAB = 0 to disable baud rate changes, Wordsize = 3 for 8 bit words
	
	U0IER = (1<<1);//1: Enable THRE interrupt
	
	IO0DIR |= 1<<10; //Unassert SHDN_L on uart level shifter
	IO0SET = 1<<10;
	
	
	VICVectAddr8 = (unsigned long)uarti_isr;
	VICVectCntl8 = 0x20 | VIC_UART0; // Timer1 Interrupt 
	VICIntEnable = 1 << VIC_UART0;   // Enable Timer1 Interrupt 
*/

#endif /* __UART_INT_H__ */
