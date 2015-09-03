#include <includes.h>
unsigned char buf[] = "Hello World!!!\r\n";
int         buf_len = 16;
int busy_flag = 0;

void uart_int_test_callback(void){
  busy_flag = 0;
}

void task_uart_test(void){
  U0THR = 'A';
}

void task_uart_int_test(void){
  if(busy_flag == 0) {
    busy_flag = 1;
    uarti_tx_buf(buf,buf_len);
  } else{
    //hopefully we won't reach here
  }
  
}
