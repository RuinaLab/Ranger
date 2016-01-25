#ifndef __BUFFER_DATA_OUT_H__
#define __BUFFER_DATA_OUT_H__

void addToQueue( CAN_ID outputID, float value );
void sendBufferedData(void);

#endif 
