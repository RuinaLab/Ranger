#include <mb_includes.h>
#include <input_output.h>
#include <mb_estimator.h>

static const int BUFFER_SIZE = 600; // How many data points can be buffered at a time.
static const int CALLS_PER_SEND = 10; // Somehow, even calling once per FSM cycle seems too much. Only send data every xth number of times sendBufferedData is called.

static CAN_ID IDs[BUFFER_SIZE]; // Keep the pointers of the labview IDs to send to.
static float data[BUFFER_SIZE];  // The values to send to labview on the ID of the same index.
static int activeNum = 0; // Keeps track of how many points are left to send.



/* Give the pointer to a CAN ID and a value to send and this will buffer it
 * so only one point is sent per main brain FSM cycle. Helps prevent lost 
 * data if you want to dump a lot in one loop. */
void addToQueue( CAN_ID outputID, float value ){

	if (activeNum < BUFFER_SIZE){ // TODO: This fails silently if too many points are added. Fix this.

		IDs[activeNum] = outputID;
		data[activeNum] = value;
		activeNum++;
	}
}

/* Send the first available buffered value to LabView. Send order == add order */
void sendBufferedData(void){
	static int calls = 0;

	int j;
	calls++;

	if ( calls > CALLS_PER_SEND-1 ){
		if ( activeNum > 0 ){

			// Send the first value to the specified channel.
			mb_io_set_float(IDs[0], data[0]);


			// Move every point back an index in data
			for (j = 0; j < activeNum - 1; j++) {   
   				data[j] = data[j+1];
   				IDs[j] = IDs[j+1];
			}

			activeNum--;
		}
		calls = 0;
	}
}
