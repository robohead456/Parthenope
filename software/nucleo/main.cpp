#include "mbed.h"
#include "EthernetInterface.h"
#include "osc_client.h"

#define SERIAL2TX PD_5
#define SERIAL2RX PD_8

#define VERBOSE 1

//Setup digital outputs
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);

/**
 * Function that processes the OSC message recieved in main
 */
static void osc_dispatch(OSCMessage* msg) {
	
	//Check if message is addressed to Parthenope
	char* token = strtok(msg->address, "/");
	if(strcmp(token, INSTRUMENT_NAME) != 0) {
		if(VERBOSE) printf("Message intended for '%s' instead of 'parthenope'\r\n", token);
		return;
	}

	led2 = 1;

	wait(0.05); //replace this with controlling the solenoids

	led2 = 0;

	return;
	
}

/**
 * Main function for Parthenope
 */
int main() {

	printf("Starting Parthenope\r\n");

	
	led1 = 1;
	led2 = 2;
	led3 = 3;

	//Setup serial communication for ODrives
	//Serial odrive1(SERIAL2TX, SERIAL2RX);
	//odrive1.baud(115200);

	//Setup ethernet interface
	EthernetInterface eth;
	eth.connect();
	if(VERBOSE) printf("Parthenope IP Address:     %s\r\n", eth.get_ip_address());
	led3 = 0;

	//Setup OSC client
	OSCClient osc(&eth);
	osc.connect();
	if(VERBOSE) printf("OSC controller IP Address: %s\r\n", osc.get_controller_ip());
	led2 = 0;
	
	//Create a pointer to an OSC Message
	OSCMessage* msg = (OSCMessage*) malloc(sizeof(OSCMessage));

	//Variable to store the OSC message size or error number
	nsapi_size_or_error_t size_or_error;
	

    while(true) {
        
		size_or_error = osc.receive(msg);

		//check if the recieve function returned an error
		if(size_or_error == NSAPI_ERROR_WOULD_BLOCK) {
			//if(VERBOSE) printf("NSAPI_ERROR_WOULD_BLOCK\n");
		}

		//check if the size of the msg is <=0
		else if(size_or_error <- 0) {
			if(VERBOSE) printf("ERROR: %d\r\n", size_or_error);
		}
		
		//process the message
		else {
			osc_dispatch(msg);
		}
    }
}

