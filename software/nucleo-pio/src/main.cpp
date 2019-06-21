#include <mbed.h>
#include <map>
#include "NSS.h"
#include "EthernetInterface.h"
#include "osc_client.h"

#define VERBOSE 1

const char* instrumentName = "OSC-Test";

//Setup digital outputs
DigitalOut led1(LED1);	//Green running LED
DigitalOut led2(LED2);	//Blue controller LED
DigitalOut led3(LED3);	//Red ethernet LED

int main() {

	printf("Setting up NSS\r\n");
	wait(2);

	// Create a map to refer to the solenoid pins 
	//     by the MIDI note
	map<uint8_t,PinName> midiNotes;

	midiNotes[60]=SOLENOID_A01; midiNotes[61]=SOLENOID_A02;
	midiNotes[62]=SOLENOID_A03; midiNotes[63]=SOLENOID_A04;
	midiNotes[64]=SOLENOID_A05; midiNotes[65]=SOLENOID_A06;
	midiNotes[66]=SOLENOID_A07; midiNotes[67]=SOLENOID_A08;
	midiNotes[68]=SOLENOID_A09; midiNotes[69]=SOLENOID_A10;
	midiNotes[70]=SOLENOID_A11; midiNotes[71]=SOLENOID_A12;
	midiNotes[72]=SOLENOID_A13; midiNotes[73]=SOLENOID_A14;
	midiNotes[74]=SOLENOID_A15; midiNotes[75]=SOLENOID_A16;
	midiNotes[76]=SOLENOID_A17; midiNotes[77]=SOLENOID_A18;
	midiNotes[78]=SOLENOID_A19; midiNotes[79]=SOLENOID_A20;
	midiNotes[80]=SOLENOID_A21; midiNotes[81]=SOLENOID_A22;
	midiNotes[82]=SOLENOID_A23; midiNotes[83]=SOLENOID_A24;
	midiNotes[84]=SOLENOID_A25;

	//Turn on all LEDs
	led1 = 1;
	led2 = 1;
	led3 = 1;

	// Create a NSS object
	NSS nss = NSS(midiNotes);

	if(VERBOSE) printf("Initializing Communications\r\n");
	//Setup ethernet interface
	printf("!");
	EthernetInterface eth;
	printf("!");
	eth.connect();
	printf("!\r\n");
	if(VERBOSE) printf("Client IP Address:     %s\r\n", eth.get_ip_address());
	led3 = 0;	//turn off red LED

	//Setup OSC client
	OSCClient osc(&eth, instrumentName);
	osc.connect();
	if(VERBOSE) printf("OSC controller IP Address: %s\r\n", osc.get_controller_ip());
	osc.initBuffer();
	if(VERBOSE) printf("OSC buffer initialized\r\n");
	led2 = 0; //turn off blue led

	//Create a pointer to an OSC Message
	OSCMessage* msg = NULL;

	while(1) {
		//get a new message
		//osc.waitForMessage(msg);
		while( osc.getMessageFromQueue(&msg) == 0) {
			//do nothing
		}
		
		//Check that the message is for this instrument
		if(strcmp(msg->address, "/OSC-Test/play") == 0) {

			if(strcmp(msg->format, ",ii") == 0) { // two ints
					
				uint8_t pitch 	  = osc.getIntAtIndex(msg, 0);
				uint8_t velocity = osc.getIntAtIndex(msg, 1);

				//printf("%p\r\n", (void*)msg);
				//printf("%d %d\r\n",pitch,velocity);

				if(velocity > 0) nss.noteOn(pitch);
				else             nss.noteOff(pitch);
			}
		}

		// Turn off all notes
		else if(strcmp(msg->address, "/OSC-Test/allNotesOff") == 0) {
			nss.allOff();
		}
		
		else {
			//Not intended for this instrument
		}
		
		osc.freeMessage(msg);
  	}
}