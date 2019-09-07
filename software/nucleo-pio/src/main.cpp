#include <mbed.h>
#include <map>
#include "NSS.h"
#include "EthernetInterface.h"
#include "osc_client.h"
#include "ODriveMbed.h"

/* ---------------------------------------------------------------------------------------- */

#define VERBOSE 				(1)

#define DISK_MOTOR_POLE_PAIRS 	(2)
#define DISK_AXIS				(1)

#define PI           			(3.14159265358979323846)

#define STATE_DISABLED			(2)
#define STATE_PLAYING 			(1)
#define STATE_NOT_PLAYING 		(0)

#define MODE_MONOPHONIC_REGULAR (0)
#define MODE_POLYPHONIC_REGULAR (1)
#define MODE_MONOPHONIC_LEGATO  (2)
#define MODE_POLYPHONIC_LEGATO	(3)
#define MODE_DIRECT_CONTROL		(4)

#define VOLUME1					(1)
#define VOLUME2 				(2)
#define VOLUME3 				(3)

#define DEFAULT_NOTE			(45)
#define DEFAULT_HOLE			(0)
#define DEFAULT_STATE			STATE_NOT_PLAYING
#define DEFAULT_VOLUME			VOLUME3

/* ---------------------------------------------------------------------------------------- */

const char* instrumentName = "Parthenope";

DigitalOut led1(LED1);	//Green running LED
DigitalOut led2(LED2);	//Blue controller LED
DigitalOut led3(LED3);	//Red ethernet LED

uint8_t numHoleSets = 4;
uint8_t numHoles[] = {4, 6, 8, 10};

DigitalOut solenoids_vol3[] =  {SOLENOID_A03,
						  		SOLENOID_A04,
						  		SOLENOID_A01,
						  		SOLENOID_A02};
DigitalOut solenoids_vol2[] =  {SOLENOID_A08,
						  		SOLENOID_A09,
						  		SOLENOID_A06,
						  		SOLENOID_A07};
DigitalOut solenoids_vol1[] =  {SOLENOID_A13,
						  		SOLENOID_A14,
						  		SOLENOID_A11,
						  		SOLENOID_A12};						  								  								  																  

uint8_t currentNote = DEFAULT_NOTE;
uint8_t currentHole = DEFAULT_HOLE;
uint8_t currentState = DEFAULT_STATE;
uint8_t currentVolume = DEFAULT_VOLUME;

/* ---------------------------------------------------------------------------------------- */

uint8_t mapVelocity(uint8_t x) {
	return ((x * 5 / 127) + 1);
}

float getSpeed(uint8_t midiNote, uint8_t holeSet) {
	
	if(holeSet < numHoleSets) {
		return ((440.0 * pow(pow(2,1.0/12.0), (midiNote-69)) / numHoles[holeSet]) * 60.0  * 2.0 * PI / 60.0 * DISK_MOTOR_POLE_PAIRS);
	}
	else return -1;	
}

void setSpeed(ODriveMbed odrv, uint8_t midiNote, uint8_t holeSet) {
	
	float speed = getSpeed(midiNote, holeSet);
	
	if(speed > 0) {
		printf("Setting Velocity\r\n");
		odrv.setVelocity(DISK_AXIS, speed);
		printf("Done\r\n");
	}
}

void stopDisk(ODriveMbed odrv) {
	setSpeed(odrv, DEFAULT_NOTE, DEFAULT_HOLE);
	wait(4);
	odrv.run_state(DISK_AXIS, ODriveMbed::AXIS_STATE_IDLE, 0);
}

void startDisk(ODriveMbed odrv) {
	odrv.run_state(DISK_AXIS, ODriveMbed::AXIS_STATE_SENSORLESS_CONTROL, 0);
	odrv.setControlMode(DISK_AXIS, ODriveMbed::CTRL_MODE_VELOCITY_CONTROL, 0);
	wait(4);
	setSpeed(odrv, DEFAULT_NOTE, DEFAULT_HOLE);
}

void solenoidOn(uint8_t holeSet, uint8_t volume) {
	if(holeSet < numHoleSets) {
		currentVolume = volume;
		switch(volume) {
			case VOLUME1:
				solenoids_vol1[holeSet] = 1;
				break;
			case VOLUME2:
				solenoids_vol2[holeSet] = 1;
				break;
			default:
				solenoids_vol3[holeSet] = 1;
				break;
		}
	} 
}

void solenoidOff(uint8_t holeSet) {
	if(holeSet < numHoleSets) {
		switch(currentVolume) {
			case VOLUME1:
				solenoids_vol1[holeSet] = 0;
				break;
			case VOLUME2:
				solenoids_vol2[holeSet] = 0;
				break;
			default:
				solenoids_vol3[holeSet] = 0;
				break;
		}
	} 
}

/* ---------------------------------------------------------------------------------------- */

int main() {

	Serial pc(USBTX,USBRX);
	pc.baud(9600);
	Serial odriveSerial(PG_14,PG_9);
  	odriveSerial.baud(115200);

	ODriveMbed odrv(odriveSerial);

	//Turn on all LEDs
	led1 = 1;
	led2 = 1;
	led3 = 1;

	if(VERBOSE) pc.printf("\r\n\r\n\r\nInitializing Communications\r\n");

	//Setup ethernet interface
	EthernetInterface eth;
	eth.connect();
	if(VERBOSE) pc.printf("Client IP Address:         %s\r\n", eth.get_ip_address());
	led3 = 0;	//turn off red LED

	//Setup OSC client
	OSCClient osc(&eth, instrumentName);
	osc.connect();
	if(VERBOSE) pc.printf("OSC controller IP Address: %s\r\n", osc.get_controller_ip());
	led2 = 0; //turn off blue led

	//Create a pointer to an OSC Message
	OSCMessage* msg = (OSCMessage*)malloc(sizeof(OSCMessage*));

	wait(10); // to ensure that the motor is calibrated

	startDisk(odrv);

	while(1) {

		osc.waitForMessage(msg);
		pc.printf("\r\nGot a message\r\n");
		
		//Check that the message is for this instrument
		if(strcmp(msg->address, "/Parthenope/play") == 0) {

			pc.printf("For this instrument\r\n");

			if(strcmp(msg->format, ",ii") == 0) { // two ints

				pc.printf("ii format\r\n");
					
				uint8_t pitch	 = osc.getIntAtIndex(msg, 0);
				uint8_t velocity = osc.getIntAtIndex(msg, 1);

				//pc.printf("%p\r\n", (void*)msg);
				//pc.printf("%d %d\r\n",pitch,velocity);

				if(currentState != STATE_DISABLED) {
					if(velocity > 0) { // note on message
						if(currentState == STATE_NOT_PLAYING) { // not playing a note

							// calculate which hole requires the least velocity change
							float minVelChange = infinityf();
							int8_t minVelHole = -1;
							for(uint8_t i = 0; i < numHoleSets; i++) {
								float diff = abs(getSpeed(pitch, i) - getSpeed(currentNote, currentHole));
								pc.printf("Hole set %d, change %f\r\n", i, diff);
								if( diff < minVelChange ) {
									minVelHole = i;
									minVelChange = diff;
									pc.printf("Set to hole set %d",i);
								}
							}

							pc.printf("Best hole set %d", minVelHole);

							// Set the speed and play the note
							if(minVelHole != -1) {
								currentNote = pitch;
								currentHole = minVelHole;
								currentState = STATE_PLAYING;

								setSpeed(odrv, currentNote, currentHole);
								solenoidOn(currentHole, mapVelocity(velocity));
							}

						}
						else { //playing a note - glissando up/down
							//TODO
						}
					}
					else { // note off message
						if(pitch == currentNote) {
							currentState = STATE_NOT_PLAYING;

							solenoidOff(currentHole);
						}
						else {
							//do nothing
						}
					}
				}			
			}
		} // if(strcmp(msg->address, "/Parthenope/play") == 0)

		// Turn off all notes
		else if(strcmp(msg->address, "/Parthenope/allNotesOff") == 0) {
			if(currentState != STATE_DISABLED) {
				pc.printf("All notes off\r\n");
				for(uint8_t i = 0; i < numHoleSets; i++) {
					solenoids_vol1[i] = 0;
					solenoids_vol2[i] = 0;
					solenoids_vol3[i] = 0;
				}
				currentState = STATE_NOT_PLAYING;
			}
		}

		// Turn off disk
		else if(strcmp(msg->address, "/Parthenope/stopDisk") == 0) {
			pc.printf("Stop Disk\r\n");
			if(currentState != STATE_DISABLED) {
				currentState = STATE_DISABLED;
				for(uint8_t i = 0; i < numHoleSets; i++) {
					solenoids_vol1[i] = 0;
					solenoids_vol2[i] = 0;
					solenoids_vol3[i] = 0;
				}
				stopDisk(odrv);
				pc.printf("Disk Stopped\r\n");
			}
			else {
				pc.printf("Disk already stopped\r\n");
			}
		}

		// Turn on disk
		else if(strcmp(msg->address, "/Parthenope/startDisk") == 0) {
			pc.printf("Start Disk\r\n");
			if(currentState == STATE_DISABLED) {
				startDisk(odrv);
				pc.printf("Disk Started\r\n");
				currentState = STATE_NOT_PLAYING;
				currentNote = DEFAULT_NOTE;
				currentHole = DEFAULT_HOLE;
			}
			else {
				pc.printf("Disk already started\r\n");
			}
		}
		
		else {
			// Do nothing - not intended for this instrument
		}
  	}
}