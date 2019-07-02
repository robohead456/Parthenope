#include <mbed.h>
#include <map>
#include "NSS.h"
#include "EthernetInterface.h"
#include "osc_client.h"
#include "ODriveMbed.h"

/* ---------------------------------------------------------------------------------------- */

#define VERBOSE 				(1)
#define DISK_AXIS 				(1)
#define DISK_MOTOR_POLE_PAIRS 	(2)
#define LOW_DISK_HOLES 			(2)
#define HIGH_DISK_HOLES 		(2*LOW_DISK_HOLES)
#define HIGH_HOLE 				(1)
#define LOW_HOLE 				(0)
#define PI           			(3.14159265358979323846)
#define STATE_DISABLED			(2)
#define STATE_PLAYING 			(1)
#define STATE_NOT_PLAYING 		(0)

/* ---------------------------------------------------------------------------------------- */

const char* instrumentName = "OSC-Test";

//Setup digital outputs
DigitalOut led1(LED1);	//Green running LED
DigitalOut led2(LED2);	//Blue controller LED
DigitalOut led3(LED3);	//Red ethernet LED

DigitalOut lowHoleSolenoid(SOLENOID_A01);
DigitalOut highHoleSolenoid(SOLENOID_A02);

/* ---------------------------------------------------------------------------------------- */

void setSpeed(ODriveMbed odrv, uint8_t midiNote, uint8_t hole) {
	
	float speed = 0;
	if(hole == LOW_HOLE) {
		speed = ((440.0 * pow(pow(2,1.0/12.0), (midiNote-69)) / LOW_DISK_HOLES) * 60.0  * 2.0 * PI / 60.0 * DISK_MOTOR_POLE_PAIRS);
	}
	else if(hole == HIGH_HOLE) {
		speed = ((440.0 * pow(pow(2,1.0/12.0), (midiNote-69)) / HIGH_DISK_HOLES) * 60.0  * 2.0 * PI / 60.0 * DISK_MOTOR_POLE_PAIRS);
	}
	else return;
	
	printf("Setting Velocity\r\n");
	odrv.setVelocity(DISK_AXIS, speed);
	printf("Done\r\n");
	
}

void stopDisk(ODriveMbed odrv) {
	setSpeed(odrv, 45, LOW_HOLE);
	wait(4);
	odrv.run_state(DISK_AXIS, ODriveMbed::AXIS_STATE_IDLE, 0);
}

void startDisk(ODriveMbed odrv) {
	odrv.run_state(DISK_AXIS, ODriveMbed::AXIS_STATE_SENSORLESS_CONTROL, 0);
	odrv.setControlMode(DISK_AXIS, ODriveMbed::CTRL_MODE_VELOCITY_CONTROL, 0);
	wait(4);
	setSpeed(odrv, 45, LOW_HOLE);
}

void solenoidOn(uint8_t hole) {
	if(hole == HIGH_HOLE) 
		highHoleSolenoid = 1;
	else if(hole == LOW_HOLE)
		lowHoleSolenoid = 1;
	return;	
}

void solenoidOff(uint8_t hole) {
	if(hole == HIGH_HOLE) 
		highHoleSolenoid = 0;
	else if(hole == LOW_HOLE)
		lowHoleSolenoid = 0;
	return;	
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
	if(VERBOSE) pc.printf("Client IP Address:     %s\r\n", eth.get_ip_address());
	led3 = 0;	//turn off red LED

	//Setup OSC client
	OSCClient osc(&eth, instrumentName);
	osc.connect();
	if(VERBOSE) pc.printf("OSC controller IP Address: %s\r\n", osc.get_controller_ip());
	led2 = 0; //turn off blue led

	//Create a pointer to an OSC Message
	OSCMessage* msg = (OSCMessage*)malloc(sizeof(OSCMessage*));

	uint8_t currentNote = 45;
	uint8_t hole = LOW_HOLE;
	uint8_t state = STATE_NOT_PLAYING;

	wait(10); // to ensure that the motor is calibrated

	startDisk(odrv);

	while(1) {

		osc.waitForMessage(msg);
		pc.printf("\r\nGot a message\r\n");
		
		//Check that the message is for this instrument
		if(strcmp(msg->address, "/OSC-Test/play") == 0) {

			pc.printf("For this instrument\r\n");

			if(strcmp(msg->format, ",ii") == 0) { // two ints

				pc.printf("ii format\r\n");
					
				uint8_t pitch	 = osc.getIntAtIndex(msg, 0);
				uint8_t velocity = osc.getIntAtIndex(msg, 1);

				//pc.printf("%p\r\n", (void*)msg);
				//pc.printf("%d %d\r\n",pitch,velocity);
				if(state != STATE_DISABLED) {
						if(velocity > 0) {
						pc.printf("note on message: %d\r\n",pitch);
						if(pitch == currentNote) { // The disk is already at the required velocity
							pc.printf("current note\r\n");
							solenoidOn(hole);
							currentNote = pitch;
							state = STATE_PLAYING;
						}
						else if((hole == LOW_HOLE) && (pitch == currentNote+12)) { // The disk is at a correct velocity for the high hole
							pc.printf("current speed but on high hole\r\n");
							hole = HIGH_HOLE;
							solenoidOn(hole);
							currentNote = pitch;
							state = STATE_PLAYING;
						}
						else if((hole == HIGH_HOLE) && (pitch == currentNote-12)) { // The disk is at a correct velocity for the high hole
							pc.printf("current speed but on low hole\r\n");
							hole = LOW_HOLE;
							solenoidOn(hole);
							currentNote = pitch;
							state = STATE_PLAYING;
						}
						else if(state == STATE_NOT_PLAYING) { // The disk velocity needs to be changed to the correct velocity befor playing
							if(hole == HIGH_HOLE) {
								pc.printf("not playing - currently on high hole\r\n");
								if(pitch > currentNote) {
									pc.printf("high hole speed up\r\n");
									setSpeed(odrv, pitch, hole);
									pc.printf("Set speed\r\n");
									//TODO delay?
									solenoidOn(hole);
									pc.printf("solenoid on\r\n");
									currentNote = pitch;
									state = STATE_PLAYING;
									pc.printf("Set states\r\n");
								}
								else { // (pitch < currentNote)
									if(currentNote-pitch < 6) {
										pc.printf("high hole slow down\r\n");
										setSpeed(odrv, pitch, hole);
										//TODO delay? 
										solenoidOn(hole);
										currentNote = pitch;
										state = STATE_PLAYING;
									}
									else { //(currentNote-pitch > 6)
										pc.printf("low hole slow down\r\n");
										setSpeed(odrv, pitch, hole);
										//TODO delay?
										hole = LOW_HOLE;
										solenoidOn(hole);
										currentNote = pitch;
										state = STATE_PLAYING;
									}
								}
							}
							else { // (hole == LOW_HOLE)
								pc.printf("not playing - currently on low hole\r\n");
								if(pitch < currentNote) {
									pc.printf("low hole slow down\r\n");
									setSpeed(odrv, pitch, hole);
									//TODO delay?
									solenoidOn(hole);
									currentNote = pitch;
									state = STATE_PLAYING;
								}
								else { // (pitch > currentNote)
									if(pitch - currentNote < 6) {
										pc.printf("low hole speed up\r\n");
										setSpeed(odrv, pitch, hole);
										//TODO delay?
										solenoidOn(hole);
										currentNote = pitch;
										state = STATE_PLAYING;
									}
									else { // (pitch - currentNote > 6)
										pc.printf("high hole speed up\r\n");
										setSpeed(odrv, pitch, hole);
										//TODO delay?
										hole = HIGH_HOLE;
										solenoidOn(hole);
										currentNote = pitch;
										state = STATE_PLAYING;
									}
								}
							}
						}
						else { // currently playing a note and should glissando up to it
							//TODO
						}
					}
					else { // velocity == 0
						if(pitch == currentNote) {
							pc.printf("note off message\r\n");
							solenoidOff(hole);
							state = STATE_NOT_PLAYING;
						}
						else { // got an off message for a different note and should ignore

						}
					}
				}
				else { // state == STATE_DISABLED
					// do nothing
				}
			}
		}

		// Turn off all notes
		else if(strcmp(msg->address, "/OSC-Test/allNotesOff") == 0) {
			if(state != STATE_DISABLED) {
				pc.printf("All notes off\r\n");
				solenoidOff(HIGH_HOLE);
				solenoidOff(LOW_HOLE);
				state = STATE_NOT_PLAYING;
			}
		}

		// Turn off disk
		else if(strcmp(msg->address, "/OSC-Test/stopDisk") == 0) {
			pc.printf("Stop Disk\r\n");
			if(state != STATE_DISABLED) {
				state = STATE_DISABLED;
				stopDisk(odrv);
				solenoidOff(HIGH_HOLE);
				solenoidOff(LOW_HOLE);
				pc.printf("Disk Stopped\r\n");
			}
			else {
				pc.printf("Disk already stopped\r\n");
			}
		}

		// Turn off disk
		else if(strcmp(msg->address, "/OSC-Test/startDisk") == 0) {
			pc.printf("Start Disk\r\n");
			if(state == STATE_DISABLED) {
				startDisk(odrv);
				pc.printf("Disk Started\r\n");
				state = STATE_NOT_PLAYING;
				currentNote = 45;
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