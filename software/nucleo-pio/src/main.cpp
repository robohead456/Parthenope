//#define OS_MAINSTKSIZE          16384


#include <mbed.h>
#include <map>
#include "NSS.h"
#include "EthernetInterface.h"
#include "osc_client.h"
#include "ODriveMbed.h"

/* ---------------------------------------------------------------------------------------- */

#define VERBOSE 				(1)

#define DISK_MOTOR_POLE_PAIRS 	(2)
#define DISK1_AXIS				(0)
#define DISK2_AXIS				(1)

#define PI           			(3.14159265358979323846)

#define DISKSTATE_WAITING 		(0)
#define DISKSTATE_PLAYING 		(1)

#define STATE_IDLE 				(0)
#define STATE_SINGLEREGISTER 	(1)
#define STATE_MULTIREGISTER  	(2)

#define VOLUME_LEVELS			(3)

#define DEFAULT_NOTE			(69)
#define DEFAULT_HOLE			(0)
#define DEFAULT_VOLUME			(2)

#define NUM_DISKS				(2)
#define NUM_HOLE_SETS			(4)

#define IP_INSTRUMENT "192.168.2.11"
#define IP_CONTROLLER "192.168.2.9"
#define IP_GATEWAY	  "255.255.255.0"

/* ---------------------------------------------------------------------------------------- */

const char* instrumentName = "Parthenope";

DigitalOut led1(LED1);	//Green running LED
DigitalOut led2(LED2);	//Blue controller LED
DigitalOut led3(LED3);	//Red ethernet LED

uint8_t numHoles[NUM_DISKS][NUM_HOLE_SETS] = {
								{9, 10, 11, 12},
								{9, 10, 11, 12}
							};

DigitalOut solenoids[NUM_DISKS][VOLUME_LEVELS][NUM_HOLE_SETS] = { 
							{ // disk 0
								{SOLENOID_A11, // vol level 0
						  		 SOLENOID_A12,
						  		 SOLENOID_A09,
						  		 SOLENOID_A10},
								{SOLENOID_A07, // vol level 1
						  		 SOLENOID_A08,
						  		 SOLENOID_A05,
						  		 SOLENOID_A06},
								{SOLENOID_A03, // vol level 2
						  		 SOLENOID_A04,
						  		 SOLENOID_A01,
						  		 SOLENOID_A02}   
							},
							{ // disk 1
								{SOLENOID_A23, // vol level 0
						  		 SOLENOID_A24,
						  		 SOLENOID_A21,
						  		 SOLENOID_A22},
								{SOLENOID_A19, // vol level 1
						  		 SOLENOID_A20,
						  		 SOLENOID_A17,
						  		 SOLENOID_A18},
								{SOLENOID_A15, // vol level 2
						  		 SOLENOID_A16,
						  		 SOLENOID_A13,
						  		 SOLENOID_A14}
							}
						};

uint8_t diskAxis[NUM_DISKS] = {DISK1_AXIS, DISK2_AXIS};
uint8_t currentState = STATE_SINGLEREGISTER;
uint8_t requestedState = STATE_SINGLEREGISTER;
uint8_t currentNote[NUM_DISKS] = {DEFAULT_NOTE, DEFAULT_NOTE};
uint8_t currentHole[NUM_DISKS] = {DEFAULT_HOLE, DEFAULT_HOLE};
uint8_t currentVolume[NUM_DISKS] = {DEFAULT_VOLUME, DEFAULT_VOLUME};
uint8_t currentDiskState[NUM_DISKS] = {DISKSTATE_WAITING, DISKSTATE_WAITING};

// single register variables
uint8_t currentDisk = 1;
uint8_t singleRegisterState = DISKSTATE_WAITING;
int8_t  diskOffset[NUM_DISKS][NUM_DISKS] = { {  0, 10},
                                             {-10,  0}  };
/* Example layout for the disk offset variables

Each of the 4 disks has its first hole set spinning at a speed to play the note 10 above the
first hole set of the previous one

first value  = currentDisk
second value = disk to compare to
uint8_t diskOffset[NUM_DISKS][NUM_DISKS] = { {  0, 10, 20, 30},
											 {-10,  0, 10, 20},
											 {-20,-10,  0, 10},
											 {-30,-20,-10,  0}  };
*/

// multi register variables

// serial interfaces
Serial pc(USBTX,USBRX);
Serial odriveSerial(PG_14,PG_9);

// odrive object
ODriveMbed odrv(odriveSerial);

// Pointer to an OSC Message
OSCMessage* msg = (OSCMessage*)malloc(sizeof(OSCMessage*));

// Networking objects
EthernetInterface eth;
OSCClient *gOSC = NULL;

/* ---------------------------------------------------------------------------------------- */
//  FUNCTION PROTOTYPES

uint8_t mapVelocity(uint8_t x);
float getSpeed(uint8_t midiNote, uint8_t holeSet, uint8_t disk);
void setSpeed(uint8_t disk, uint8_t midiNote, uint8_t holeSet);
void stopDisks(void);
void startDisks(void);
void solenoidOn(uint8_t disk, uint8_t holeSet, uint8_t volume);
void solenoidOff(uint8_t disk, uint8_t holeSet, uint8_t volume);
void singleRegisterPlay(uint8_t pitch, uint8_t velocity);
void multiRegisterPlay(uint8_t pitch, uint8_t velocity);
void handleOtherMessage(void);
void allSolenoidsOff(void);
void swapStateWhileRunning(void);

/* ---------------------------------------------------------------------------------------- */
//  MAIN CODE

int main() {

	// Setup serial ports
	pc.baud(9600);
  	odriveSerial.baud(115200);

	// Turn on all LEDs
	led1 = 1;
	led2 = 1;
	led3 = 1;

	if(VERBOSE) pc.printf("\r\n\r\n\r\nInitializing Communications\r\n");

	// Setup ethernet interface
	eth.connect();
	if(VERBOSE) pc.printf("Client IP Address:         %s\r\n", eth.get_ip_address());
	led3 = 0;	//turn off red LED

	// Setup OSC client
	OSCClient osc(&eth, instrumentName);
	osc.connect();
	if(VERBOSE) pc.printf("OSC controller IP Address: %s\r\n", osc.get_controller_ip());
	gOSC = &osc;
	led2 = 0; //turn off blue led

	wait(20); // to ensure that the motor is calibrated

	startDisks();

	// main loop
	while(1) {

		pc.printf("\r\nWaiting for a message\r\n");
		osc.waitForMessage(msg);
		pc.printf("\r\nGot a message\r\n");

		// Handle Mode-based messages
		switch(currentState) {
			case STATE_SINGLEREGISTER:
				// Handle Play Messages
				if( (strcmp(msg->address, "/Parthenope/play") == 0) && (strcmp(msg->format, ",ii") == 0) ) {

					pc.printf("For this instrument\r\n");

					uint8_t pitch	 = osc.getIntAtIndex(msg, 0);
					uint8_t velocity = osc.getIntAtIndex(msg, 1);

					pc.printf("%p\r\n", (void*)msg);
					pc.printf("%d %d\r\n",pitch,velocity);

					singleRegisterPlay(pitch, velocity);	
				
				}

				// else if( OTHER SINGLEREGISTER STATE MESSAGE )

				else { // not a state-dependent message
					handleOtherMessage();
				}
				break; //STATE_SINGLEREGISTER

			case STATE_MULTIREGISTER:
				// Handle Play Messages
				if( (strcmp(msg->address, "/Parthenope/play") == 0) && (strcmp(msg->format, ",ii") == 0) ) {

					//pc.printf("For this instrument\r\n");

					uint8_t pitch	 = osc.getIntAtIndex(msg, 0);
					uint8_t velocity = osc.getIntAtIndex(msg, 1);

					//pc.printf("%p\r\n", (void*)msg);
					//pc.printf("%d %d\r\n",pitch,velocity);

					multiRegisterPlay(pitch, velocity);	
				
				}

				// else if( OTHER MULTIREGISTER STATE MESSAGE )

				else { // not a state-dependent message
					handleOtherMessage();
				}
				break; //STATE_MULTIREGISTER

			case STATE_IDLE:
				// Handle setHoles messages
				if( (strcmp(msg->address, "/Parthenope/setHoles") == 0) && (strcmp(msg->format, ",iiiiiiii") == 0) ) {

					// TODO
				
				}

				// else if( OTHER IDLE STATE MESSAGE )

				else { // not a state-dependent message
					handleOtherMessage();
				}
				break; //STATE_MULTIREGISTER

			default:
				break;
		}
  	}
}

/* ---------------------------------------------------------------------------------------- */
//  FUNCTIONS

// x is the MIDI velocity [0, 127]
// the return value is [0, VOLUME_LEVELS-1]
uint8_t mapVelocity(uint8_t x) {
	// [0, 63]   -> 0
	// [64, 126] -> 1
	// [127]     -> 2
	return ((x * (VOLUME_LEVELS-1) / 127));
}

void allSolenoidsOff(void) {
	for(uint8_t d = 0; d < NUM_DISKS; d++) {
		for(uint8_t v = 0; v < VOLUME_LEVELS; v++) {
			for(uint8_t i = 0; i < NUM_HOLE_SETS; i++) {
				solenoids[d][v][i] = 0;
			}
		}
		currentDiskState[d] = DISKSTATE_WAITING;
	}
	singleRegisterState = DISKSTATE_WAITING;
}

float getSpeed(uint8_t midiNote, uint8_t holeSet, uint8_t disk) {
	
	if( (disk < NUM_DISKS) && (holeSet < NUM_HOLE_SETS) )
		return ((440.0 * pow(pow(2,1.0/12.0), (midiNote-69)) / numHoles[disk][holeSet]) * 2.0 * PI * DISK_MOTOR_POLE_PAIRS);
		//return ((440.0 * pow(pow(2,1.0/12.0), (midiNote-69)) / numHoles[disk][holeSet]) * 60.0  * 2.0 * PI / 60.0 * DISK_MOTOR_POLE_PAIRS); worked
	return -1.0;
}

void setSpeed(uint8_t disk, uint8_t midiNote, uint8_t holeSet) {
	
	float speed = getSpeed(midiNote, holeSet, disk);
	
	if(speed > 0) {
		printf("Setting Velocity %5f rad/s\r\n",speed);
		odrv.setVelocity(diskAxis[disk], speed);
		printf("Done\r\n");
	}
}

void stopDisks(void) {
	setSpeed(0, DEFAULT_NOTE, DEFAULT_HOLE);
	setSpeed(1, DEFAULT_NOTE, DEFAULT_HOLE);
	wait(4);
	odrv.run_state(DISK1_AXIS, ODriveMbed::AXIS_STATE_IDLE, 0);
	odrv.run_state(DISK2_AXIS, ODriveMbed::AXIS_STATE_IDLE, 0);
}

void startDisks(void) {

	printf("disk0 mode set\r\n");
	odrv.run_state(DISK1_AXIS, ODriveMbed::AXIS_STATE_SENSORLESS_CONTROL, 0);
	odrv.setControlMode(DISK1_AXIS, ODriveMbed::CTRL_MODE_VELOCITY_CONTROL, 0);
	wait(10);
	printf("disk0 mode set\r\n");
	odrv.run_state(DISK1_AXIS, ODriveMbed::AXIS_STATE_SENSORLESS_CONTROL, 0);
	odrv.setControlMode(DISK1_AXIS, ODriveMbed::CTRL_MODE_VELOCITY_CONTROL, 0);
	wait(10);
	printf("disk0 speed set\r\n");
	setSpeed(0, DEFAULT_NOTE, DEFAULT_HOLE);
	wait(10);
	printf("disk1 mode set\r\n");
	odrv.run_state(DISK2_AXIS, ODriveMbed::AXIS_STATE_SENSORLESS_CONTROL, 0);
	odrv.setControlMode(DISK2_AXIS, ODriveMbed::CTRL_MODE_VELOCITY_CONTROL, 0);
	wait(10);
	printf("disk1 speed set\r\n");
	setSpeed(1, DEFAULT_NOTE, DEFAULT_HOLE);

	
}

void solenoidOn(uint8_t disk, uint8_t holeSet, uint8_t volume) {
	solenoids[disk][volume][holeSet] = 1;		
}

void solenoidOff(uint8_t disk, uint8_t holeSet, uint8_t volume) {
	solenoids[disk][volume][holeSet] = 0;
}

void singleRegisterPlay(uint8_t pitch, uint8_t velocity) {
	if(velocity > 0) { // note on message
		if(singleRegisterState == DISKSTATE_WAITING) { // not playing a note

			// calculate which hole requires the least velocity change
			float minVelChange = infinityf();
			int8_t minVelHole = -1;
			int8_t minVelDisk = -1;

			float currentSpeeds[NUM_DISKS];

			for(uint8_t d = 0; d < NUM_DISKS; d++) {
				currentSpeeds[d] = getSpeed(currentNote[d], currentHole[d], d);
				for(uint8_t i = 0; i < NUM_HOLE_SETS; i++) {
					float diff = abs(getSpeed(pitch, i, d) - currentSpeeds[d]);
					//pc.printf("Hole set %d, change %f\r\n", i, diff);
					if( diff < minVelChange ) {
						minVelHole = i;
						minVelDisk = d;
						minVelChange = diff;
						//pc.printf("Set to hole set %d",i);
					}
				}
			}

			//pc.printf("Best hole set %d", minVelHole);

			// Set the speed and play the note
			if(minVelHole != -1) {
				currentDisk = minVelDisk;

				currentNote[currentDisk] = pitch;
				currentHole[currentDisk] = minVelHole;
				currentVolume[currentDisk] = mapVelocity(velocity);
				singleRegisterState = DISKSTATE_PLAYING;

				for(uint8_t d = 0; d < NUM_DISKS; d++) {
					if(d != currentDisk) {
						currentNote[d] = pitch + diskOffset[currentDisk][d];
						currentHole[d] = DEFAULT_HOLE;
					}
					setSpeed(d, currentNote[d], currentHole[d]);
				}

				solenoidOn(currentDisk, currentHole[currentDisk], currentVolume[currentDisk]);
			}
		} // end turn on

		else { //playing a note - glissando up/down
			//TODO
		} // end glissando
	}
	else { // note off message

		singleRegisterState = DISKSTATE_WAITING;
		solenoidOff(currentDisk, currentHole[currentDisk], currentVolume[currentDisk]);

	} // end note off
} // end single register play function

void multiRegisterPlay(uint8_t pitch, uint8_t velocity) {
	//TODO
}

void handleOtherMessage(void) {

	// Turn off all notes
	if(strcmp(msg->address, "/Parthenope/allNotesOff") == 0) {
		//pc.printf("All notes off\r\n");
		allSolenoidsOff();		
	} // allNotesOff message

	// Set Playing Mode
	else if( (strcmp(msg->address, "/Parthenope/setMode") == 0) && (strcmp(msg->format, ",i") == 0) ) {
		//pc.printf("Set Mode\r\n");
		uint8_t m = gOSC->getIntAtIndex(msg, 0);		

		if(m == 1) requestedState = STATE_SINGLEREGISTER;
		else if(m == 2) requestedState = STATE_MULTIREGISTER;
		// else ignore

		if(currentState != STATE_IDLE) {
			currentState = requestedState;
			swapStateWhileRunning();
		}
	} // setMode message

	// Turn off disks
	else if(strcmp(msg->address, "/Parthenope/stopDisks") == 0) {
		pc.printf("Stop Disks\r\n");
		if(currentState != STATE_IDLE) {
			requestedState = currentState;
			currentState = STATE_IDLE;
			allSolenoidsOff();
			printf("sd");
			stopDisks();
			pc.printf("Disks Stopped\r\n");
		}
		else {
			pc.printf("Disks already stopped\r\n");
		}
	} // stopDisk message

	// Turn on disk
	else if(strcmp(msg->address, "/Parthenope/startDisk") == 0) {
		pc.printf("Start Disk\r\n");
		if(currentState == STATE_IDLE) {
			currentState = requestedState;
			if(currentState == STATE_SINGLEREGISTER) {
				singleRegisterState = DISKSTATE_WAITING;
			}
			else { // (currentState = STATE_MULTIREGISTER) 
				for(uint8_t d = 0; d < NUM_DISKS; d++) {
					currentDiskState[d] = DISKSTATE_WAITING;
				}
			}
			startDisks();
			//pc.printf("Disks Started\r\n");
		}
		else {
			//pc.printf("Disks already started\r\n");
		}
	}

	// else do nothing
}

void swapStateWhileRunning(void) {
	//TODO
}