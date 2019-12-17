#include <Arduino.h>

/* ---------------------------------------------------------------------------------------- */
//  CONSTANTS

#define VERBOSE 				        (1)

#define DISK_MOTOR_POLE_PAIRS 	(2)
#define DISK1_AXIS			      	(0)
#define DISK2_AXIS			      	(1)

#define PI           	      		(3.14159265358979323846)

#define DISKSTATE_WAITING 	  	(0)
#define DISKSTATE_PLAYING 	  	(1)

#define STATE_IDLE 			      	(0)
#define STATE_SINGLEREGISTER  	(1)
#define STATE_MULTIREGISTER   	(2)

#define VOLUME_LEVELS			      (3)

#define DEFAULT_NOTE	      		(69)
#define DEFAULT_HOLE	      		(0)
#define DEFAULT_VOLUME	    		(2)

#define NUM_DISKS			        	(2)
#define NUM_HOLE_SETS	      		(4)

#define IP_INSTRUMENT "192.168.2.11"
#define IP_CONTROLLER "192.168.2.9"
#define IP_GATEWAY	  "255.255.255.0"

/* ---------------------------------------------------------------------------------------- */
//  FUNCTION PROTOTYPES

uint8_t mapVelocity(uint8_t x);
float getSpeed(uint8_t midiNote, uint8_t holeSet, uint8_t disk);
float setSpeed(uint8_t midiNote, uint8_t holeSet, uint8_t disk);
uint8_t getNote_HS0(uint8_t midiNote, uint8_t holeSet, uint8_t disk);
void stopDisks(void);
void startDisks(void);
void solenoidOn(uint8_t disk, uint8_t holeSet, uint8_t volume);
void solenoidOff(uint8_t disk, uint8_t holeSet, uint8_t volume);
void singleRegisterPlay(uint8_t pitch, uint8_t velocity);
void multiRegisterPlay(uint8_t pitch, uint8_t velocity);
void handleOtherMessage(void);
void allSolenoidsOff(void);
void swapStateWhileRunning(void);

void testSequence(uint8_t pitch);

/* ---------------------------------------------------------------------------------------- */
//  VARIABLES

const char* instrumentName = "Parthenope";

uint8_t numHoles[NUM_DISKS][NUM_HOLE_SETS] = {
								{9, 10, 11, 12},
								{9, 10, 11, 12}
							};

const uint8_t solenoidPins[NUM_DISKS][VOLUME_LEVELS][NUM_HOLE_SETS] = { 
  { // disk 0
    {2, // vol level 0
     3,
     4,
     5},
    {6, // vol level 1
     7,
     8,
     9},
    {11, // vol level 2
     12,
     13,
     SDA}   
  },
  { // disk 1
    {A5, // vol level 0
     A4,
     A3,
     A2},
    {A1, // vol level 1
     A0,
     0,
     0},
    {31, // vol level 2
     30,
     38,
     SCL}
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
uint8_t currentDisk = 0;
uint8_t singleRegisterState = DISKSTATE_WAITING;
int8_t  diskOffset[NUM_DISKS][NUM_DISKS] = { { 0, 7},
                                             {-7, 0}  };
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
// TODO
//Serial pc(USBTX,USBRX);
//Serial odriveSerial(PG_14,PG_9);

// odrive object
// TODO
//ODriveMbed odrv(odriveSerial);

// Pointer to an OSC Message
// TODO
//OSCMessage* msg = (OSCMessage*)malloc(sizeof(OSCMessage*));

// Networking objects
// TODO
//EthernetInterface eth;
//OSCClient *gOSC = NULL;

/* ---------------------------------------------------------------------------------------- */
//  MAIN CODE

void setup() {

  SerialUSB.begin(9600);

  delay(5000);

  SerialUSB.println("Starting Setup");

  for(uint8_t d = 0; d < NUM_DISKS; d++) {
    for(uint8_t v = 0; v < VOLUME_LEVELS; v++) {
      for(uint8_t h = 0; h < NUM_HOLE_SETS; h++) {
        pinMode(solenoidPins[d][v][h], OUTPUT);
      }
    }
  }

  SerialUSB.println("Starting Disks");

  startDisks();

}

void loop() {

  /*
  for(uint8_t d = 0; d < NUM_DISKS; d++) {
    for(uint8_t v = 0; v < VOLUME_LEVELS; v++) {
      for(uint8_t h = 0; h < NUM_HOLE_SETS; h++) {
        digitalWrite(solenoidPins[d][v][h], HIGH);
        delay(500);
      }
    }
  }
  for(uint8_t d = 0; d < NUM_DISKS; d++) {
    for(uint8_t v = 0; v < VOLUME_LEVELS; v++) {
      for(uint8_t h = 0; h < NUM_HOLE_SETS; h++) {
        digitalWrite(solenoidPins[d][v][h], LOW);
        delay(500);
      }
    }
  }
  */

  SerialUSB.println("Loop Start");
  // test single register on/off
  testSequence(69);
  testSequence(71);
  testSequence(73);
  testSequence(74);
  testSequence(76);
  testSequence(78);
  testSequence(80);
  testSequence(81);

  // test single register glissando
  singleRegisterPlay(69,1);
  delay(1000);
  singleRegisterPlay(73,1);
  delay(1000);
  singleRegisterPlay(69,1);
  delay(1000);
  singleRegisterPlay(69,0);
  delay(1000);

}

/* ---------------------------------------------------------------------------------------- */
//  FUNCTIONS

// x is the MIDI velocity [0, 127]
// the return value is [0, VOLUME_LEVELS-1]
uint8_t mapVelocity(uint8_t x) {
	// [1, 63]   -> 0
	// [64, 126] -> 1
	// [127]     -> 2
	return ((x * (VOLUME_LEVELS-1) / 127));
}

void allSolenoidsOff(void) {
	for(uint8_t d = 0; d < NUM_DISKS; d++) {
		for(uint8_t v = 0; v < VOLUME_LEVELS; v++) {
			for(uint8_t i = 0; i < NUM_HOLE_SETS; i++) {
				digitalWrite(solenoidPins[d][v][i], LOW);
			}
		}
		currentDiskState[d] = DISKSTATE_WAITING;
	}
	singleRegisterState = DISKSTATE_WAITING;
}

float getSpeed(uint8_t midiNote, uint8_t holeSet, uint8_t disk) {
	
	if( (disk < NUM_DISKS) && (holeSet < NUM_HOLE_SETS) )
		return (440.0f * powf(2,(midiNote-69)/12.0f) / (float)numHoles[disk][holeSet] * 2.0f * PI * (float)DISK_MOTOR_POLE_PAIRS);
		//return ((440.0 * pow(pow(2,1.0/12.0), (midiNote-69)) / numHoles[disk][holeSet]) * 60.0  * 2.0 * PI / 60.0 * DISK_MOTOR_POLE_PAIRS); worked
	return -1.0;
}

uint8_t getNote_HS0(uint8_t midiNote, uint8_t holeSet, uint8_t disk) {
	
	if( (disk < NUM_DISKS) && (holeSet < NUM_HOLE_SETS) ) {
    float speed = getSpeed(midiNote, holeSet, disk);

    speed = (speed/2.0f/PI/(float)DISK_MOTOR_POLE_PAIRS*numHoles[disk][DEFAULT_HOLE]); // sets the speed to the frequency (Hz)

    return ((12 * log2f(speed/440.0f)) + 69);

  }
		
	return 0;
}

float setSpeed(uint8_t midiNote, uint8_t holeSet, uint8_t disk) {
	
	float speed = getSpeed(midiNote, holeSet, disk);
	
	if(speed > 0) {
		//SerialUSB.println("Setting Speed");
		//odrv.setVelocity(diskAxis[disk], speed);
		//SerialUSB.println("Done");
	}

  return speed;
}

void stopDisks(void) {
	setSpeed(0, DEFAULT_NOTE, DEFAULT_HOLE);
	setSpeed(1, DEFAULT_NOTE, DEFAULT_HOLE);
	delay(4000);
	//odrv.run_state(DISK1_AXIS, ODriveMbed::AXIS_STATE_IDLE, 0);
	//odrv.run_state(DISK2_AXIS, ODriveMbed::AXIS_STATE_IDLE, 0);
}

void startDisks(void) {

  if(requestedState == STATE_MULTIREGISTER) {
    for(uint8_t i = 0; i < NUM_DISKS; i++) {
      SerialUSB.print("disk"); SerialUSB.println(i);
      SerialUSB.println("mode set to sensorless");
      //odrv.run_state(DISK1_AXIS, ODriveMbed::AXIS_STATE_SENSORLESS_CONTROL, 0);
      //odrv.setControlMode(DISK1_AXIS, ODriveMbed::CTRL_MODE_VELOCITY_CONTROL, 0);
      delay(1000); //10000
      currentNote[i] = DEFAULT_NOTE;
      currentHole[i] = DEFAULT_HOLE;
      float speed = setSpeed(currentNote[i], currentHole[i], i);
      SerialUSB.print("speed set to "); SerialUSB.println(speed);
      delay(1000); //10000
    }
  }

  else if(requestedState == STATE_SINGLEREGISTER) {
    for(uint8_t i = 0; i < NUM_DISKS; i++) {
      SerialUSB.print("disk"); SerialUSB.println(i);
      SerialUSB.println("mode set to sensorless");
      //odrv.run_state(DISK1_AXIS, ODriveMbed::AXIS_STATE_SENSORLESS_CONTROL, 0);
      //odrv.setControlMode(DISK1_AXIS, ODriveMbed::CTRL_MODE_VELOCITY_CONTROL, 0);
      delay(1000); //10000
      currentNote[i] = DEFAULT_NOTE + diskOffset[currentDisk][i];
      currentHole[i] = DEFAULT_HOLE;
      float speed = setSpeed(currentNote[i], currentHole[i], i);
      SerialUSB.print("speed set to "); SerialUSB.println(speed);
      delay(1000); //10000
    }
  }

  else {
    // do nothing
  }
}

void solenoidOn(uint8_t disk, uint8_t holeSet, uint8_t volume) {
	digitalWrite(solenoidPins[disk][volume][holeSet], HIGH);		
}

void solenoidOff(uint8_t disk, uint8_t holeSet, uint8_t volume) {
	digitalWrite(solenoidPins[disk][volume][holeSet], LOW);
}

void singleRegisterPlay(uint8_t pitch, uint8_t velocity) {
	if(velocity > 0) { // note on message
		if(singleRegisterState == DISKSTATE_WAITING) { // not playing a note

			// calculate which hole requires the least velocity change
			float minVelChange = infinityf();
			int8_t minVelHole = -1;
			int8_t minVelDisk = -1;

			volatile float currentSpeed = 0.0f;

			for(uint8_t d = 0; d < NUM_DISKS; d++) {
				currentSpeed = getSpeed(currentNote[d], currentHole[d], d);
				for(uint8_t i = 0; i < NUM_HOLE_SETS; i++) {
					volatile float diff = abs(getSpeed(pitch, i, d) - currentSpeed);
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
            volatile uint8_t basePitch = getNote_HS0(currentNote[currentDisk], currentHole[currentDisk], currentDisk);
						currentNote[d] = basePitch + diskOffset[currentDisk][d]; // shouldnt be pitch -> should be the pitch (MIDI Note) of the base hole of that disk
						currentHole[d] = DEFAULT_HOLE;
					}
					float speed = setSpeed(currentNote[d], currentHole[d], d);
          SerialUSB.print("disk"); SerialUSB.print(d);
          SerialUSB.print(" speed set to "); SerialUSB.println(speed);
				}

				solenoidOn(currentDisk, currentHole[currentDisk], currentVolume[currentDisk]);

        SerialUSB.print("ON  ");
        SerialUSB.print("Disk "); SerialUSB.print(currentDisk); 
        SerialUSB.print(" Hole "); SerialUSB.print(currentHole[currentDisk]); 
        SerialUSB.print(" Vol "); SerialUSB.println(currentVolume[currentDisk]); 
        SerialUSB.println("");
			}
		} // end turn on

		else { //playing a note - glissando up/down
			currentNote[currentDisk] = pitch;

      for(uint8_t d = 0; d < NUM_DISKS; d++) {
        if(d != currentDisk) {
          volatile uint8_t basePitch = getNote_HS0(currentNote[currentDisk], currentHole[currentDisk], currentDisk);
          currentNote[d] = basePitch + diskOffset[currentDisk][d]; // shouldnt be pitch -> should be the pitch (MIDI Note) of the base hole of that disk
          currentHole[d] = DEFAULT_HOLE;
        }
        float speed = setSpeed(currentNote[d], currentHole[d], d);
        SerialUSB.print("disk"); SerialUSB.print(d);
        SerialUSB.print(" speed set to "); SerialUSB.println(speed);
        SerialUSB.println("Glissando"); SerialUSB.println("");
      }

		} // end glissando
	}
	else { // note off message

		singleRegisterState = DISKSTATE_WAITING;
		solenoidOff(currentDisk, currentHole[currentDisk], currentVolume[currentDisk]);
    SerialUSB.print("OFF ");
    SerialUSB.print("Disk "); SerialUSB.print(currentDisk); 
    SerialUSB.print(" Hole "); SerialUSB.print(currentHole[currentDisk]); 
    SerialUSB.print(" Vol "); SerialUSB.println(currentVolume[currentDisk]); 
    SerialUSB.println("");

	} // end note off
} // end single register play function

void multiRegisterPlay(uint8_t pitch, uint8_t velocity) {
	//TODO
}

void handleOtherMessage(void) {
/*
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
		//pc.printf("Stop Disks\r\n");
		if(currentState != STATE_IDLE) {
			requestedState = currentState;
			currentState = STATE_IDLE;
			allSolenoidsOff();
			printf("sd");
			stopDisks();
			//pc.printf("Disks Stopped\r\n");
		}
		else {
			//pc.printf("Disks already stopped\r\n");
		}
	} // stopDisk message

	// Turn on disk
	else if(strcmp(msg->address, "/Parthenope/startDisk") == 0) {
		//pc.printf("Start Disk\r\n");
		if(currentState == STATE_IDLE) {
			currentState = requestedState;
			if(currentState == STATE_SINGLEREGISTER) {
				singleRegisterState = DISKSTATE_WAITING;
        currentDisk = 0;
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
*/
	// else do nothing
}

void swapStateWhileRunning(void) {
	//TODO
}

void testSequence(uint8_t pitch) {

  singleRegisterPlay(pitch,1);
  delay(1000);
  singleRegisterPlay(pitch,0);
  delay(1000);
  singleRegisterPlay(pitch,63);
  delay(1000);
  singleRegisterPlay(pitch,0);
  delay(1000);
  singleRegisterPlay(pitch,64);
  delay(1000);
  singleRegisterPlay(pitch,0);
  delay(1000);
  singleRegisterPlay(pitch,126);
  delay(1000);
  singleRegisterPlay(pitch,0);
  delay(1000);
  singleRegisterPlay(pitch,127);
  delay(1000);
  singleRegisterPlay(pitch,0);
  delay(1000);

}