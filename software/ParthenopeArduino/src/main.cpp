#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <OSCBundle.h>
#include <ODriveArduino.h>

/* ---------------------------------------------------------------------------------------- */
//  CONSTANTS

#define VERBOSE 				        (1)

#define DISK_MOTOR_POLE_PAIRS 	(2)
#define DISK1_AXIS			      	(0)
#define DISK2_AXIS			      	(1)

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

#define PORT_IN             (8000)
#define PORT_OUT            (8001)

#define SerialPC Serial
#define SerialOD Serial1

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
void handleOtherMessage(OSCMessage msg);
void allSolenoidsOff(void);
void swapStateWhileRunning(void);

void singleRegisterPlayHelper(OSCMessage &msg);
void setModeHelper(OSCMessage &msg);
void allSolenoidsOffHelper(OSCMessage &msg);
void stopDisksHelper(OSCMessage &msg);
void startDisksHelper(OSCMessage &msg);
void setHolesHelper(OSCMessage &msg);

void test(void);
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
    {22, // vol level 0
     24,
     26,
     28},
    {30, // vol level 1
     32,
     34,
     36},
    {38, // vol level 2
     40,
     42,
     44}   
  },
  { // disk 1
    {23, // vol level 0
     25,
     27,
     29},
    {31, // vol level 1
     33,
     35,
     37},
    {39, // vol level 2
     41,
     43,
     45}
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
// <none>

// ODrive
ODriveArduino odrv0(SerialOD);

// Networking
EthernetUDP UdpR;
IPAddress IP(192, 168, 2, 11);
byte MAC[] = {0x98, 0x76, 0xB6, 0x11, 0x36, 0x47};
int size;

/* ---------------------------------------------------------------------------------------- */
//  MAIN CODE

void setup() {

  pinMode(SCL,OUTPUT);

  SerialPC.begin(9600);
  SerialOD.begin(115200);

  pinMode(0,OUTPUT);
  digitalWrite(0,LOW);
  delay(20);
  digitalWrite(0,HIGH);
  delay(20);
  digitalWrite(0,LOW);

  delay(5000);

  SerialPC.println("Starting Setup");

  digitalWrite(SCL,LOW);
  delay(20);
  digitalWrite(SCL,HIGH);
  delay(20);
  digitalWrite(SCL,LOW);

  SerialPC.println("Configuring Pins");
  for(uint8_t d = 0; d < NUM_DISKS; d++) {
    for(uint8_t v = 0; v < VOLUME_LEVELS; v++) {
      for(uint8_t h = 0; h < NUM_HOLE_SETS; h++) {
        pinMode(solenoidPins[d][v][h], OUTPUT);
      }
    }
  }

  SerialPC.println("Starting Ethernet");
  Ethernet.begin(MAC,IP);
  if(Ethernet.hardwareStatus() == EthernetNoHardware) {
    SerialPC.println("Ethernet shield was not found");
    while(true) {delay(1);}
  } 
  else if(Ethernet.hardwareStatus() != EthernetW5500) {
    SerialPC.println("Found an incompatible interface");
    while(true) {delay(1);}
    // SPI speed has been increased and will only work with the 5500
  }
  else if (Ethernet.linkStatus() == LinkOFF) {
    SerialPC.println("Ethernet cable not connected");
    while(true) {delay(1);}
  }
  if(UdpR.begin(PORT_IN) == 0) {
    SerialPC.println("Unable to open receiver port");
    while(true) {delay(1);}
  }

  SerialOD.print("r vbus_voltage\n");
  SerialPC.print("Vbus voltage: ");Serial.println(odrv0.readFloat());

  SerialPC.println("Starting Disks");
  startDisks();

}

void loop() {

  // test();

  // If there is available data on the UDP port
  if( (size = UdpR.parsePacket()) >0 ) {
    //SerialPC.print("Available Packet(s) - Size="); SerialPC.println(size);

    OSCMessage msg;
    msg.empty();
    //SerialPC.print("Did the bundle start with errors? "); SerialPC.println(bundleIn.getError());

    // Fill the OSC Bundle
    while(size--) {
      msg.fill(UdpR.read());
    }

    // If there are no errors process the OSC Bundle
    if(!msg.hasError()) {
      SerialPC.println("");
      SerialPC.println("Processing Message");
      
      switch(currentState) {
        case STATE_SINGLEREGISTER:
          // Handle Play Messages
          if( msg.dispatch("/play", singleRegisterPlayHelper, 0) ) {
            SerialPC.println("Ran: Single Register Play");
          }

          // else if( OTHER SINGLEREGISTER STATE MESSAGE )

          else { // not a state-dependent message
            handleOtherMessage(msg);
          }
          break; //STATE_SINGLEREGISTER

        case STATE_MULTIREGISTER:
          // Handle Play Messages
          if( msg.dispatch("/play", singleRegisterPlayHelper, 0) ) {
            SerialPC.println("Ran: Multi Register Play");
          }

          // else if( OTHER MULTIREGISTER STATE MESSAGE )

          else { // not a state-dependent message
            handleOtherMessage(msg);
          }
          break; //STATE_MULTIREGISTER

        case STATE_IDLE:
          // Handle setHoles messages
          if( msg.dispatch("/setHoles", setHolesHelper, 0) ) {
            SerialPC.println("Ran: Set Holes");
          }

          // else if( OTHER IDLE STATE MESSAGE )

          else { // not a state-dependent message
            handleOtherMessage(msg);
          }
          break; //STATE_IDLE

        default:
          break;
        
      } // end switch(currentState)
    } // end if(!bundleIN.hasError())
    else {
      SerialPC.println("Bundle had an error");
      SerialPC.println(msg.getError());
    }
  } //end if( (size = UdpR.parsePacket()) >0 )
  //*/
} // end loop()

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

void allSolenoidsOffHelper(OSCMessage &msg) {
	allSolenoidsOff();
}

void allSolenoidsOff() {
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
		//SerialPC.println("Setting Speed");
		odrv0.SetVelocity(diskAxis[disk], speed);
		//SerialPC.println("Done");
	}

  return speed;
}

void stopDisks(void) {
	setSpeed(0, DEFAULT_NOTE, DEFAULT_HOLE);
	setSpeed(1, DEFAULT_NOTE, DEFAULT_HOLE);
	delay(4000);
	odrv0.run_state(DISK1_AXIS, ODriveArduino::AXIS_STATE_IDLE, 0);
	odrv0.run_state(DISK2_AXIS, ODriveArduino::AXIS_STATE_IDLE, 0);
}

void startDisks(void) {

  if(requestedState == STATE_MULTIREGISTER) {
    char buf[50];
    for(uint8_t i = 0; i < NUM_DISKS; i++) {
      SerialPC.print("disk"); SerialPC.println(i);
      SerialPC.println("mode set to sensorless");
      odrv0.run_state(diskAxis[i], ODriveArduino::AXIS_STATE_SENSORLESS_CONTROL, 0);
      //odrv0.setControlMode(diskAxis[i], ODriveArduino::CTRL_MODE_VELOCITY_CONTROL, 0);
      sprintf(buf, "w axis%d.controller.config.control_mode %d\n", diskAxis[i], ODriveArduino::CTRL_MODE_VELOCITY_CONTROL);
      SerialOD.print(buf);
      delay(10000); //10000
      currentNote[i] = DEFAULT_NOTE;
      currentHole[i] = DEFAULT_HOLE;
      float speed = setSpeed(currentNote[i], currentHole[i], i);
      SerialPC.print("speed set to "); SerialPC.println(speed);
      delay(10000); //10000
    }
  }

  else if(requestedState == STATE_SINGLEREGISTER) {
    char buf[50];
    for(uint8_t i = 0; i < NUM_DISKS; i++) {
      SerialPC.print("disk"); SerialPC.println(i);
      SerialPC.println("mode set to sensorless");
      odrv0.run_state(diskAxis[i], ODriveArduino::AXIS_STATE_SENSORLESS_CONTROL, 0);
      //odrv0.setControlMode(DISK1_AXIS, ODriveArduino::CTRL_MODE_VELOCITY_CONTROL, 0);
      sprintf(buf, "w axis%d.controller.config.control_mode %d\n", diskAxis[i], ODriveArduino::CTRL_MODE_VELOCITY_CONTROL);
      SerialOD.print(buf);
      delay(10000); //10000
      currentNote[i] = DEFAULT_NOTE + diskOffset[currentDisk][i];
      currentHole[i] = DEFAULT_HOLE;
      float speed = setSpeed(currentNote[i], currentHole[i], i);
      SerialPC.print("speed set to "); SerialPC.println(speed);
      delay(10000); //10000
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

void singleRegisterPlayHelper(OSCMessage &msg) {

  uint8_t pitch = msg.getInt(0);
  uint8_t velocity = msg.getInt(1);

  singleRegisterPlay(pitch, velocity);
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
          SerialPC.print("disk"); SerialPC.print(d);
          SerialPC.print(" speed set to "); SerialPC.println(speed);
				}

				solenoidOn(currentDisk, currentHole[currentDisk], currentVolume[currentDisk]);

        SerialPC.print("ON  ");
        SerialPC.print("Disk "); SerialPC.print(currentDisk); 
        SerialPC.print(" Hole "); SerialPC.print(currentHole[currentDisk]); 
        SerialPC.print(" Vol "); SerialPC.println(currentVolume[currentDisk]); 
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
        SerialPC.print("disk"); SerialPC.print(d);
        SerialPC.print(" speed set to "); SerialPC.println(speed);
      }
      SerialPC.println("Glissando");

		} // end glissando
	}
	else { // note off message
    // only turn off if it is for the currently playing note
    if(pitch == currentNote[currentDisk]) {
      singleRegisterState = DISKSTATE_WAITING;
      solenoidOff(currentDisk, currentHole[currentDisk], currentVolume[currentDisk]);
      SerialPC.print("OFF ");
      SerialPC.print("Disk "); SerialPC.print(currentDisk); 
      SerialPC.print(" Hole "); SerialPC.print(currentHole[currentDisk]); 
      SerialPC.print(" Vol "); SerialPC.println(currentVolume[currentDisk]);
    }
    else {
      SerialPC.println("Off for different note ignored");
    }
	} // end note off
} // end single register play function

void multiRegisterPlay(uint8_t pitch, uint8_t velocity) {
	//TODO
}

void handleOtherMessage(OSCMessage msg) {

	// Turn off all notes
  if( msg.dispatch("/allNotesOff", allSolenoidsOffHelper, 0) ) {
    SerialPC.println("Ran: All Notes Off");
  }

  else if( msg.dispatch("/setMode", setModeHelper, 0) ) {
    SerialPC.println("Ran: Set Mode");
  }

  else if( msg.dispatch("/stopDisks", stopDisksHelper, 0) ) {
    SerialPC.println("Ran: Stop Disks");
  }

  else if( msg.dispatch("/startDisks", startDisksHelper, 0) ) {
    SerialPC.println("Ran: Start Disks");
  }

  else {
    SerialPC.println("Unable to process message");
  }

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

void test(void) {
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
  
  SerialPC.println("Loop Start");
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

void setModeHelper(OSCMessage &msg) {
  //pc.printf("Set Mode\r\n");
		uint8_t m = msg.getInt(0);		

		if(m == 1) requestedState = STATE_SINGLEREGISTER;
		else if(m == 2) requestedState = STATE_MULTIREGISTER;
		// else ignore

		if(currentState != STATE_IDLE) {
			currentState = requestedState;
			swapStateWhileRunning();
		}
}

void stopDisksHelper(OSCMessage &msg) {
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
}

void startDisksHelper(OSCMessage &msg) {
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

void setHolesHelper(OSCMessage &msg) {

}