/* Michael Sidler - A18 Musical Robotics Practicum
 * 
 * This code controls the disks on parthenope from serial data recieved
 * 
 * This code runs on an STM32 Nucleo F401RE board
 * 
 */

#include <mbed.h>
#include "clock.h"

/////////////////////////////////////////////////////////
//   DISK SPEED CONSTANTS

// P controller constants per disk
#define DISK1_KP (0.000038)
#define DISK2_KP (0.000038)
#define DISK3_KP (0.000038)

// Maximum pulse width offset per disk
#define DISK1_MAXOFFSET (0.001)
#define DISK2_MAXOFFSET (0.001)
#define DISK3_MAXOFFSET (0.001)

/////////////////////////////////////////////////////////
//   PIN ASSIGNMENTS

#define DISK1_ENCODER_PIN (PB_3)
#define DISK2_ENCODER_PIN (PB_5)
#define DISK3_ENCODER_PIN (PB_4)

#define DISK1_MOTOR_PIN (PB_15)
#define DISK2_MOTOR_PIN (PB_14)
#define DISK3_MOTOR_PIN (PB_13)

#define NUCLEO_BUTTON_PIN (PC_13)

#define DEBUG_SERIAL_TX (PA_11)
#define DEBUG_SERIAL_RX (PA_12)

#define solenoid_d0n4 (PA_0)
#define solenoid_d0n3 (PA_1)
#define solenoid_d0n2 (PA_4)
#define solenoid_d0n1 (PB_0)
#define solenoid_d0n0 (PC_1)

#define solenoid_d1n3 (PA_5)
#define solenoid_d1n2 (PA_6)
#define solenoid_d1n1 (PA_7)
#define solenoid_d1n0 (PB_6)

#define solenoid_d2n4 (PC_7)
#define solenoid_d2n3 (PA_9)
#define solenoid_d2n2 (PA_8)
#define solenoid_d2n1 (PB_10)
#define solenoid_d2n0 (PC_0)

/////////////////////////////////////////////////////////
//   I/O CONFIGURATION

// PWM motor controllers
PwmOut disk1(DISK1_MOTOR_PIN);
PwmOut disk2(DISK2_MOTOR_PIN);
PwmOut disk3(DISK3_MOTOR_PIN);

// Interrupts for motor encoders 
InterruptIn disk1encoder(DISK1_ENCODER_PIN);
InterruptIn disk2encoder(DISK2_ENCODER_PIN);
InterruptIn disk3encoder(DISK3_ENCODER_PIN);

// Interrupt for start/stop button
InterruptIn button(NUCLEO_BUTTON_PIN);

// Serial connection to PC
Serial serial(USBTX, USBRX);

/////////////////////////////////////////////////////////
//   GLOBAL VARIABLES

bool run = 0;   //should disks be spinning

// Period in ms for the disks. Ideally would be 100, but the controller is constantly
//  ~3ms above the setpoint (1rps @ 103ms). In the future if we have controllers that 
//  can break the disks to slow them we can implement a full PID controller and not
//  have the manual offset.
uint16_t disk1_period = 97;
uint16_t disk2_period = 97;
uint16_t disk3_period = 97;

// Time in ms that the disk passed the encoder
volatile uint64_t thisCount_d1, lastCount_d1;
volatile uint64_t thisCount_d2, lastCount_d2;
volatile uint64_t thisCount_d3, lastCount_d3;

// Temporary variables for calculating output to disk
int64_t diff_d1, diff_d2, diff_d3;
float offset_d1, offset_d2, offset_d3;

// Array for holding note velocities
uint8_t velocities[14] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};

// Array for holding solenoid pins
DigitalOut solenoidPins[14] = {solenoid_d0n0,
                               solenoid_d0n1,
                               solenoid_d0n2,
                               solenoid_d0n3,
                               solenoid_d0n4,
                               solenoid_d1n0,
                               solenoid_d1n1,
                               solenoid_d1n2,
                               solenoid_d1n3,
                               solenoid_d2n0,
                               solenoid_d2n1,
                               solenoid_d2n2,
                               solenoid_d2n3,
                               solenoid_d2n4};

// Buffer to hold incoming serial data
char buffer[12];

// Bool is true if there is new data 
bool newData = 0;

// temporary variables to hold data from the buffer
uint8_t disk = 0;
uint8_t note = 0;
uint8_t velocity = 0;

/////////////////////////////////////////////////////////
//   INTERRUPT SERVICE ROUTINES

void disable_motors() {
    run = !run;
}

void encoder_d1() {
    lastCount_d1 = thisCount_d1;
    thisCount_d1 = clock_ms();
}

void encoder_d2() {
    lastCount_d2 = thisCount_d2;
    thisCount_d2 = clock_ms();
}

void encoder_d3() {
    lastCount_d3 = thisCount_d3;
    thisCount_d3 = clock_ms();
}

/////////////////////////////////////////////////////////
//   MAIN CODE

int main() {
    //  SETUP SERIAL
    serial.baud(115200);
    serial.set_blocking(0);

    //  ATTACH ISRs
    button.rise(&disable_motors);
    disk1encoder.rise(&encoder_d1);
    disk2encoder.rise(&encoder_d2);
    disk3encoder.rise(&encoder_d3);

    //  SET SERVO TIMINGS
    disk1.period(0.020);
    disk2.period(0.020);
    disk3.period(0.020);

    //  MAIN LOOP
    while(1) {

        //MIDI PITCH CONTROL
        if(serial.readable()) {
            serial.gets(buffer,10);
            if(buffer[0] == 'd' && buffer[2] == 'n') {
                disk = buffer[1] - 48;
                note = buffer[3] - 48;
                velocity = ((100*(buffer[5]-48)) + (10*(buffer[6]-48)) + (buffer[7]-48));

                switch(disk) {
                    case 0:
                        switch(note) {
                            case 0:
                                velocities[0] = velocity;
                                break;
                            case 1:
                                velocities[1] = velocity;
                                break;
                            case 2:
                                velocities[2] = velocity;
                                break;
                            case 3:
                                velocities[3] = velocity;
                                break;
                            case 4:
                                velocities[4] = velocity;
                                break;
                            default:
                                break;
                        }
                        break;
                    case 1:
                        switch(note) {
                            case 0:
                                velocities[5] = velocity;
                                break;
                            case 1:
                                velocities[6] = velocity;
                                break;
                            case 2:
                                velocities[7] = velocity;
                                break;
                            case 3:
                                velocities[8] = velocity;
                                break;
                            default:
                                break;
                        }
                        break;
                    case 2:
                        switch(note) {
                            case 0:
                                velocities[9] = velocity;
                                break;
                            case 1:
                                velocities[10] = velocity;
                                break;
                            case 2:
                                velocities[11] = velocity;
                                break;
                            case 3:
                                velocities[12] = velocity;
                                break;
                            case 4:
                                velocities[13] = velocity;
                                break;
                            default:
                                break;
                        }
                        break;
                    default:
                        break;
                }
                
                newData = 0;

                for(int n = 0; n < 14; n++){
                    if(velocities[n] > 0)
                        solenoidPins[n] = 1;
                    else
                        solenoidPins[n] = 0;
                }
            } 
        }
        
        //DISK VELOCITY CONTROL
        if(run) {
            //DISK 1 VELOCITY CONTROL
            diff_d1 = ((thisCount_d1 - lastCount_d1) - disk1_period);
            //serial.printf("Disk1 Error: %4d", diff_d1);
            offset_d1 = diff_d1 * DISK1_KP;
            if(offset_d1 > DISK1_MAXOFFSET)
                offset_d1 = DISK1_MAXOFFSET;
            if(offset_d1 < -DISK1_MAXOFFSET)
                offset_d1 = -DISK1_MAXOFFSET;
            disk1.pulsewidth(0.001 + offset_d1);

            //DISK 2 VELOCITY CONTROL
            diff_d2 = ((thisCount_d2 - lastCount_d2) - disk2_period);
            //serial.printf("   Disk2 Error: %4d", diff_d2);
            offset_d2 = diff_d2 * DISK2_KP;
            if(offset_d2 > DISK2_MAXOFFSET)
                offset_d2 = DISK2_MAXOFFSET;
            if(offset_d2 < -DISK2_MAXOFFSET)
                offset_d2 = -DISK2_MAXOFFSET;
            disk2.pulsewidth(0.001 + offset_d2);

            //DISK 3 VELOCITY CONTROL
            diff_d3 = ((thisCount_d3 - lastCount_d3) - disk3_period);
            //serial.printf("   Disk3 Error: %4d\n\r", diff_d3);
            offset_d3 = diff_d3 * DISK3_KP;
            if(offset_d3 > DISK3_MAXOFFSET)
                offset_d3 = DISK3_MAXOFFSET;
            if(offset_d3 < -DISK3_MAXOFFSET)
                offset_d3 = -DISK3_MAXOFFSET;
            disk3.pulsewidth(0.001 + offset_d3);
        }
        else {
            //SET DISK VEOLCITIES TO 0
            disk1.pulsewidth(0.001);
            disk2.pulsewidth(0.001);
            disk3.pulsewidth(0.001);
        }
    }
}