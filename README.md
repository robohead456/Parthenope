# Parthenope

Parthenope is a musical siren initially created for WPI's Music, Perception, and Robotics lab during the A18 practicum.

## Using Parthenope

1) Turn on power supply and set one output to 12V and the other to 8.5V
   The red wire going to the solenoid board should connect to the 12V supply and the corresponding black wire should connect to ground.
   The yellow wire going to the motor drivers should connect to the 8.5V supply and the corresponding black wire should connect to ground.
   
2) Turn on compressor. Set the output pressure to ~60psi. 

3) Connect the USB cable from the Nucleo board to the computer.
   The lights on the board should turn on. Wait for the motor drivers to calibrate (will make the the following pattern of beeps: beep. beep beep. beep beep beep.)
   
4) Begin spinning the disks in a clockwise direction. The current motors are not powerful enough to start spinning the stalled disks.

5) Press the blue user botton on the nucleo board to start driving the motors

6) Open the max patch (Multi Solenoid Module 12.6.0), select the right serial port, and begin writing/playing music.

NOTES:
- The higher the PSI the greater the signal-to-noise ratio, but the quicker the tank will empty and cause the compressor to run again
- At <40psi the solenoids can begin to stick and become unable to turn on/off. If notes have a high latency or don't turn off check the pressure
- Parthenope is on MIDI channel 4 of the MAX1 output from Ableton or other DAWs

## Things to work on in the future

Hardware:
- [ ] Quieter motors/drivers to eliminate high pitched whine
- [ ] More powerful motors to eliminate the hand spinning setup step
- [ ] Motor controllers that can break to slow disk - for use in disk speed controll
- [ ] Change to a larger Nucleo board with Ethernet
- [ ] More disks for a larger range

Software:
- [ ] Send disk speed in rps from max to the nucleo (MIDI pitch bend?)
- [ ] Ethernet control
