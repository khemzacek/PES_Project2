# PES_Project2
### Katherine Hemzacek

## Project Comments
This program blinks an onboard tricolor LED on and off in a predefined timing pattern. The program runs through 10 cycles of this pattern and then ends. The LEDs will change color based on input from the capacitive touch sensor (left = red, middle = green, left = blue).

The program can also be configured to run on a PC with the altered behavior of a print statement in place of the LED turning on or off (i.e. "B LED ON" instead of the blue LED turning on) and the color changing every 3 activations rather than in response to the touch sensor.

The program can also be configured to run in DEBUG mode on either the board or the PC. Debug mode outputs extra print statements which give extra information about program activity (program start/end, cycle count, slider value or color change, timer start).

## Repo Contents
- README.md
- Project2.c
- Work Breakdown Structure for project


## Installation Execution Notes
The target board for this program is the Freedom Freescale KL25Z.

The program was created in the MCUXpresso IDE. Board-appropriate startup code and include files were created by the IDE.

The PC versions of the program were compiled and run using gcc in Cygwin.

The target (FB_RUN, FB_DEBUG, PC_RUN, PC_DEBUG) can be changed by changing the first define statement under the heading /* DEFINE TARGET */.
