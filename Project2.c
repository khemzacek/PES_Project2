/* PES Project 2
 *@file	Project2.c
 *@brief	LEDs blink in predefined pattern and change color with slider touches
 *
 *This program blinks an onboard tricolor LED on and off in a predefined timing
 *pattern. The program runs through 10 cycles of this pattern and then ends. The LEDs
 *will change color based on input from the capacitive touch sensor (left = red,
 *middle = green, left = blue). The program can also be configured to run on a PC with
 *the altered behavior of a print statement in place of the LED turning on or off (i.e.
 *"B LED ON" instead of the blue LED turning on) and the color changing every 3 activations
 *rather than in response to the touch slider. The program can also be configured to run in
 *DEBUG mode on either the board or the PC. Debug mode outputs extra print statements which
 *give extra information about program activity (program start/end, cycle count, slider value
 *or color change, timer start).
 *
 *Target board: Freedom Freescale KL25Z
 *
 *@author Katherine Hemzacek
 */

/*
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    Project2.c
 * @brief   Application entry point.
 */


/* DEFINE TARGET */
#define FB_RUN

#ifdef	FB_RUN
#define MY_FRDM_BOARD
#endif	/* FB_RUN */

#ifdef	FB_DEBUG
#define MY_FRDM_BOARD
#define MY_DEBUG
#endif	/* FB_DEBUG */

#ifdef	PC_RUN
#define MY_PC
#endif	/* PC_RUN */

#ifdef	PC_DEBUG
#define MY_PC
#define MY_DEBUG
#endif	/* PC_DEBUG */


/* INCLUDES */
#include <stdio.h>
#ifdef MY_FRDM_BOARD
	#include "board.h"
	#include "peripherals.h"
	#include "pin_mux.h"
	#include "clock_config.h"
	#include "MKL25Z4.h"
	#include "fsl_debug_console.h"
#endif /* MY_FRDM_BOARD */


/* DEFINITIONS AND DECLARATIONS */
#ifdef MY_FRDM_BOARD
	// LED Peripheral
	// referenced https://github.com/alexander-g-dean/ESF/blob/master/Code/Chapter_2/Source/main.c
	#define R_LED_PORT	PTB //Port B
	#define R_LED_PIN	(18)
	#define G_LED_PORT	PTB //Port B
	#define G_LED_PIN	(19)
	#define B_LED_PORT	PTD //Port D
	#define B_LED_PIN	(1)
	GPIO_Type *LEDport = B_LED_PORT;
	uint32_t LEDpin = B_LED_PIN;
	#define MASK(x)	(1UL << (x))

	// Slider Peripheral
	// referenced https://www.digikey.com/eewiki/display/microcontroller/Using+the+Capacitive+Touch+Sensor+on+the+FRDM-KL46Z
	#define SCAN_OFFSET 544  // Offset for scan range
	#define SCAN_DATA TSI0->DATA & 0xFFFF // Accessing the bits held in TSI0_DATA_TSICNT
	volatile uint32_t slider = 0;

	// Processor Timing
	#define MILLISEC	4800
#endif	/* MY_FRDM_BOARD */

#ifdef MY_PC
	// Typedefs
	typedef unsigned int uint32_t;
	typedef unsigned char uint8_t;

	// Print macro
	#define	PRINTF	printf

	// Color Change Tracking
	uint8_t activCNT = 0;

	// Processor Timing
	#define MILLISEC	430000
#endif	/* MY_PC */

/* All Platforms */
// Blink Pattern
#define NUM_CYCLES	10
uint32_t onDelay[4] = {500, 1000, 2000, 3000};
uint32_t offDelay = 500;

// Colors
char colorTXT[3] = {'R', 'G', 'B'};
enum LEDcolor{red, green, blue};
enum LEDcolor color = blue;


/* FUNCTION DEFINITIONS */
#ifdef	MY_FRDM_BOARD
/*
 * @brief	Initialize LED pins
 * taken from https://github.com/alexander-g-dean/ESF/blob/master/Code/Chapter_2/Source/main.c
 */
void LED_init(void)
{
	//Enable clock to Port B and Port D
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTD_MASK;

	//Make 3 pins (to RGB LEDs) GPIO
	PORTB->PCR[R_LED_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[R_LED_PIN] |= PORT_PCR_MUX(1);
	PORTB->PCR[G_LED_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[G_LED_PIN] |= PORT_PCR_MUX(1);
	PORTD->PCR[B_LED_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[B_LED_PIN] |= PORT_PCR_MUX(1);

	//Set pins to outputs
	PTB->PDDR |= MASK(R_LED_PIN) | MASK(G_LED_PIN);
	PTD->PDDR |= MASK(B_LED_PIN);

	PTB->PCOR |= MASK(R_LED_PIN) | MASK(G_LED_PIN);
	PTD->PCOR |= MASK(B_LED_PIN);

	//Turn all off to start
	R_LED_PORT->PSOR = MASK(R_LED_PIN);
	G_LED_PORT->PSOR = MASK(G_LED_PIN);
	B_LED_PORT->PSOR = MASK(B_LED_PIN);

	#ifdef	MY_DEBUG
		PRINTF("LEDS INITIALIZED\n");
	#endif	/* MY_DEBUG */

}
#endif	/* MY_FRDM_BOARD */

#ifdef	MY_FRDM_BOARD
/*
 * @brief	Changes LED color
 * changes peripheral ports and pins to light the right color LED
 */
void change_LED_color()
{
	if(color == red){
		LEDport = R_LED_PORT;
		LEDpin = R_LED_PIN;
	}else if(color == green){
		LEDport = G_LED_PORT;
		LEDpin = G_LED_PIN;
	}else if(color == blue){
		LEDport = B_LED_PORT;
		LEDpin = B_LED_PIN;
	}
}
#endif	/* MY_FRDM_BOARD */

/*
 * @brief	Turns on LED
 * on board, clears gpio pin to turn on light
 * on PC, prints *color* LED ON message
 * //referenced https://github.com/alexander-g-dean/ESF/blob/master/Code/Chapter_2/Source/main.c
 */
void LED_on()
{
	#ifdef	MY_FRDM_BOARD
	LEDport->PCOR = MASK(LEDpin);
	#ifdef	MY_DEBUG
		PRINTF("%c LED ON\n", colorTXT[color]);
	#endif	/* MY_DEBUG */
	#endif	/* MY_FRDM_BOARD */

	#ifdef	MY_PC
		PRINTF("%c LED ON\n", colorTXT[color]);
	#endif	/* MY_PC */
}

/*
 * @brief	Turns off LED
 * on board, sets gpio pin to turn off light
 * on PC, prints *color* LED OFF message
 * //referenced https://github.com/alexander-g-dean/ESF/blob/master/Code/Chapter_2/Source/main.c
 */
void LED_off()
{
	#ifdef	MY_FRDM_BOARD
	LEDport->PSOR = MASK(LEDpin);
	#ifdef	MY_DEBUG
   		PRINTF("%c LED OFF\n", colorTXT[color]);
	#endif	/* MY_DEBUG */
	#endif	/* MY_FRDM_BOARD */

	#ifdef	MY_PC
		PRINTF("%c LED OFF\n", colorTXT[color]);
	#endif	/* MY_PC */
}

#ifdef	MY_FRDM_BOARD
/*
 * @brief	TSI initialization function
 * taken from https://www.digikey.com/eewiki/display/microcontroller/Using+the+Capacitive+Touch+Sensor+on+the+FRDM-KL46Z
 */
void Touch_Init()
{
	// Enable clock for TSI PortB 16 and 17
	SIM->SCGC5 |= SIM_SCGC5_TSI_MASK;


	TSI0->GENCS = TSI_GENCS_OUTRGF_MASK |  // Out of range flag, set to 1 to clear
								//TSI_GENCS_ESOR_MASK |  // This is disabled to give an interrupt when out of range.  Enable to give an interrupt when end of scan
								TSI_GENCS_MODE(0u) |  // Set at 0 for capacitive sensing.  Other settings are 4 and 8 for threshold detection, and 12 for noise detection
								TSI_GENCS_REFCHRG(0u) | // 0-7 for Reference charge
								TSI_GENCS_DVOLT(0u) | // 0-3 sets the Voltage range
								TSI_GENCS_EXTCHRG(0u) | //0-7 for External charge
								TSI_GENCS_PS(0u) | // 0-7 for electrode prescaler
								TSI_GENCS_NSCN(31u) | // 0-31 + 1 for number of scans per electrode
								TSI_GENCS_TSIEN_MASK | // TSI enable bit
								//TSI_GENCS_TSIIEN_MASK | //TSI interrupt is disables
								TSI_GENCS_STPE_MASK | // Enables TSI in low power mode
								//TSI_GENCS_STM_MASK | // 0 for software trigger, 1 for hardware trigger
								//TSI_GENCS_SCNIP_MASK | // scan in progress flag
								TSI_GENCS_EOSF_MASK ; // End of scan flag, set to 1 to clear
								//TSI_GENCS_CURSW_MASK; // Do not swap current sources

	#ifdef	MY_DEBUG
    	PRINTF("SLIDER INITIALIZED\n");
	#endif	/* MY_DEBUG */

}
#endif	/* MY_FRDM_BOARD */

#ifdef	MY_FRDM_BOARD
/*
 * @brief	Function to read touch sensor
 * low to high capacitance from left to right
 * taken from https://www.digikey.com/eewiki/display/microcontroller/Using+the+Capacitive+Touch+Sensor+on+the+FRDM-KL46Z
 */
uint32_t  Touch_Scan_LH(void)
{
	int scan;
	TSI0->DATA = 	TSI_DATA_TSICH(10u); // Using channel 10 of The TSI
	TSI0->DATA |= TSI_DATA_SWTS_MASK; // Software trigger for scan
	scan = SCAN_DATA;
	TSI0->GENCS |= TSI_GENCS_EOSF_MASK ; // Reset end of scan flag

	return scan - SCAN_OFFSET;
}
#endif	/* MY_FRDM_BOARD */

/*
 * @brief	Checks if color should be changed for the next LED_on cycle
 * on board, checks if/where slider touched
 * on PC, checks if 3 activations have occurred
 */
void check_color_change()
{
	#ifdef	MY_FRDM_BOARD
	if(slider > 50){	// untouched baseline <50 expected
		if(slider < 500){	// left touch
			color = red;
    	}else if(slider > 1500){	//right touch
	        color = blue;
	    }else{				// middle touch
	        color = green;
	    }
	    change_LED_color();	// change LED based on slider touch
	}
	#endif	/* MY_FRDM_BOARD */

	#ifdef	MY_PC
	if(activCNT == 3){
		if(color == red){
			color = blue;
		}else if(color == blue){
			color = green;
		}else if(color == green){
			color = red;
		}
		#ifdef	MY_DEBUG
			PRINTF("COLOR CHANGE\n");
		#endif	/* MY_DEBUG */
		activCNT = 0;
	}
	#endif	/* MY_PC */
}


/*
 * @brief	Busy-Wait with delay and slider polling
 * referenced "Getting Started with KL25Z on MCUXpresso IDE" Shreya Chakraborty
 * experimentally determined that inner while loop is 10 clock cycles
 * ^^ used to set MILLISEC macro
 */
static void delay_poll(volatile uint32_t number)
{
	#ifdef	MY_DEBUG
    	PRINTF("START %d MILLISECOND DELAY\n", number);
	#endif	/* MY_DEBUG */

	int n = 100*MILLISEC;
	number = number/100;
	#ifdef	MY_FRDM_BOARD
	volatile uint32_t sliderPoll = 0;
	#endif	/* MY_FRDM_BOARD */

	//full delay-polling loop
	while(number != 0){
		//100ms delay loop
		while(n !=0){
			n--;
		}
	#ifdef	MY_FRDM_BOARD
		//poll slider and check if touched
		sliderPoll = Touch_Scan_LH();
		if(sliderPoll == -544) sliderPoll = 0;	//before slider has been touched at all
												//scan reads -544, related to offset
		if(sliderPoll > 50){	// untouched baseline of <50 expected
			if(slider != sliderPoll){	//new slider value
				slider = sliderPoll;
				#ifdef	MY_DEBUG
					PRINTF("SLIDER VALUE %d\n", slider);
				#endif	/* MY_DEBUG */
			}
		}
	#endif	/* MY_FRDM_BOARD */
		n = 100*MILLISEC;
		number--;
	}
}


/* MAIN FUNCTION */
/*
 * @brief   Main application entry point.
 */
int main(void) {
#ifdef	MY_FRDM_BOARD
  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();

	#ifdef	MY_DEBUG
    	PRINTF("BOARD INITIALIZED\n");
	#endif	/* MY_DEBUG */

    /* Init peripherals */
    LED_init();
    Touch_Init();
#endif	/* MY_FRDM_BOARD */

    /* Init loop counters */
    uint32_t cycleCNT = NUM_CYCLES;
    volatile static int i = 0 ;

	#ifdef	MY_PC
	#ifdef	MY_DEBUG
    	PRINTF("START PROGRAM\n");
	#endif	/* MY_DEBUG */
    #endif	/* MY_PC */

    /* Main Loop */
    while(cycleCNT != 0) {
		#ifdef	MY_DEBUG
    		PRINTF("PATTERN CYCLE %d\n", (NUM_CYCLES-cycleCNT+1));
		#endif	/* MY_DEBUG */

        for(i = 0; i < 4; i++){
        	LED_on();
			#ifdef	MY_PC
        		activCNT++;
			#endif	/* MY_PC */
        	delay_poll(onDelay[i]);
        	LED_off();
        	delay_poll(offDelay);
        	check_color_change();
        }
        cycleCNT--;
    }
	#ifdef	MY_DEBUG
		PRINTF("END PROGRAM\n");
	#endif	/* MY_DEBUG */
    return 0 ;
}
