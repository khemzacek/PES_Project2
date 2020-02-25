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
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL25Z4.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */
#define R_LED_PORT	PTB //Port B
#define R_LED_PIN	(18)
#define G_LED_PORT	PTB //Port B
#define G_LED_PIN	(19)
#define B_LED_PORT	PTD //Port D
#define B_LED_PIN	(1)

#define MASK(x)	(1UL << (x))

#define MILLISEC	4800

uint32_t delayTable[4] = {500, 1000, 2000, 3000};
uint32_t offDelay = 500;

enum LEDcolor{red, green, blue}color;
GPIO_Type *LEDport = B_LED_PORT;
uint32_t LEDpin = B_LED_PIN;

#define SCAN_OFFSET 544  // Offset for scan range
#define SCAN_DATA TSI0->DATA & 0xFFFF // Accessing the bits held in TSI0_DATA_TSICNT

volatile uint32_t slider = 0;

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

}

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


	// The TSI threshold isn't used is in this application
//	TSI0->TSHD = 	TSI_TSHD_THRESH(0x00) |
//								TSI_TSHD_THRESL(0x00);


}

/*
 * @brief	Function to read touch sensor from low to high capacitance for left to right
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

/*
 * @brief	Delay loop with slider polling
 * referenced "Getting Started with KL25Z on MCUXpresso IDE" Shreya Chakraborty
 * experimentally determined that entire delay loop (w/out NOP) is 10 clock cycles
 * ^^ used to set MILLISEC macro
 */
static void delay(volatile uint32_t number)
{
	volatile uint32_t sliderPoll = 0;
	int n = 100*MILLISEC;
	number = number/100;
	while(number != 0){
		while(n !=0){
			n--;
		}
		sliderPoll = Touch_Scan_LH();
		if(sliderPoll > 50){
			slider = sliderPoll;
		}
		n = 100*MILLISEC;
		number--;
	}
}

/*
 * @brief   Application entry point.
 */
int main(void) {

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();

    LED_init();
    Touch_Init();

    PRINTF("Hello World\n");

    /* Force the counter to be placed into memory. */
    volatile static int i = 0 ;
    /* Enter an infinite loop, just incrementing a counter. */
    while(1) {
        //i++ ;
        /* 'Dummy' NOP to allow source level single stepping of
            tight while() loop */
        //__asm volatile ("nop");
        //referenced https://github.com/alexander-g-dean/ESF/blob/master/Code/Chapter_2/Source/main.c

        for(i = 0; i < 4; i++){
        	LEDport->PCOR = MASK(LEDpin);	//LED on
        	delay(delayTable[i]);
        	LEDport->PSOR = MASK(LEDpin);	//LED off
        	delay(offDelay);
        	//slider = Touch_Scan_LH();
        	PRINTF("%d\n", slider);
        	if(slider > 50){
        		if(slider < 500){
        			color = red;
        		}else if(slider > 1500){
        			color = blue;
        		}else{
        			color = green;
        		}
        		change_LED_color();
        	}
        }


    }
    return 0 ;
}
