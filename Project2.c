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

uint32_t delayTable[4] = {500*MILLISEC, 1000*MILLISEC, 2000*MILLISEC, 3000*MILLISEC};
uint32_t offDelay = 500*MILLISEC;

enum LEDcolor{red, green, blue}color;
GPIO_Type *LEDport = R_LED_PORT;
uint32_t LEDpin = R_LED_PIN;

/*
 * @brief	Delay Loop
 * taken from "Getting Started with KL25Z on MCUXpresso IDE" Shreya Chakraborty
 * experimentally determined that entire delay loop (w/out NOP) is 10 clock cycles
 * ^^ used to set MILLISEC macro
 */
static void delay(volatile uint32_t number)
{
	while(number != 0){
		//__asm volatile("NOP");
		number--;
	}
}

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
		color = green;
		LEDport = G_LED_PORT;
		LEDpin = G_LED_PIN;
	}else if(color == green){
		color = blue;
		LEDport = B_LED_PORT;
		LEDpin = B_LED_PIN;
	}else if(color == blue){
		color = red;
		LEDport = R_LED_PORT;
		LEDpin = R_LED_PIN;
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
        	LEDport->PCOR = MASK(LEDpin);
        	delay(delayTable[i]);
        	LEDport->PSOR = MASK(LEDpin);
        	delay(offDelay);
        	change_LED_color();
        }


    }
    return 0 ;
}
