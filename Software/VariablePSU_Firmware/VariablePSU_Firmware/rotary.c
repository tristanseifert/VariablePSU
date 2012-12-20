/*
 * rotary.x
 *
 * Created: 18.12.2012 20:39:45
 * Author: Tristan Seifert
 */ 

#include <avr/interrupt.h>
#include <util/delay.h>

#include "rotary.h"

static uint8_t rotarystatus = 0;
static uint8_t wait = 0;

/************************************************************************/
/* Initialise IO ports and timer.										*/
/************************************************************************/
void rotary_init(void) {
	//set pins as input
	ROTDDR &= ~((1<<ROTPA)|(1<<ROTPB)|(1<<ROTPBUTTON));
	//enable interrnal pullups;
	ROTPORT |= (1<<ROTPA)|(1<<ROTPB)|(1<<ROTPBUTTON);
	
	// Set up our timer.
	TCCR0A = 0x00; // normal port operation, no outputs to pins.
	TCCR0B |= (1<<CS22) | (1<<CS20); //prescaler of 1024
	OCR0A = 0xC3; // Interrupt frequency of 100Hz at 20 MHz CPU clock
	
	TIMSK0 |= (1 << TOIE0); //Enable Timer0 Overflow interrupts
}

/************************************************************************/
/* Check rotary encoder status.											*/
/************************************************************************/
void rotary_checkStatus(void) {
//check if rotation is left
 	if(ROTA & (!wait)) {
		wait = 1;
	} 
	
	if (ROTB & ROTA & (wait)) {
		rotarystatus = 2;
		wait = 2;
	} else if(ROTA & (!ROTB) & wait) {
		rotarystatus = 1;
		wait = 2;	
	}
	
	if ((!ROTA) & !(ROTB) & (wait==2)) {
		wait=0;
	}
	
	//check button status
	if (ROTCLICK) {
		for(volatile uint16_t x=0; x < 0xFFF; x++) {
			
		}
		
		if (ROTCLICK) {
			rotarystatus = 4;
		}
	}		
}

/************************************************************************/
/* Get rotary encoder status.											*/
/*																		*/
/* 1 = turned left, 2 = turned right, 4 = centre button.				*/
/************************************************************************/
uint8_t rotary_getStatus(void) {
	return rotarystatus;
}

/************************************************************************/
/* Reset the rotary encoder status.										*/
/************************************************************************/
void rotary_resetStatus(void) {
	rotarystatus=0;
}

/************************************************************************/
/* ISR for rotary encoder reads											*/
/************************************************************************/
ISR(TIMER0_OVF_vect, ISR_BLOCK) {
	// Check rotary encoder status.
	rotary_checkStatus();
}