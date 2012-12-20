/*
 * rotary.h
 *
 * Created: 18.12.2012 20:39:45
 * Author: Tristan Seifert
 */ 

#include <avr/io.h>

#ifndef ROTARY_H
#define ROTARY_H

//define port where encoder is connected
#define ROTPORT PORTA
#define ROTDDR DDRA
#define ROTPIN PINA

//define rotary encoder pins
#define ROTPA PA4
#define ROTPB PA5
#define ROTPBUTTON	PB6

//define macros to check status
#define ROTA !((1 << ROTPA) & ROTPIN)
#define ROTB !((1 << ROTPB) & ROTPIN)
#define ROTCLICK !((1 << ROTPBUTTON) & ROTPIN)

void rotary_init(void);
void rotary_checkStatus(void);

uint8_t rotary_getStatus(void);
void rotary_resetStatus(void);

#endif
