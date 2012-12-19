/*
 * VariablePSU_Firmware.c
 *
 * Created: 18.12.2012 20:38:38
 *  Author: Tristan Seifert
 */ 

#define F_CPU 20000000UL

#include "main.h"

#include <avr/io.h>
#include <avr/pgmspace.h>

#include "lcdlib.h"

int main(void) {
	InitIOPorts();
	
    while(1) {
        //TODO:: Please write your application code 
    }
}

void InitIOPorts() {
	// Initialise the LCD IO ports
	LCD_InitIO();
}	