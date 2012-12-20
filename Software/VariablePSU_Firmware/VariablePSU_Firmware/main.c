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
#include <avr/interrupt.h>
#include <util/delay.h>

#include <stdio.h>

#include "lcdlib.h"
#include "rotary.h"
#include "ds18x20lib.h"

/************************************************************************/
/* Global variables														*/
/************************************************************************/
unsigned char temp_case_str[8];
float temp_case;

// 128 byte read buffer
unsigned char readBuffer[128];
uint8_t curReadBuffPos;

/************************************************************************/
/* Main routine															*/
/************************************************************************/

int main(void) {
	InitIOPorts();
	
    while(1) {
		updateCaseTemp();
		printf("Case Temp: %f\n", temp_case);
		_delay_ms(100);
        //TODO:: Please write your application code 
    }
}

void InitIOPorts() {
	// Initialize the LCD IO ports
	LCD_InitIO();
	InitUART();
	
	// Initialise our rotary encoder; takes care of setting the timer.
	rotary_init();
	rotary_resetStatus();
	
	// Configure IO ports for other peripherals
	CHRelayDDR |= (1 << CH1Relay) | (1 << CH2Relay); // Relay pins as outputs
	CHLEDDDR |= (1 << CH1LED) | (1 << CH2LED); // LED pins as outputs
	SPI_CS_DDR |= (1 << SPI_CS_ADC) | (1 << SPI_CS_CH1) | (1 << SPI_CS_CH2); // SPI CS outputs
	
	DDRD |= (1 << PD6) | (1 << PD7); // FAN PWM and speaker output
	
	PORTA |= (1 << PA4) | (1 << PA5) | (1 << PA6) | (1 << PA7); // pull-ups on switch inputs
	DDRA |= (1 << PA4) | (1 << PA5) | (1 << PA6) | (1 << PA7); // switch inputs
	
	// Initialise temperature sensors
	ds1820_init(TEMP_CASE_PIN);
	
	sei(); // Enable interrupts
}	

void InitUART() {
	int baud = 19200;
	
	// Set baud rate (19200 bps at 20 MHz clock)
	UBRR0H = (unsigned char) (baud>>8);
	UBRR0L = (unsigned char) baud;
	
	// Enable receiver and transmitter
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	// Set frame format: 8data, 2stop bit
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
	
	// Enable receive interrupt
	UCSR0B |= (1 << RXCIE0);
}

/************************************************************************/
/* USART Interrupt Handler Routines	(For receive)						*/
/************************************************************************/

ISR(USART0_RX_vect, ISR_BLOCK) {
	// Read a byte from the FIFO into our buffer, increment counter.
	readBuffer[curReadBuffPos] = UDR0;
	curReadBuffPos++;
}

/************************************************************************/
/* USART Interface Routines												*/
/************************************************************************/

void USART_SendByte(char character) {
	// Wait until the USART is ready to accept more data.
	while ((UCSR0A & (1 << UDRE0)) == 0) {};

	// Send out the character
	UDR0 = character;
}

void USART_SendChars_P(char *string) {
	while(pgm_read_byte(string) != 0x00) {
		USART_SendByte(pgm_read_byte(string++));	
	}
}

void USART_SendChars(char *string) {
	while(*string != 0x00) {
		USART_SendByte(*string++);
	}
}

/************************************************************************/
/* Various utility interface routines.									*/
/************************************************************************/

void updateCaseTemp() {
	temp_case = ds1820_read_temp(TEMP_CASE_PIN);
}