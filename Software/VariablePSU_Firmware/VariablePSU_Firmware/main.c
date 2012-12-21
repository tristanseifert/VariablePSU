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
unsigned char UART0_readBuffer[128];
uint8_t UART0_curReadBuffPos;

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
	InitSPI();
	InitInterrupts();
	InitPWMTimer();
	
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
	
	DDRB &= ~(1 << PB1); // Make sure PB1 (!ADC_RDY!) is an input.
	// Initialise temperature sensors
	ds1820_init(TEMP_CASE_PIN);
	
	sei(); // Enable interrupts
}	

// Initialise UART0.
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

// Initialise SPI in master mode.
void InitSPI() {
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0); // Enable SPI with SCK = FCK/16
	DDRB |= (1 << PB7) | (1 << PB5); // SCK and MOSI as outputs
}

// Initialise external ADC interrupt.
void InitInterrupts() {
	PCMSK1 |= (1 << PINB1); // Enable pin B1 on-change interrupt
	PCICR |= 0x02; // Enable interrupts for pins 9-16.
}

// Initialise Timer2 for PWM of speaker and fan.
void InitPWMTimer() {
/*	// Set OC2B when up counting, toggle OC1B when TOP is reached.
	TCCR2A = (1 << COM2A0) | (1 << COM2B0) | (1 << COM2B1) | (1 << WGM20);
	// Prescaler Clk/256
	TCCR2B = (1 << CS22) | (1 << CS21);*/
}

/************************************************************************/
/* Voltage/current measurement stuff									*/
/************************************************************************/

// Called when there is a state change in the !ADC_RDY! input.
ISR(PCINT1_vect, ISR_BLOCK) {
	// We have data ready.
	if(PORTB & (1 << PINB1) == 0) {
		
	} else {
		// The input transitioned back to high -- ignore this.
	}
}

/************************************************************************/
/* USART Interrupt Handler Routines	(For receive)						*/
/************************************************************************/

ISR(USART0_RX_vect, ISR_BLOCK) {
	// Read a byte from the FIFO into our buffer, increment counter.
	UART0_readBuffer[UART0_curReadBuffPos] = UDR0;
	UART0_curReadBuffPos++;
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
/* SPI Interface Routines												*/
/************************************************************************/

char SPI_ReceiveChar(void) {
	// Wait for reception complete
	while(!(SPSR & (1<<SPIF)));
	// Return Data Register
	return SPDR;
}

void SPI_SendChar(char cData) {
	// Start transmission
	SPDR = cData;
	// Wait for the transmission to complete
	while(! (SPSR & (1 << SPIF)));
}

void SPI_SendChars(char *string) {
	while(*string != 0x00) {
		SPI_SendChar(string++);
	}
}

void SPI_SendChars_P(char *string) {
	while(pgm_read_byte(string) != 0x00) {
		SPI_SendChar(pgm_read_byte(string++));
	}
}

/************************************************************************/
/* Various utility interface routines.									*/
/************************************************************************/

void updateCaseTemp() {
	temp_case = ds1820_read_temp(TEMP_CASE_PIN);
}