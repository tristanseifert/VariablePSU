/*
 * main.h
 *
 * Created: 18.12.2012 21:53:01
 *  Author: Tristan Seifert
 */ 

#include <stdint.h>
#include <avr/eeprom.h>

#ifndef MAIN_H_
#define MAIN_H_

#define CH1Relay PD5
#define CH2Relay PD4
#define CHRelayPort PORTD
#define CHRelayDDR DDRD

#define CH1LED PD2
#define CH2LED PD3
#define CHLEDPort PORTD
#define CHLEDDDR DDRD

#define SPI_CS_Port PORTB
#define SPI_CS_DDR DDRB
#define SPI_CS_ADC PB2
#define SPI_CS_CH1 PB3
#define SPI_CS_CH2 PB4

#define TEMP_CASE_PIN PA3

uint32_t EEMEM EEPROM_bootupCount = 0x00000000;

void InitIOPorts();
void InitUART();

#endif /* MAIN_H_ */