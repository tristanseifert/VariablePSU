/*
 * LCDLib.h
 *
 * Created: 18.12.2012 20:39:45
 *  Author: Tristan Seifert
 */ 

#include <avr/io.h>

#ifndef LCDLIB_H_
#define LCDLIB_H_

#define LCD_PORT PORTA // Port that the LCD is connected to
#define LCD_STB 0x02 // Bit that the STB pin is on
#define LCD_CLK 0x01 // Bit that the CLK pin is on
#define LCD_SIO 0x00 // Bit that the SIO pin is on

#define LCD_STB_BITMASK (0x01 << LCD_STB) // Bitmask to use in bitwise ops for STB
#define LCD_CLK_BITMASK (0x01 << LCD_CLK) // Bitmask to use in bitwise ops for CLK
#define LCD_SIO_BITMASK (0x01 << LCD_SIO) // Bitmask to use in bitwise ops for SIO

void LCD_InitIO();
void LCD_Init(uint8_t brightness);
void LCD_ResetLCD();

void LCD_PutText(char *text);
void LCD_PutText_P(char *text); // NOTE: Uses PROGMEM
void LCD_PutChar(uint8_t x, uint8_t y, char character);
void LCD_PutChar_P(uint8_t x, uint8_t y, char character); // NOTE: Uses PROGMEM
void LCD_ClearScreen();

void LCD_SetCGRAM(uint8_t character, char *data); // NOTE: Uses PROGMEM

void LCD_CursorHome();
void LCD_SetCursorPos(uint8_t x, uint8_t y);
void LCD_SetDisplayState(uint8_t dispOn, uint8_t cursorOn, uint8_t cursorBlink);

// Only to be called by the functions above, not by itself.
static void LCD_WriteByteToLCD(uint8_t byte, uint8_t flags);

#endif /* LCDLIB_H_ */