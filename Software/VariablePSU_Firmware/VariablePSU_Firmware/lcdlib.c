/*
 * LCDLib.c
 *
 * Created: 18.12.2012 20:39:31
 *  Author: Tristan Seifert
 */ 

#include <avr/interrupt.h>
#include <avr/pgmspace.h>

// Needs F_CPU to be defined somewhere
#include <util/delay.h>

#include "lcdlib.h"

/************************************************************************/
/* Writes a single byte out to the LCD over the serial link with flags.	*/
/* Takes care of all the SCK generation automagically.					*/
/*																		*/
/* Note that this destroys the input you give it.						*/
/* Also disables interrupts so it doesn't screw up; too lazy to find a	*/
/* cleaner way of preventing screw-ups/race conditions/					*/
/************************************************************************/
void LCD_WriteByteToLCD(uint8_t byte, uint8_t flags) {
	cli(); // Disable interrupts
	
	LCD_PORT |= LCD_CLK_BITMASK; // Make sure CLK is high
	LCD_PORT &= ~LCD_STB_BITMASK; // Set LCD enable pin to 0.
	
	// Loop to write the flag
	for (uint8_t i = 0; i < 8; i++) {
		_delay_us(100); // Wait 100 microseconds for the VFD.
		LCD_PORT ^= ((flags & 0x01) << LCD_SIO) + LCD_CLK_BITMASK; // Shift out a single bit, toggle the state of clk as well. (it's low now)
		_delay_us(100); // Wait 100 microseconds for the VFD.
		LCD_PORT ^= LCD_CLK_BITMASK; // Toggle clk again (It's high again)	
		flags = flags >> 0x01; // Shift byte to the right 1 bit.
	}
	
	// Loop to transfer the bytes
	for(uint8_t i = 0; i < 7; i++) {
		_delay_us(100); // Wait 100 microseconds for the VFD.
		LCD_PORT ^= ((byte & 0x01) << LCD_SIO) + LCD_CLK_BITMASK; // Shift out a single bit, toggle the state of clk as well. (it's low now)
		_delay_us(100); // Wait 100 microseconds for the VFD.
		LCD_PORT ^= LCD_CLK_BITMASK; // Toggle clk again (It's high again)
		byte = byte >> 0x01; // Shift byte to the right 1 bit.		
	}
	
	LCD_PORT ^= (LCD_STB_BITMASK); // Make STB high again
	
	sei(); // Re-enable interrupts
}


/************************************************************************/
/* Initialises IO ports for the VFD										*/
/************************************************************************/
void LCD_InitIO() {
	
}

/************************************************************************/
/* Initialises the VFD with the specified brightness.					*/
/************************************************************************/
void LCD_Init(uint8_t brightness) {
	
}

/************************************************************************/
/* Resets the VFD's controller.											*/
/************************************************************************/
void LCD_ResetLCD() {
	
}

/************************************************************************/
/* Clears the entire screen and resets cursor to (0, 0).				*/
/************************************************************************/
void LCD_ClearScreen() {
	LCD_CursorHome(); // reset cursor to (0, 0).
}


/************************************************************************/
/* Puts text from RAM to the current cursor position.					*/
/************************************************************************/
void LCD_PutText(char *text) {
	
}

/************************************************************************/
/* Puts text from PROGMEM to the current cursor position.				*/
/************************************************************************/
void LCD_PutText_P(char *text) {
	
}


/************************************************************************/
/* Puts a character from RAM to a specific on-screen position.			*/
/************************************************************************/
void LCD_PutChar(uint8_t x, uint8_t y, char character) {
	
}

/************************************************************************/
/* Puts a character from PROGMEM to a specific on-screen position.		*/
/************************************************************************/
void LCD_PutChar_P(uint8_t x, uint8_t y, char character) {
	
}	


/************************************************************************/
/* Sets a specific custom character in CG-RAM from PROGMEM.				*/
/************************************************************************/
void LCD_SetCGRAM(uint8_t character, char *data) {
	
}


/************************************************************************/
/* Resets the cursor to the home position, (0,0).						*/
/************************************************************************/
void LCD_CursorHome() {
	
}

/************************************************************************/
/* Sets the cursor's position to the specified X and Y coordinates.		*/
/************************************************************************/
void LCD_SetCursorPos(uint8_t x, uint8_t y) {
	
}

/************************************************************************/
/* Sets the LCD's internal state, such as disable/enable display, cursor*/
/* display, and the cursor blinking.									*/
/************************************************************************/
void LCD_SetDisplayState(uint8_t dispOn, uint8_t cursorOn, uint8_t cursorBlink) {
	
}