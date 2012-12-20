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
	
	flags |= 0xF8; // First 5 bits are 1's
	
	// Loop to write the flag
	for (volatile uint8_t i = 0; i < 8; i++) {
		_delay_us(100); // Wait 100 microseconds for the VFD.
		LCD_PORT ^= (((flags & 0x80) >> 0x07) << LCD_SIO) + LCD_CLK_BITMASK; // Shift out a single bit, toggle the state of clk as well. (it's low now)
		_delay_us(100); // Wait 100 microseconds for the VFD.
		LCD_PORT ^= LCD_CLK_BITMASK; // Toggle clk again (It's high again)	
		flags = flags << 0x01; // Shift byte to the left 1 bit.
	}
	
	// Loop to transfer the bytes
	for(volatile uint8_t i = 0; i < 7; i++) {
		_delay_us(100); // Wait 100 microseconds for the VFD.
		LCD_PORT ^= (((byte & 0x80) >> 0x07) << LCD_SIO) + LCD_CLK_BITMASK; // Shift out a single bit, toggle the state of clk as well. (it's low now)
		_delay_us(100); // Wait 100 microseconds for the VFD.
		LCD_PORT ^= LCD_CLK_BITMASK; // Toggle clk again (It's high again)
		byte = byte << 0x01; // Shift byte to the left 1 bit.		
	}
	
	LCD_PORT ^= (LCD_STB_BITMASK); // Make STB high again
	
	sei(); // Re-enable interrupts
}


/************************************************************************/
/* Initialises IO ports for the VFD										*/
/************************************************************************/
void LCD_InitIO() {
	// Configure all three pins the LCD uses as outputs.
	LCD_DDR |= LCD_STB_BITMASK | LCD_CLK_BITMASK | LCD_SIO_BITMASK;
}

/************************************************************************/
/* Initialises the VFD with the specified brightness.					*/
/************************************************************************/
void LCD_Init(uint8_t brightness) {
	brightness &= 0x03;
	
	// Init LCD in 2 line mode, with specified brightness
	LCD_WriteByteToLCD(0x38 | brightness, 0x00);
	
	// Set the cursor movement and autoincrement
	LCD_WriteByteToLCD(0x06, 0x00);
}

/************************************************************************/
/* Resets the VFD's controller.											*/
/************************************************************************/
void LCD_ResetLCD() {
	LCD_Init(0);
	LCD_ClearScreen();
}

/************************************************************************/
/* Clears the entire screen and resets cursor to (0, 0).				*/
/************************************************************************/
void LCD_ClearScreen() {
	LCD_WriteByteToLCD(0x01, 0x00);
	LCD_CursorHome(); // reset cursor to (0, 0).
}


/************************************************************************/
/* Puts text from RAM to the current cursor position.					*/
/************************************************************************/
void LCD_PutText(uint8_t x, uint8_t y, char *text) {
	// Make sure y is in range
	y &= 0x03;
	
	// Calculate DD-RAM loc (y * 0x40, plus x)
	uint8_t ddramloc = (y << 0x06) + x;
	
	// Write DD-RAM address
	LCD_WriteByteToLCD(0x80 | ddramloc, 0x00);
	
	// Do we have more text to write?
	while(text != 0x00) {
		// Write next character. (RS = 1, RW = 0)
		LCD_WriteByteToLCD((uint8_t) text++, 0x02);
	}
}

/************************************************************************/
/* Puts text from PROGMEM to the current cursor position.				*/
/************************************************************************/
void LCD_PutText_P(uint8_t x, uint8_t y, char *text) {
	// Make sure y is in range
	y &= 0x03;
	
	// Calculate DD-RAM loc (y * 0x40, plus x)
	uint8_t ddramloc = (y << 0x06) + x;
	
	// Write DD-RAM address
	LCD_WriteByteToLCD(0x80 | ddramloc, 0x00);
	
	// Do we have more text to write?
	while(pgm_read_byte(text) != 0x00) {
		// Write next character. (RS = 1, RW = 0)
		LCD_WriteByteToLCD(pgm_read_byte(text++), 0x02);
	}	
}


/************************************************************************/
/* Puts a character from RAM to a specific on-screen position.			*/
/************************************************************************/
void LCD_PutChar(uint8_t x, uint8_t y, char character) {
	// Make sure y is in range
	y &= 0x03;
	
	// Calculate DD-RAM loc (y * 0x40, plus x)
	uint8_t ddramloc = (y << 0x06) + x; 
	
	// Write DD-RAM address
	LCD_WriteByteToLCD(0x80 | ddramloc, 0x00);
	
	// Write character to DD-RAM (RS is set to 1, RW set 0)
	LCD_WriteByteToLCD(character, 0x02);
}

/************************************************************************/
/* Puts a character from PROGMEM to a specific on-screen position.		*/
/************************************************************************/
void LCD_PutChar_P(uint8_t x, uint8_t y, char character) {
	// Just call the RAM version of this routine.
	LCD_PutChar(x, y, pgm_read_byte(character));
}	


/************************************************************************/
/* Sets a specific custom character in CG-RAM from PROGMEM.				*/
/************************************************************************/
void LCD_SetCGRAM(uint8_t character, char *data) {
	// Make sure value is in range
	character &= 0x07;
	
	// Write the CGRAM address
	LCD_WriteByteToLCD(0x40 | (character << 0x03), 0x00);
	
	for (volatile uint8_t i = 0; i < 8; i++) {
		// Write a character of the CGRAM data (RS = 1, RW = 0)
		LCD_WriteByteToLCD(pgm_read_byte(data++), 0x02);
	}
}


/************************************************************************/
/* Resets the cursor to the home position, (0,0).						*/
/************************************************************************/
void LCD_CursorHome() {
	LCD_WriteByteToLCD(0x02, 0x00);
}

/************************************************************************/
/* Sets the cursor's position to the specified X and Y coordinates.		*/
/************************************************************************/
void LCD_SetCursorPos(uint8_t x, uint8_t y) {
	// TODO: Implement	
}

/************************************************************************/
/* Sets the LCD's internal state, such as disable/enable display, cursor*/
/* display, and the cursor blinking.									*/
/************************************************************************/
void LCD_SetDisplayState(uint8_t dispOn, uint8_t cursorOn, uint8_t cursorBlink) {
	LCD_WriteByteToLCD(0x08 | ((dispOn & 0x01) << 0x02) | ((cursorOn & 0x01) << 0x01) | (cursorBlink & 0x01), 0x00);
}