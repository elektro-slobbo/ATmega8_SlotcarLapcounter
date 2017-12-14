/*
,-----------------------------------------------------------------------------------------.
| LCD
|-----------------------------------------------------------------------------------------
| This file provides routines for controlling an extrenally connected LC-Display.
| The LCD must possess a HD44780 compatible controller. Data exchange is done in 4 bit mode.
| The LCD is connected to the microcontroller by 6 port pins. The wiring diagram looks like:
|
| GND     ---------------- LCD Pin  1 (Logic Supply GND)
| +5V or Port Pin -------- LCD Pin  2 (Logic Supply +5V)
| VADJ    ---------------- LCD Pin  3 (LCD Contrast Adjust Voltage)
| LCD_RS  ---------------- LCD Pin  4 (Register Select)
| GND     ---------------- LCD Pin  5 (Read/Write)
| LCD_E   ---------------- LCD Pin  6 (Enable)
|                          LCD Pin  7 (D0)
|                          LCD Pin  8 (D1)
|                          LCD Pin  9 (D2)
|                          LCD Pin 10 (D3)
| LCD_D04 ---------------- LCD Pin 11 (D4)
| LCD_D15 ---------------- LCD Pin 12 (D5)
| LCD_D26 ---------------- LCD Pin 13 (D6)
| LCD_D37 ---------------- LCD Pin 14 (D7)
| +5V     ---|R=56 Ohm|--- LCD Pin 15 (Backlight Anode)
| GND     ---------------- LCD Pin 16 (Backlight Cathode)
|
| Because data are only written to the display, R/W (Pin 5) can be connected directly to GND.
| The LCD Contrast Adjust Voltage can simply be created from the supply voltage using a 
| potentiometer or a voltage divider (+5V--|100k|---VADJ---|5k6|--GND).
| The backlight contrast greatly depends on the temperature, so some extra circuitry might
| be neccessary!
| The LCD Logic Supply voltage can be provided by a freely chosen port pin. This allows you
| to power down the display if the microcontroller circuitry is going to power-down mode.
|
| All data written to the display are strored in an internal write buffer. This enables the
| driver to provide some extras like automatic scroll of the display content when reaching
| the end of the last line.
| Some special characters ('ä','ö', or the like) are automatically replaced with their
| LCD ROM counterparts.
| If automatic CR/FL is enabled, the cursor will automatically skip to the beginning of the
| next line when reaching the end of the previous line. When the cursor reaches the end of
| the last line, the display content will automatically scroll up one line.
| If automatic CR/LF is disabled, the cursor will stop when it reaches the end of the line.
| All following write attempts will be discarded.
| The automatic CR/LF funtion is enabled by setting LCD_Autofeed to a non-zero value.
| It is enabled by default.
|
| The user can easily return the cursor to the beginning of the current line by writing
| the CR character (#13) to the display. With the LF character (#10) the user can proceed the
| cursor to the next line. 
|
| LC Displays with 1,2 or 4 lines and 8,10, or 16 characters per line are supported.
|
|-----------------------------------------------------------------------------------------
| Usage:
| First, you have to reset and init the LCD. This is done by the routine
|   void LCD_Init (void)
| It will set up the ports, reset the buffer and send some init codes to the display.
| The display will be cleared and the cursor is set to position 1,1. Cursor is off.
|
| Then you can write characters to the display by using
|   void LCD_Write (Character)
| If "Character" is not a control character (like #10 "Carriage Return) it will be displayed.
| If it is a control character, the correspondig action will take place.
|
| You can directly set the cursor to a new display position if you wish with
|   void LCD_GoXY (uint8_t New_X, uint8_t New_Y)
| "New_X" may be in the range of 1..LCD_XSize, "New_Y" in the range of 1..LCD_YSize.
| Illegal values are corrected automatically.
|
| To switch the cursor on and off, respectively, use
|   void LCD_Cursor_On (void)
|   void LCD_Cursor_Off (void)
|
| To clear the display content and return the cursor home to position 1,1 call
|   void LCD_Clear (void)
|
| Author: Daniel Möhlenbrock (d.moehlenbrock[at]web.de)
| Date: 2007/05/30
|-----------------------------------------------------------------------------------------
| License:
| This program is free software; you can redistribute it and/or modify it under
| the terms of the GNU General Public License as published by the Free Software
| Foundation; either version 2 of the License, or (at your option) any later
| version.
| This program is distributed in the hope that it will be useful, but
|
| WITHOUT ANY WARRANTY;
|
| without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
| PURPOSE. See the GNU General Public License for more details.
|
| You should have received a copy of the GNU General Public License along with
| this program; if not, write to the Free Software Foundation, Inc., 51
| Franklin St, Fifth Floor, Boston, MA 02110, USA
|
| http://www.gnu.de/gpl-ger.html
`-----------------------------------------------------------------------------------------*/

#ifndef _LCD_H_
#define _LCD_H_

#include <avr\io.h> 
#include <util\delay.h>
#include <stdint.h>
#include <avr\pgmspace.h>

#define		LCD_XSize					16						// LC display line length
#define		LCD_YSize					2						// LC display number of lines

uint8_t		LCD_Buffer					[LCD_XSize*LCD_YSize];	// display data buffer
uint8_t 		LCD_Buffer_Index;								// display data buffer index

uint8_t		LCD_X;												// current LCD cursor x position
uint8_t		LCD_Y;												// current LCD cursor y position
uint8_t		LCD_Autofeed;										// automatic CR/LF enable
uint8_t		LCD_Clear_Buffer_On_Clear;							// internally used parameter

//------------------------------------------------------------------------------------------------------
//#define		PORT_LCD_Power				PORTB				// LCD Power supply PORT register
//#define		DDR_LCD_Power				DDRB				// LCD Power supply DDR register
//#define		LCD_Power					1					// LCD Power suply port pin
//#define		LCD_Power_On				PORT_LCD_Power |=  (1<<LCD_Power)	// enable LCD Power supply
//#define		LCD_Power_Off				PORT_LCD_Power &= ~(1<<LCD_Power)	// disable LCD Power supply

#define 	PORT_LCD_D04				PORTD				// port output register LCD data line bit 0,4
#define 	DDR_LCD_D04					DDRD				// data direction register LCD data line bit 0,4
#define		LCD_D04						4					// port pin LCD data line bit 0,4
#define 	PORT_LCD_D15				PORTD				// port output register LCD data line bit 1,5
#define 	DDR_LCD_D15					DDRD				// data direction register LCD data line bit 1,5
#define		LCD_D15						5					// port pin LCD data line bit 1,5
#define 	PORT_LCD_D26				PORTD				// port output register LCD data line bit 2,6
#define 	DDR_LCD_D26					DDRD				// data direction register LCD data line bit 2,6
#define		LCD_D26						6					// port pin LCD data line bit 2,6
#define 	PORT_LCD_D37				PORTD				// port output register LCD data line bit 3,7
#define 	DDR_LCD_D37					DDRD				// data direction register LCD data line bit 3,7
#define		LCD_D37						7					// port pin LCD data line bit 3,7

#define		PORT_LCD_RS					PORTB				// port output register LCD register select bit
#define		DDR_LCD_RS					DDRB				// data direction register LCD register select bit
#define		LCD_RS						0					// port pin LCD register select bit
#define		PORT_LCD_RW					PORTB				// port output register LCD register select bit
#define		DDR_LCD_RW					DDRB				// data direction register LCD register select bit
#define		LCD_RW						1
#define		PORT_LCD_E					PORTB				// port output register LCD enable bit
#define		DDR_LCD_E					DDRB				// data direction register LCD enable bit
#define		LCD_E							2					// port pin LCD enable bit

#define		PORT_LCD_LED				PORTB				// port controlling the LCD LED backlight
#define		DDR_LCD_LED					DDRB				// port controlling the LCD LED backlight
#define		LCD_LED						3					// port pin controlling the LCD LED backlight
//------------------------------------------------------------------------------------------------------

// Routines for initializing, controlling and writing data to the display
void LCD_GoXY (uint8_t New_X, uint8_t New_Y);
void LCD_Cursor_On (void);
void LCD_Cursor_Off (void);
void LCD_Clear (void);
void LCD_Write (uint8_t Character);
void LCD_Init (void);
void LCD_LED_On (void);
void LCD_LED_Off (void);


// Do not use these procedures (unless you know what you are doing :)! They are for internal use only!
void LCD_WrNibble (uint8_t Nibble);
void LCD_WrByte (uint8_t Data);
void LCD_WrD (uint8_t Data);
void LCD_WrI (uint8_t Instruction);
void LCD_Write_Character (uint8_t Character);
uint8_t LCD_Replace (uint8_t Character);
void LCD_Line_Feed (void);
void LCD_Carriage_Return (void);


//-------------------------------------------------------------------------------------------
void LCD_LED_On(void) 
{
	// configure LCD control pin as output and pull it high
	DDR_LCD_LED  |= (1<<LCD_LED);
	PORT_LCD_LED |= (1<<LCD_LED);
}

void LCD_LED_Off(void) 
{
	// configure LCD control pin as output and pull it low
	DDR_LCD_LED  |=  (1<<LCD_LED);
	PORT_LCD_LED &= ~(1<<LCD_LED);
}


//------------------------------------------------------------------------------------------
void LCD_WrNibble (uint8_t Nibble) 
{
  if (Nibble & 0x10) PORT_LCD_D04 |=  (1<<LCD_D04);		// if Nibble.4 is set then set LCD data line 0,4
  else 					PORT_LCD_D04 &= ~(1<<LCD_D04); 	// else clear LCD data line 0,4
  if (Nibble & 0x20) PORT_LCD_D15 |=  (1<<LCD_D15); 	// if Nibble.5 is set then set LCD data line 1,5
  else 					 PORT_LCD_D15 &= ~(1<<LCD_D15); // else clear LCD data line 1,5
  if (Nibble & 0x40) PORT_LCD_D26 |=  (1<<LCD_D26); 	// if Nibble.6 is set then set LCD data line 2,6
  else 					PORT_LCD_D26 &= ~(1<<LCD_D26); 	// else clear LCD data line 2,6
  if (Nibble & 0x80) PORT_LCD_D37 |=  (1<<LCD_D37); 	// if Nibble.7 is set then set LCD data line 3,7
  else 					PORT_LCD_D37 &= ~(1<<LCD_D37); 	// else clear LCD data line 3,7
  _delay_us (1);										// wait for one microsecond
  PORT_LCD_E |= (1<<LCD_E);								// pull LCD E high
  _delay_us (1);										// wait for 1 microsecond
  PORT_LCD_E &= ~(1<<LCD_E);							// pull LCD E low
}

void LCD_WrByte (uint8_t Data) 
{
  _delay_us (40);										// wait for 40 microseconds
  LCD_WrNibble (Data);									// write high nibble
  LCD_WrNibble (Data << 4);								// write low nibble
} 
  
void LCD_WrD (uint8_t Data) 
{
  PORT_LCD_RS |= (1<<LCD_RS);							// set LCD register select bit
  LCD_WrByte (Data);									// then write the data byte
}

void LCD_WrI (uint8_t Instruction) 
{
  PORT_LCD_RS &= ~(1<<LCD_RS);							// clear LCD register select bit
  LCD_WrByte (Instruction);								// then write the instruction byte
}


void LCD_GoXY (uint8_t New_X, uint8_t New_Y) 
{
  uint8_t Offset;
  if (New_X > LCD_XSize) New_X = LCD_XSize;				// check boundaries
  if (New_Y > LCD_YSize) New_Y = LCD_YSize;				// check boundaries
  if (!(New_X)) New_X = 1;								// check boundaries
  if (!(New_Y)) New_Y = 1;								// check boundaries
  LCD_X = New_X;										// update cursor x position
  LCD_Y = New_Y;										// update cursor y position
  // calculate line offset
  // line 1: Offset = 0x00
  // line 2: Offset = 0x40
  // line 3: Offset = 0x14
  // line 4: Offset = 0x54
  New_Y--;												// decrement New_Y
  New_X--;												// decrement New_X
  Offset = 0x80+New_X;									// set Offset to LCD_X-1; add 0x80 (instruction bit)
  if (New_Y & 0x01) Offset = Offset+0x40;				// increment Offset by 0x40 if New_Y.0 is set
  if (New_Y & 0x02) Offset = Offset+0x14;				// increment Offset by 0x14 if New_Y.1 is set
  LCD_WrI (Offset);										// set cursor to new position
  LCD_Buffer_Index = (uint8_t) (New_Y*LCD_XSize)+New_X;	// calculate new Buffer offset
}


//------------------------------------------------------------------------------------------------------
void LCD_Cursor_On (void) 
{
  LCD_WrI (0x0F);										// write instruction (cursor on)
}
//------------------------------------------------------------------------------------------------------
void LCD_Cursor_Off (void) 
{
  LCD_WrI (0x0C);										// write instruction (cursor off)
}
//------------------------------------------------------------------------------------------------------
void LCD_Clear (void) 
{
  LCD_WrI (0x01);										// write instruction 0x01 (clear display)
  _delay_ms (2);										// wait for 2 milliseconds
  LCD_WrI (0x02);										// write instruction 0x02 (cursor home)
  _delay_ms (2);										// wait for 2 milliseconds
  LCD_X = 1;											// set current cursor position x = 1
  LCD_Y = 1;											// set current cursor position y = 1
  if (LCD_Clear_Buffer_On_Clear) {
    for (LCD_Buffer_Index = 0;LCD_Buffer_Index < LCD_XSize*LCD_YSize;LCD_Buffer_Index++)
      LCD_Buffer[LCD_Buffer_Index] = 32;				// clear buffer content
  }
  LCD_Buffer_Index = 0;									// reset data buffer index
}


const uint8_t LCD_SpecialChars[] PROGMEM   = {'ä',0xE1,'Ä',0xE1,'ö',0xEF,'Ö',0xEF,'ü',0xF5,'Ü',0xF5,'ß',0xE2,'°',0xDF};

uint8_t LCD_Replace (uint8_t Character) 
{
  uint16_t Index;
  for (Index = 0;Index < sizeof(LCD_SpecialChars); Index = Index+2) 
  {
    if (Character == pgm_read_byte(&LCD_SpecialChars[Index])) return pgm_read_byte(&LCD_SpecialChars[Index+1]);
  }
  return (Character);
}


//------------------------------------------------------------------------------------------------------
void LCD_Line_Feed (void) 
{
  uint8_t YCount;
  uint8_t XCount;
  if (LCD_Y == LCD_YSize) 
  {
    LCD_Clear_Buffer_On_Clear = 0x00;					// prevent clearing the buffer content
    LCD_Clear ();
    LCD_Clear_Buffer_On_Clear = 0xFF;	
	for (YCount=0;YCount<LCD_YSize-1;YCount++) 
	{
	  for (XCount=0;XCount<LCD_XSize;XCount++)
        LCD_Write_Character (LCD_Buffer[LCD_Buffer_Index+LCD_XSize]);
	}
  }
  else LCD_GoXY (LCD_X,LCD_Y+1);
}


//------------------------------------------------------------------------------------------------------
void LCD_Write_Character (uint8_t Character) 
{
  if (LCD_X <= LCD_XSize)								// if cursor is still in boundaries...
  {									
		Character = LCD_Replace (Character);			// replace special characters/align to LCD font
		LCD_WrD (Character);							// write character to the display
		LCD_X++;										// increment cursor X position
		LCD_Buffer[LCD_Buffer_Index] = Character;		// store character in LCD buffer
		LCD_Buffer_Index++;								// increment LCD buffer index
  }

  if ((LCD_X > LCD_XSize) & LCD_Autofeed)				// if end of line is reached and autofeed is activated
  {					
		LCD_Carriage_Return ();							// do carriage return
		LCD_Line_Feed ();								// do line feed
  }
}


//------------------------------------------------------------------------------------------------------
void LCD_Carriage_Return (void) 
{
  LCD_GoXY (1,LCD_Y);
}

//------------------------------------------------------------------------------------------------------
void LCD_Write (uint8_t Character) 
{
  switch (Character) 
  {
    case (13):	LCD_Carriage_Return ();						// carriage return?
				break;
    case (10):	LCD_Line_Feed ();							// line feed?
				break;
    default:	LCD_Write_Character (Character);			// else write character

  }
}

//------------------------------------------------------------------------------------------------------
void LCD_Init (void)
{
  //LCD_Power_Off;								// switch off the LCD power supply
  //DDR_LCD_Power |= (1<<LCD_Power);			// configure LCD power pin as output
  PORT_LCD_E   &= ~(1<<LCD_E);					// pull LCD E low
  DDR_LCD_E    |=  (1<<LCD_E);					// configure LCD E as output	
  PORT_LCD_RS  &= ~(1<<LCD_RS);					// pull LCD RS low - select instruction registers
  DDR_LCD_RS   |=  (1<<LCD_RS);					// configure LCD register select as output
  PORT_LCD_RW  &=  ~(1<<LCD_RW);				// pull LCD Rw low - select instruction registers
  DDR_LCD_RW   |=  (1<<LCD_RW);  				// configure LCD RW select as output
  DDR_LCD_D04  |=  (1<<LCD_D04);				// configure LCD data line port pin as output
  DDR_LCD_D15  |=  (1<<LCD_D15);				// configure LCD data line port pin as output
  DDR_LCD_D26  |=  (1<<LCD_D26);				// configure LCD data line port pin as output
  DDR_LCD_D37  |=  (1<<LCD_D37);				// configure LCD data line port pin as output
  _delay_ms (15);								// wait for 15 milliseconds
  //LCD_Power_On;								// enable the LCD power supply
  _delay_ms (15);								// wait for 15 milliseconds
  LCD_WrNibble (0x30);							// write high nibble of 0x30
  _delay_ms (15);								// wait for 15 milliseconds
  LCD_WrNibble (0x30);							// write high nibble of 0x30
  _delay_ms (15);								// wait for 15 milliseconds
  LCD_WrNibble (0x30);							// write high nibble of 0x30
  _delay_ms (15);								// wait for 15 milliseconds
  LCD_WrNibble (0x20);							// write high nibble of 0x20
  LCD_WrI (0x28);								// write instruction 0x28
  LCD_WrI (0x08);								// write instruction 0x08
  LCD_Cursor_Off ();							// switch cursor off
  LCD_WrI (0x06);								// write instruction 0x06
  LCD_Clear_Buffer_On_Clear = 0xFF;				// clear buffer content on clear command
  LCD_Clear ();									// cear the display, set cursor to position 1,1
  LCD_Autofeed = 0xFF;							// enable automatic CR/LF
}

//------------------------------------------------------------------------------------------------------
void LCD_Write_String (uint8_t *RAMString) 
{
  while (*RAMString)
    LCD_Write (*RAMString++);
}


#endif
