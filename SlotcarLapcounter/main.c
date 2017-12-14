//avrdude -e -c avr910 -p m16 -P /dev/ttyUSB0 -U flash:w:SlotcarLapcounter.hex



#define	F_CPU						16000000						// processor clock frequency in Hertz
// standard AVR libraries
#include <stdlib.h>
#include <stdint.h>													// standard integer declarations
#include <avr/io.h>													// io register definitions
#include <avr/interrupt.h>											// enable interrupt handling
#include <avr/sleep.h>												// sleep and power down mode control
#include <avr/wdt.h>												// watchdog timer
#include <util/delay.h>												// delay routines
// libraries for using peripheral modules
// #include "ADC.h"
#include "lcd_HD44780.h"

#define TOLERANCE							2

// Port-Pin declarations for IO port B
#define LED_RED								4		// LED-red
#define LED_GREEN							5		// LED-green

// Port-Pin declarations for IO port C
#define ADC1								1
#define ADC2								2
#define BUTTON3								3
		
// Port-Pin declarations for IO port D
#define BUZZER								5		// buzzer (not implemented)


// macro switches
#define	LED_RED_ON			PORTB |=  (1<<LED_RED)
#define	LED_RED_OFF			PORTB &= ~(1<<LED_RED)
#define	LED_GREEN_ON		PORTB |=  (1<<LED_GREEN)
#define	LED_GREEN_OFF		PORTB &= ~(1<<LED_GREEN)


// used vars
uint8_t		currentValue1=0;					// store current ADC results
uint8_t		currentValue2=0;
uint8_t		offsetValue1=0;						// store calibration values
uint8_t		offsetValue2=0;
uint8_t		toggleChannel=0;

uint8_t		writeNew1 = 0;						// used for lcd writing 
uint8_t		writeNew2 = 0;
uint8_t		str1[16];
uint8_t		str2[16];
int8_t		str3[8];

uint16_t	counterInput1 = 0;
uint16_t	counterInput2 = 0;
uint16_t	result1 = 0xFFFF;
uint16_t	result2 = 0xFFFF;
uint16_t	bestResult1 = 0xFFFF;				// store lap times
uint16_t	bestResult2 = 0xFFFF;
uint16_t	bestResult1last = 0xFFFF;
uint16_t	bestResult2last = 0xFFFF;

volatile uint16_t	timer1Counter = 0;			// counts time between laps
volatile uint8_t	SREG_temp;					



//------------------------------------------------------------------------------------------------
// Error Modus
//------------------------------------------------------------------------------------------------
void ErrorMode(void)
{
	while(1)
	{
		LED_RED_ON;
		_delay_ms(200);
		LED_RED_OFF;
		_delay_ms(200);
	}
}


void setToInput1(void)
{
	ADMUX &= ~(1<<MUX1);										// set input ADC1
	ADMUX |=  (1<<MUX0);
}

void setToInput2(void)
{
	ADMUX &= ~(1<<MUX0);										// set input ADC2
	ADMUX |=  (1<<MUX1);
}

void getADCValue1(void)
{
	//AVCC as Ref-V; Result is LEFT! adjusted; Single Ended Input at ADC1 (0001):
	setToInput1();
	ADCSRA |= ((1<<ADSC));										// start conversion
}

void getADCValue2(void)
{
	//Abstand2 berechnen at ADC2 (0010):
	setToInput2();
	ADCSRA |= (1<<ADSC);										// start conversion
}

void writeNessySpaces(int secResult)
{
	if(secResult<10)
	LCD_Write_String("  ");
	else
	if(secResult<100)
	LCD_Write_String(" ");
}


//------------------------------------------------------------------------------------------------
// Interrupt 16-Bit Timer (Timer1)
//----------------------------------------------------
ISR(TIMER1_OVF_vect)
{
	TCNT1 = 64911;				// 65536 - 625  -> (10ms to overflow at 16MHz)
	SREG_temp = SREG;

	timer1Counter++;			// Timer var 16-bit timer 

	SREG = SREG_temp;
}



//------------------------------------------------------------------------------------------------
// Interrupt ADC Conversion Complete 
//------------------------------------------------------------------------------------------------
ISR(ADC_vect)
{
	SREG_temp = SREG;
	if(toggleChannel == 0)											// Value1 measured
	{
		currentValue1=ADCH;											// read out ADC1
		toggleChannel = 1;
		setToInput2();												// toggle to ADC2 input 
		ADCSRA |= (1<<ADSC);										// start conversion (channel 2)
	}
	else
	{	
		currentValue2=ADCH;											// read out ADC2
		toggleChannel = 0;
		setToInput1();												// toggle to ADC1 input 
		
		if( (currentValue1 <= offsetValue1) &&						// start new conversion only it no car hit finish line!
			(currentValue2 <= offsetValue2) )						// else: handle lap time in main() and start new convertion there! 
		{
			ADCSRA |= (1<<ADSC);									// start conversion (channel 1)
		}
	}
	SREG = SREG_temp;
}




//=======================================================================
// Init  ADC, see page 200ff:
void MY_ADC_Init(void)
{	
	//see page 212, ADMUX=((REFS1)|(REFS0)|(ADLAR)|(MUX4)|(MUX3)|(MUX2)|(MUX1)|( MUX0)):
	ADMUX = 0;
	//AVCC as Ref-V; Result is LEFT! adjusted; Single Ended Input at ADC2:
	ADMUX |= ((0<<REFS1)|(1<<REFS0)|(1<<ADLAR)|(0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(1<<MUX0));
	//Result is LEFT adjusted: it is sufficient to read ADCH!!)

	//ADCSRA = ADEN ADSC ADATE ADIF ADIE ADPS2 ADPS1 ADPS0	
	ADCSRA = 0;
	//ADC on; ADC Start Conversion off; Auto Trigger off; ;ADC Interrupt off; ADC Prescaler=128
	//ADC-Prescaler: make sure ADC frequency is between 50-200kHz)
	//16MHz / 128 => 125kHz
	ADCSRA |= ((1<<ADEN)|(0<<ADSC)|(0<<ADFR)|(0<<ADIF)|(0<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));
	
	//Start convertion by setting ADSC-bit, 
	//convertion takes 13 adc-cycles, 125kHz is a cycles every 8µ  (13*8µS=104µS)
	//ADC = (VIN*1024) / VREF
	//At RefV=5V: 0V=0, 1V~205, 2V~410, 3V~615, 4V~820...
}


//=======================================================================
// Reset the system.
void Reset (void) 
{
	// set port pin direction PortB:
	DDRB |= ((1<<LED_RED)|(1<<LED_GREEN));
    
	// set port pin direction PortC:
	PORTC = 0;
	
	// set port pin direction PortD: 
	//DDRD = 0;
	//DDRD |= (1<<BUZZER);
}



//####################################################################################
// Main program.
//####################################################################################
int main(void)
{
	wdt_disable ();												// stop the watchdog
	sei ();														// enable global interrupts

	//LED_GREEN_ON;
	//-Start init:
	Reset();
	MY_ADC_Init();
	LCD_Init();													// LCD initalisieren
	LCD_GoXY(1, 2);
	LCD_Write_String("Speed Monitor");
	_delay_ms(2000);
	
	TCCR1B |= ((1<<CS12)|(0<<CS11)|(0<<CS10));					// set Timer1(16bit) to CPU-Takt/256 
	TCNT1 = 64911;												// 65536 - 625  -> (10ms to overflow at 16MHz)
	TIMSK	|=	(1<<TOIE1);										// Enable Timer1
	
	uint8_t bIsGreenOn = 1;
	uint8_t bIsRedOn = 0;

	uint8_t countRound1 = 0;
	uint8_t countRound2 = 0;
	
	
	//calibation of hall sensors
	LCD_Clear();
	LCD_Write_String("Calibration...");

	getADCValue2();												// get calib value2
	_delay_ms(20);		
	offsetValue2 = ADCH;
	offsetValue2 += TOLERANCE;

	getADCValue1();												// get calib value1
	_delay_ms(20);		
	offsetValue1 = ADCH;
	offsetValue1 += TOLERANCE;

	//write calib result
	itoa(offsetValue1, (char*)str2, 10 );
	LCD_Clear();
	LCD_Write_String("Kalib1: ");
	LCD_Write_String(str2);
	LCD_GoXY(1, 2);
	itoa(offsetValue2, (char*)str2, 10 );
	LCD_Write_String("Kalib2: ");
	LCD_Write_String(str2);
	_delay_ms(3000);
	
	int c=9;														// start sequence 10s: 
	while(c>0)
	{
		LCD_Clear();
		itoa(c, (char*)str2, 10 );
		LCD_GoXY(8, 1);
		LCD_Write_String(str2);
		_delay_ms(1000);
		c--;
	}
	LCD_Clear();
	LCD_GoXY(7, 2);
	LCD_Write_String("GO !!");

	counterInput1 = timer1Counter;
	counterInput2 = counterInput1;
	ADCSRA |= (1<<ADIE);											// set conversion complet interrupt enable
	ADCSRA |= (1<<ADSC);											// start conversion (of channel1)

	while(1)
	{
		if(bIsGreenOn)
		{
			LED_GREEN_OFF;
			bIsGreenOn = 0;
		}
		else
		{
			LED_GREEN_ON;
			bIsGreenOn = 1;
		}

		// falls taste gedrückt (PC3 low)
		if ( (PINC & (1<<BUTTON3)) )
		{
			_delay_ms(50);											// debounce button
			while( (PINC & (1<<BUTTON3)) ){};						// wait until button released
			if(bIsRedOn)
			{
				LED_RED_OFF;
				bIsRedOn = 0;
				//bestResult1 = 0xFFFF;								// reset best time
				//bestResult2 = 0xFFFF;
			}
			else
			{	// RED_ON => show best time
				LED_RED_ON;
				bIsRedOn = 1;
			}
		}
	
		// if car1 is on finish line
		if(currentValue1 > offsetValue1)
		{
			uint16_t localCounter = timer1Counter;					// get current timer value
			if(localCounter > counterInput1)
				result1 = localCounter - counterInput1;				// calc result time
			else
				result1 = (65536 - counterInput1) + localCounter;	// calc result time after overflow

			counterInput1 = localCounter;							// store new current timer value
			if(result1 > 40)
			{
				writeNew1=1;										// set new value and write to lcd
				countRound1++;
				if( result1 < bestResult1 )							// store best result1
					bestResult1 = result1;
			}
		}
	
		// if car2 is on finish line
		if(currentValue2 > offsetValue2)
		{
			uint16_t localCounter = timer1Counter;					// get current timer value
			if(localCounter >= counterInput2)
				result2 = localCounter - counterInput2;				// calc result time
			else
				result2 = (65536 - counterInput2) + localCounter;	// calc result time after overflow

			counterInput2 = localCounter;							// store new current timer value
			if(result2 > 40)
			{
				writeNew2=1;										// set new value and write to lcd
				countRound2++;
				if( result2 < bestResult2 )							// store best result2 
					bestResult2 = result2;
			}
		}

		ADCSRA |= (1<<ADSC);										// start ADC conversion (channel 1)
	
		if( (writeNew1) && (!bIsRedOn) )							// write current time if 'best time mode' not activ
		{
			int iSec = 0;		
			if(result1 >= 100)
			{
				iSec = result1/100;
				result1 -= iSec * 100;
				itoa(iSec, (char*)str1, 10 );
			}
			else
			{
				itoa(0, (char*)str1, 10 );
			}
			itoa(result1, (char*)str2, 10 );
			LCD_GoXY(1, 1);
			LCD_Write_String("Time1:");
			writeNessySpaces(iSec);
			LCD_Write_String(str1);
			LCD_Write_String(".");
			LCD_Write_String(str2);
			itoa(countRound1, (char*)str3, 10 );
			LCD_GoXY(14, 1);
			LCD_Write_String(str3);
			writeNew1 = 0;
		}

		
		if( (writeNew2) && (!bIsRedOn) )							// write current time if 'best time mode' not activ
		{
			int iSec = 0;
			if(result2 >= 100) 
			{
				iSec = result2/100;
				result2 -= iSec * 100;
				itoa(iSec, (char*)str1, 10 );
			}
			else
			{
				itoa(0, (char*)str1, 10 );
			}
			itoa(result2, (char*)str2, 10 );
			LCD_GoXY(1, 2);
			LCD_Write_String("Time2:");
			writeNessySpaces(iSec);
			LCD_Write_String(str1);
			LCD_Write_String(".");
			LCD_Write_String(str2);
			itoa(countRound2, (char*)str3, 10 );
			LCD_GoXY(14, 2);
			LCD_Write_String(str3);
			writeNew2 = 0;
		}

		//write best time result
		if( (bIsRedOn) && ((bestResult1last!=bestResult1)||(bestResult2last!=bestResult2)) )
		{
			uint16_t temp = bestResult1last = bestResult1;
			if( (temp >= 100) )
			{
				int iSec = temp/100;
				temp -= iSec * 100;
				itoa(iSec, (char*)str1, 10 );
			}
			else
				itoa(0, (char*)str1, 10 );
			
			LCD_Clear();
			itoa(temp, (char*)str2, 10 );
			LCD_GoXY(1, 1);
			LCD_Write_String("Best1: ");
			LCD_Write_String(str1);
			LCD_Write_String(".");
			LCD_Write_String(str2);

			temp = bestResult2last = bestResult2;
			if( (temp >= 100) )
			{
				int iSec = temp/100;
				temp -= iSec * 100;
				itoa(iSec, (char*)str1, 10 );
			}
			else
				itoa(0, (char*)str1, 10 );
			
			itoa(temp, (char*)str2, 10 );
			LCD_GoXY(1, 2);
			LCD_Write_String("Best2: ");
			LCD_Write_String(str1);
			LCD_Write_String(".");
			LCD_Write_String(str2);
		}// end of write best time result
	}// end of while(1)
}// end of main==============================================================
