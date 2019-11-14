/*
 Simple test program for RCA 1802 microprocessor
 Jeffrey T. Birt (Hey Birt!)
 website: http://www.soigeneris.com
 YouTube: http://www.youtube.com/c/HeyBirt
*/
// NOTES: Uses a Arduino Mega 2560

#include "1802_Tester.h"

// This is not finalized yet
// used to keep track on main loop
enum SYS_STATE
{
	PwrOn,
	Initilized,
	Reset,
	Wait,
	Run
};

// Load D with 0x55 store to Address pointed to by register 1
// 0xF8, 0x55, 0x51, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4,

// Q blink program
byte virtRAM[] = {	0x7A, 0xF8, 0x10, 0xB1, 0xF8, 0x00, 0xA1, 0x21,
					0x91, 0x3A, 0x07, 0x31, 0x00, 0x7B, 0x30, 0x01,
					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 
					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 
					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 
					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 
					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 
					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4 }; // virtual system RAM, 0x0000 to 0x0039

// gloabls
int Start_Delay = 0;		// delay reset release 64 half clocks after start
unsigned int Address16 = 0; // used to latch in full 16-bit address
unsigned int clkCount = 0;	// counts in 1/2 clock cycles
SYS_STATE sysState = PwrOn;

byte newPORTA = 0; byte oldPORTA = 0;	// not sure if oldPORTA actually needed
byte newPORTL = 0; byte oldPORTL = 0;	// track old state, compare to new for rising/falling edges
byte newSCx = 0; byte oldSCx = 0;		// not sure if oldSCx actually needed


// The setup() function runs once each time the micro-controller starts
void setup()
{
	PORTA = PORTA & ~ADD_BUS;	// all pins low for HIZ
	PORTB = ( PORTB & ~(CLEAR | CLOCK) ) | WAIT; // set to reset mode
	PORTC = PORTC & ~DATA_BUS;	// all pins low for HIZ
	PORTG = PORTG & ~(SC0 | SC1); // all pins low for HIZ
	PORTL = PORTL & ~(N0 | N1 | N2 | TPA | TPB | MRD | MWR | Q); // all pins low for HIZ

	DDRA = ~ADD_BUS;	// all inputs, HIZ at this point
	DDRB = CLEAR | WAIT | CLOCK; // outputs
	DDRC = ~DATA_BUS;	// all inputs, HIZ at this point
	DDRG = DDRG & ~(SC0 | SC1); // inputs, HIZ at this point
	DDRL = ~(N0 | N1 | N2 | TPA | TPB | MRD | MWR | Q); // all inputs, HIZ at this point

	sysState = Initilized;

	Serial.begin(9600); // open the serial port at 9600 bps:
	Serial.println("Starting...");
}

// Add the main program code into the continuous loop() function
void loop()
{
	if (sysState == Run)
	{
		// read in current state of I/O
		newPORTA = PINA;			 // 1802 address bus
		newPORTL = PINL;			 // 1802 status outputs
		newSCx = PING & (SC0 | SC1); // 1802 state outputs
	
		// watch for falling/rising edge of /MRD
		if ( !(newPORTL & MRD) && (oldPORTL & MRD) )
		{
			//logState("/MRD");	// falling edge so,
			portC_ModeOutput(); // enable data bus output from Arduino to 1802
		}
		else if ((newPORTL & MRD) && !(oldPORTL & MRD))
		{
			//logState("MRD");	// rising edge so,
			portC_ModeInput();	// enable data bus to input to Arduino from 1802
		}

		// watch for rising edge of TPA or TPB to latch in full 16-bit address
		if ( (newPORTL & TPA) && !(oldPORTL & TPA) )
		{
			Address16 = newPORTA; // TPA rising edge, latch in address MSB
			//logState("TPA");
		} 
		else if ((newPORTL & TPB) && !(oldPORTL & TPB))
		{
			Address16 = (Address16 << 8) | newPORTA; // TPB rising edge, read address LSB
			//logState("TPB");
		}

		// It is a bit redundent to check for the rising edge of TPB again
		// Below, we are using it as an indication that we might need to read/write
		// to the data bus but other decide logic could be used so we are keeping this
		// bit of code seperate.
		if ((newPORTL & TPB) && !(oldPORTL & TPB))
		{
			// if /MRD low Arduino writing to data bus, 1802 reading from memory
			if ( !(newPORTL & MRD) )
			{
				if (Address16 < 18)
				{
					portC_OutputValue(virtRAM[Address16]);
				}
				else
				{
					portC_OutputValue(0xC4); // NOP
					logState("NOP");
				}
			}
			// Arduino reading data from data bus, i.e. 1802 is writing to virtual memory
			else if ( !(newPORTL & MWR) )
			{
				byte fromDataBus = portC_InputValue();
				logState(String(fromDataBus, HEX) + " Data bus");
			}
		}

		// is Q rising?
		if ((newPORTL & Q) && !(oldPORTL & Q))
		{
			logState("Q turned on");
		}
		else if (!(newPORTL & Q) && (oldPORTL & Q))
		{
			logState("Q turned off");
		}

		//delay(100);			// delay for serial debug display
		clkCount++;				// inc clock 1/2 tick
		oldPORTA = newPORTA;	// save new values for comparison next cycle
		oldPORTL = newPORTL;
		oldSCx = newSCx;
	}
	else if (sysState == Initilized)
	{
		// we are using this to provide a 64 clock delay from start up to reset reelase
		Start_Delay++;
		if (Start_Delay > 63)
		{
			PORTB = PORTB | CLEAR; // put in RUN mode			
			sysState = Run;
			Serial.println("Run Mode");
			Serial.println("Clk \tSCx \tPORTL \tAdd \tData \tNote");
		}
	}

	PORTB = PORTB ^ CLOCK; // toggle clock 1/2 tick
}

// helper to dump current state out to serial port
void logState(String note)
{
	Serial.print(clkCount, HEX); Serial.print("\t");
	Serial.print(newSCx); Serial.print("\t");
	Serial.print(newPORTL, HEX); Serial.print("\t");
	Serial.print(Address16, HEX); Serial.print("\t");
	Serial.print(PINC, HEX); Serial.print("\t");
	Serial.println(note);
}

// set data bus to HIZ input mode
void portC_ModeInput(void)
{
	DDRC = ~DATA_BUS;			// change direction to all inputs
	PORTC = PORTC & ~DATA_BUS;	// all pins low for HIZ
}

// input byte from data bus
byte portC_InputValue(void)
{
	return PINC;
}

// set data bus to output mode
void portC_ModeOutput(void)
{
	DDRC = DATA_BUS;	// change direction to outputs
}

// output a byte on the data bus
void portC_OutputValue(byte value)
{
	PORTC = value;		// set value to output
}
