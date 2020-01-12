/*
 Simple test program for RCA 1802 microprocessor
 Jeffrey T. Birt (Hey Birt!)
 website: http://www.soigeneris.com
 YouTube: http://www.youtube.com/c/HeyBirt
*/
// NOTES: Uses a Arduino Mega 2560
// control /CLR and /WAIT 
// what should the settings for debug view levels be?
// A) full instruction cycle
// B) every half clock
// C) memory read/write
// D) I/O input/output

#include "1802_Tester.h"

#define BTN1 5
#define BTN2 4
#define BTN3 3
#define BTN4 2

const int buttons[] = {BTN1, BTN2, BTN3, BTN4};
#define NBUTTONS (sizeof(buttons)/sizeof(int))
int buttonState[NBUTTONS];
boolean isButtonRead[NBUTTONS];

int lastButtonState[NBUTTONS];
int lastDebounceTime[NBUTTONS];
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

// Arduino system states
enum SYS_STATE
{
	SYS_INIT,		// Initialized I/O, do start delay
	SYS_RESET,		// RESET mode: /CLEAR  WAIT
	SYS_PAUSE,		// PAUSE mode:  CLEAR /WAIT
	SYS_RUN,		// RUN mode:    CLEAR  WAIT
	SYS_STEP,		// Process one opcocde then STOP mode
	SYS_STOP		// stock clock
};

// States the control singnals can be in
enum SIGNAL_STATE
{
	SIG_LOW,
	SIG_RISING,
	SIG_HIGH,
	SIG_FALLING
};

enum DEBUG_STATE
{
	DB_OFF,
	DB_MININUM,
	DB_VERBOSE
};

#pragma region "1802 Test Programs"

// Memory store --- Load D with 0x55 store to Address pointed to by R(1)
// virtual system RAM, 0x0000 to 0x0039 ->
//byte virtRAM[] =  { 0xF8, 0x0F, 0xA1, 0xF8, 0x00, 0xB1, 0xE1, 0xF8,
//					0x55, 0x51, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4,
//					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4,
//					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4,
//					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4,
//					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4,
//					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4,
//					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4 };


// I/O Output --- Set R(1)=0x000F, Set X=R(1), Mem @ R(1) output to data bus
// N0, N1, N2 indicate the lower nibble of the 6N Output instruction
// virtual system RAM, 0x0000 to 0x0039 -> 
//byte virtRAM[] =  { 0xF8, 0x0F, 0xA1, 0xF8, 0x00, 0xB1, 0xE1, 0x61,
//					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0x55,
//					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4,
//					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4,
//					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4,
//					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4,
//					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4,
//					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4 };


// I/O Input --- Set R(1)=0x000F, Set X=R(1), Input from data bus to Mem @ R(1)
// N0, N1, N2 indicate the lower nibble of the 6N Output instruction
// virtual system RAM, 0x0000 to 0x0039 -> 
//byte virtRAM[] =  { 0xF8, 0x0F, 0xA1, 0xF8, 0x00, 0xB1, 0xE1, 0x69,
//					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0x55,
//					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4,
//					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4,
//					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4,
//					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4,
//					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4,
//					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4 };


// EFx input test -- output EF input # to Memory @ 0x22
// virtual system RAM, 0x0000 to 0x0039 -> 
byte virtRAM[] = {  0xF8, 0x22, 0xA1, 0xF8, 0x00, 0xB1, 0xE1, 0x34,
					0x11, 0x35, 0x15, 0x36, 0x19, 0x37, 0x1D, 0x30,
					0x07, 0xF8, 0x01, 0x30, 0x1F, 0xF8, 0x02, 0x30,
					0x1F, 0xF8, 0x03, 0x30, 0x1F, 0xF8, 0x04, 0x51,
					0x30, 0x07, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4,
					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4,
					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4,
					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4 };

// Q blink --- Q off, load 0x0010 to R(1)
// virtual system RAM, 0x0000 to 0x0039 -> 
//byte virtRAM[] = {	0x7A, 0xF8, 0x10, 0xA1, 0xF8, 0x00, 0xB1, 0x21,
//					0x81, 0x3A, 0x07, 0x31, 0x00, 0x7B, 0x30, 0x01,
//					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 
//					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 
//					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 
//					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 
//					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 
//					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4 };

// NOP test - Simple NOP test, address should increase
// virtual system RAM, 0x0000 to 0x0039 -> 
//byte virtRAM[] = {	0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4,
//					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4,
//					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 
//					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 
//					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 
//					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 
//					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 
//					0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4, 0xC4 };

#pragma endregion "1802 Test Programs"

// gloabls
int Start_Delay = 0;		// delay reset release 64 half clocks after start
unsigned int Address16 = 0; // used to latch in full 16-bit address
unsigned int clkCount = 0;	// counts in 1/2 clock cycles
SYS_STATE sysState = SYS_INIT;
SIGNAL_STATE MRD_State = SIG_LOW;
SIGNAL_STATE MWR_State = SIG_LOW;
SIGNAL_STATE TPA_State = SIG_LOW;
SIGNAL_STATE TPB_State = SIG_LOW;
DEBUG_STATE debugState = DB_MININUM;

byte newPORTA = 0; byte oldPORTA = 0;	// not sure if oldPORTA actually needed
byte newPORTL = 0; byte oldPORTL = 0;	// track old state, compare to new for rising/falling edges
byte newSCx = 0; 						// new state of SC0 adn SC1

void readButton(int bNo) {
	int reading = digitalRead(buttons[bNo]);
	// If the switch changed, due to noise or pressing:
	if (reading != lastButtonState[bNo]) {
		// reset the debouncing timer
		lastDebounceTime[bNo] = millis();
	}

	if ((millis() - lastDebounceTime[bNo]) > debounceDelay) {
		// whatever the reading is at, it's been there for longer than the debounce
		// delay, so take it as the actual current state:

		// if the button state has changed:
		if (reading != buttonState[bNo]) {
			buttonState[bNo] = reading;
			isButtonRead[bNo] = false;
		}
	}
	lastButtonState[bNo] = reading;
}

void readButtons() {
	for (int i=0; i<NBUTTONS; i++) {
		readButton(i);
	}
}

void initButtons() {
	for (int i=0; i<NBUTTONS; i++) {
		pinMode(buttons[i], INPUT_PULLUP);
		buttonState[i] = HIGH;
		lastButtonState[i] = HIGH;
		isButtonRead[i] = true;
	}
}

boolean hasButtonPressEvent(int bNo) {
	if (buttonState[bNo] == LOW && !isButtonRead[bNo]) {
		isButtonRead[bNo] = true;
		return true;
	} else {
		return false;
	}
}

// Initalize the Arduino I/O ports, start serial coms
// Leave processer in RESET mode: /CLEAR WAIT
void setup()
{
	PORTA = PORTA & ~ADD_BUS;	  // all pins low for HIZ
	PORTB = ( PORTB & ~(CLEAR | CLOCK) ) | WAIT; // set to reset mode
	PORTC = PORTC & ~DATA_BUS;	  // all pins low for HIZ
	PORTG = PORTG & ~(SC0 | SC1); // all pins low for HIZ
	PORTK = PORTK | (EF1 | EF2 | EF3 | EF4 | DMA_IN | DMA_OUT | INTERRUPT); // all pins HI/off
	PORTL = PORTL & ~(N0 | N1 | N2 | TPA | TPB | MRD | MWR | Q); // all pins low for HIZ

	DDRA = ~ADD_BUS;	// all inputs, HIZ at this point
	DDRB = CLEAR | WAIT | CLOCK; // outputs
	DDRC = ~DATA_BUS;	// all inputs, HIZ at this point
	DDRG = DDRG & ~(SC0 | SC1);  // inputs, HIZ at this point
	DDRK = EF1 | EF2 | EF3 | EF4 | DMA_IN | DMA_OUT | INTERRUPT; // outputs, all HI at this point
	DDRL = ~(N0 | N1 | N2 | TPA | TPB | MRD | MWR | Q); // all inputs, HIZ at this point

	initButtons();

	Serial.begin(19200); // open the serial port at 9600 bps:
	Serial.println("Starting...");
}

void randomize_ef() {
	int rval = random(0, 5), ef;
	if (rval == 4) {
		ef = 0;
		Serial.println("No EF pin set.");
	} else {
		ef=1 << rval;
		Serial.println("EF pin " + String(rval + 1) + " set.");
	}
	PORTK = (PORTK & ~(EF1 | EF2 | EF3 | EF4)) | ~ef;
}


// Start up in RESET mode, start clock, after 64 clocks put in PAUSE mode
// Clock count only incremetned in RUN mode.
void loop()
{
	readButtons();
	// watch for input from user here, using a blocking read for now
	if (Serial.available())
	{
		String command = Serial.readStringUntil('\n');
		cmdDecode(command);
	}

	if (sysState == SYS_RUN)
	{
		if (hasButtonPressEvent(0)) {
			randomize_ef();
		}
    
		// read in current state of I/O
		newPORTA = PINA;			 // 1802 address bus
		newPORTL = PINL;			 // 1802 status outputs
		newSCx = PING & (SC0 | SC1); // 1802 state outputs

		// (A) Decode state of control signals
		stateDecode();

		// (B) Address latch, rising edge TPA latch MSB, rising edge TPB latch LSB
		if (TPA_State == SIG_RISING)
		{
			Address16 = newPORTA;
			//logState("TPA");
		} 
		else if (TPB_State == SIG_RISING)
		{
			Address16 = (Address16 << 8) | newPORTA;
			//logState("TPB");
		}

		// (C) if rising edge of /MRD or /MWR then set data bus to input from 1802
		if ((MRD_State == SIG_RISING) | (MWR_State == SIG_RISING))
		{
			portC_ModeInput();
			logState("Data bus Input");
		}

		// (D) TPB rising edge, reading (Arduino to 1802), or writing (1802 to Arduino)
		// if /MRD low Arduino writing to data bus, 1802 reading from memory
		// Arduino reading data from data bus, i.e. 1802 is writing to virtual memory
		if (TPB_State == SIG_RISING)
		{
			if (MRD_State == SIG_LOW)
			{
				portC_ModeOutput();

				if (Address16 < 64)
				{
					portC_OutputValue(virtRAM[Address16]);
					logState("1802 Memory Read");
				}
				else
				{
					portC_OutputValue(0xC4); // NOP
					logState("NOP");
				}
			}
			else if (MWR_State == SIG_LOW)
			{
				byte fromDataBus = portC_InputValue();
				if (Address16 < 64)
				{
					virtRAM[Address16] = fromDataBus;
				}
				logState(String(fromDataBus, HEX) + " 1802 Memory Write");
			}
		}

		// (E) reading or writing to I/O port if N0~N2 raised and
		// /MRW falling edge indicates input from Arduino to 1802, 
		// /MRD and TPB rising edge indicates output from 1802 to Arduino
		if (newPORTL & (N0 | N1 | N2))
		{
			if (MWR_State == SIG_FALLING)
			{
				portC_ModeOutput();
				portC_OutputValue(0xAA); // fixed value at this point
				logState(" I/O Input");
			}
			else if (MRD_State == SIG_LOW && TPB_State == SIG_RISING)
			{
				byte fromDataBus = portC_InputValue();
				logState(String(fromDataBus, HEX) + " I/O Output");
			}
		}

		// (F) Is Q rising or falling?
		if ((newPORTL & Q) && !(oldPORTL & Q))
		{
			logState("Q turned on");
		}
		else if (!(newPORTL & Q) && (oldPORTL & Q))
		{
			logState("Q turned off");
		}

		delay(100);				// delay for serial debug display
		clkCount++;				// inc clock 1/2 tick
		oldPORTA = newPORTA;	// not sure I need to save address bus
		oldPORTL = newPORTL;	// save new values for comparison next cycle
	}
	else if (sysState == SYS_INIT)
	{
		Start_Delay++;
		if (Start_Delay > 63)
		{
			//PORTB = PORTB | CLEAR;  // PAUSE mode:  CLEAR /WAIT
			//PORTB = PORTB & ~WAIT;	//
			//sysState = SYS_PAUSE;
			//Serial.println("");
			//Serial.println("Pause Mode");
			cmdDecode("pause");
		}
	}

	if (sysState != SYS_STOP)
	{
		PORTB = PORTB ^ CLOCK; // toggle clock 1/2 tick
	}
}

// Decodes state (rising/falling/steady) of control signals
void stateDecode()
{
	if (newPORTL & TPA)
	{
		TPA_State = (!(oldPORTL & TPA)) ? SIG_RISING : SIG_HIGH;
	}
	else
	{
		TPA_State = (oldPORTL & TPA) ? SIG_FALLING : SIG_LOW;
	}

	if (newPORTL & TPB)
	{
		TPB_State = (!(oldPORTL & TPB)) ? SIG_RISING : SIG_HIGH;
	}
	else
	{
		TPB_State = (oldPORTL & TPB) ? SIG_FALLING : SIG_LOW;
	}

	if (newPORTL & MRD)
	{
		MRD_State = (!(oldPORTL & MRD)) ? SIG_RISING : SIG_HIGH;
		//logState("MRD " + String((MRD_State == SIG_RISING)?"RISING":"HIGH"));
	}
	else
	{
		MRD_State = (oldPORTL & MRD) ? SIG_FALLING : SIG_LOW;
		//logState("MRD " + String((MRD_State == SIG_FALLING)?"FALLING":"LOW"));
	}

	if (newPORTL & MWR)
	{
		MWR_State = (!(oldPORTL & MWR)) ? SIG_RISING : SIG_HIGH;
		//logState("MWR " + String((MWR_State == SIG_RISING)?"RISING":"HIGH"));
	}
	else
	{
		MWR_State = (oldPORTL & MWR) ? SIG_FALLING : SIG_LOW;
		//logState("MWR " + String((MWR_State == SIG_FALLING)?"FALLING":"LOW"));
	}
}

// wonder if this should be used for changing states as well?
void cmdDecode(String command)
{
	if (command == "load")
	{
		// Control bits: /CLEAR /WAIT
		Serial.println();
		Serial.println("Load Mode");
	}
	else if (command == "reset")
	{
		PORTB = PORTB & ~CLEAR; // RESET mode:  /CLEAR WAIT
		PORTB = PORTB | WAIT;	//
		sysState = SYS_RESET;

		Serial.println();
		Serial.println("Reset Mode");
	}
	else if (command == "pause")
	{
		PORTB = PORTB | CLEAR;  // PAUSE mode:  CLEAR /WAIT
		PORTB = PORTB & ~WAIT;	//
		sysState = SYS_PAUSE;
		Serial.println();
		Serial.println("Pause Mode");
	}
	else if (command == "run")
	{
		// start clock so processor and/or set to RUN mode
		PORTB = PORTB | CLEAR;  // RUN mode:  CLEAR WAIT
		PORTB = PORTB | WAIT;	//
		sysState = SYS_RUN;

		Serial.println();
		Serial.println("Run mode");
		Serial.println("Clk \tSCx \tCW_CL \tPORTL \tAdd \tData \tNote");
	}
	else if (command == "stop")
	{
		// stop clock so processor stops
		sysState = SYS_STOP;
		Serial.println();
		Serial.println("System is Stopped");
	}
	else if (command == "dump")
	{
		// stop processor and dump virtual memory
		Serial.println();
		Serial.println("Memory dump 0x0000 to 0x0039");
		sysState = SYS_STOP;
		for (int row = 0; row < 8; row++)
		{
			for (int col = 0; col < 8; col++)
			{
				Serial.print(intToHex(virtRAM[row * 8 + col], 2));
				Serial.print(" ");
			}
			Serial.println();
		}
		Serial.println();
		Serial.println("System is Stopped");
		Serial.println();
	}
	else if (command == "status")
	{
		// stop processor and show current processor status
		// maybe combine with stop command
		Serial.println();
		Serial.println("Current Status");
		Serial.println("Clk \tSCx \tCW_CL \tPORTL \tAdd \tData \tNote");
		logState("status");
		sysState = SYS_STOP;
	}
	else if (command == "help")
	{
		Serial.println();
		Serial.println("List of commands");
		Serial.println("load   - not supported");
		Serial.println("reset  - processor in RESET mode");
		Serial.println("pause  - processor in PAUSE mode");
		Serial.println("run    - processor in RUN mode");
		Serial.println("stop   - stop clock");
		Serial.println("dump   - dump vertual memory");
		Serial.println("status - display status, stop");
	}

}

// helper to dump current state out to serial port
void logState(String note)
{
	//Serial.println("Clk \tSCx \tCW_CL \tPORTL \tAdd \tData \tNote");
	if (debugState != DB_OFF)
	{
		Serial.print(intToHex(clkCount, 4)); Serial.print("\t");
		Serial.print(newSCx); Serial.print("\t");
		Serial.print(intToHex(PORTC & (CLEAR | WAIT | CLOCK), 2)); Serial.print("\t");
		Serial.print(intToHex(newPORTL, 2)); Serial.print("\t");
		Serial.print(intToHex(Address16, 4)); Serial.print("\t");
		Serial.print(intToHex(PINC, 2)); Serial.print("\t");
		Serial.println(note);
	}
}

#pragma region "Data Bus PORTC Helpers"

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

#pragma endregion "Data Bus PORTC Helpers"

// Returns string representation of int in HEX with leading zeros
// value: int to convert, places: number of digits padded with leading zeros
String intToHex(int value, int places)
{
	String hexString = "000"+ String(value, HEX);
	hexString.toUpperCase();	// make sure all upper case
	int length = hexString.length();
	if ( length > places) { hexString.remove(0, length - places); }
	return hexString;  // make sure only 2 chars long
}