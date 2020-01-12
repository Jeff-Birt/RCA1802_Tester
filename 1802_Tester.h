#pragma once

// PORT A (inputs to Arduino)
#define ADD_BUS		0xFF	// well, the address bus of course

// PORT B (Outputs from Arduino)
// /CLEAR	/WAIT	MODE (1802)
//	L		L		LOAD
//	L		H		RESET
//	H		L		PAUSE
//	H		H		RUN
#define CLEAR		0x01	// active low
#define WAIT		0x02	// active Low
#define CLOCK		0x04	// Clock input, active low

// PORT C (bi-directional)
#define DATA_BUS	0xFF	// well, the data bus of course

//PORT G (inputs to Arduino)
#define SC0			0x01	// SC0==0->Fetch, SC0==1->Execute
#define SC1			0x02	// 

// PORT K (Outputs from Arduino)
#define EF1			0x01	// EF1 Input
#define EF2			0x02	// EF2 Input
#define EF3			0x04	// EF3 Input
#define EF4			0x08	// EF4 Input
#define DMA_IN		0x10	// DMA Input mode flag
#define DMA_OUT		0x20	// DMA Output mode flag
#define INTERRUPT	0x40	// Interrupt input
//#define OPEN		0x80	// Not used

// PORT L (inputs to Arduino)
#define N0			0x01	// I/O Control Line 0
#define N1			0x02	// I/O Control Line 1
#define N2			0x04	// I/O Control Line 2
#define TPA			0x08	// Timing Pulse A, MSB of address
#define TPB			0x10	// Timing Pulse B, LSB of address
#define MRD			0x20	// Memory Read, active low
#define MWR			0x40	// Memory Write, active low
#define Q			0x80	// Q flip-flop output

#define BTN1 5				// buttons included on shield
#define BTN2 4				// these are Arduino pin numbers
#define BTN3 3
#define BTN4 2

void stateDecode();				// decode control signal state (rising/falling/steady) 
void cmdDecode(String command);	// decodes commands from serial port
void logState(String note);		// helper to dump current state out to serial port
void portC_ModeInput(void);		// set data bus to HIZ input mode
byte portC_InputValue(void);	// input byte from data bus
void portC_ModeOutput(void);	// set data bus to output mode
void portC_OutputValue(byte value); // output a byte on the data bus
String intToHex(int value, int places);	// convert int to hex padded #places

const int buttons[] = { BTN1, BTN2, BTN3, BTN4 };
#define NBUTTONS (sizeof(buttons)/sizeof(int))
int buttonState[NBUTTONS];
boolean isButtonRead[NBUTTONS];
int lastButtonState[NBUTTONS];
int lastDebounceTime[NBUTTONS];