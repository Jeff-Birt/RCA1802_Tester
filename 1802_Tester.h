#pragma once

// PORT A (inputs to Arduino)
#define ADD_BUS		0xFF	// well, the address bus of course

// PORT B (Outputs from Arduino)
#define CLEAR		0x01	// active low
#define WAIT		0x02	// active Low
#define CLOCK		0x04	// Clock input, active low

// PORT C (bi-directional)
#define DATA_BUS	0xFF	// well, the data bus of course

//PORT G (inputs to Arduino)
#define SC0			0x01	// SC0==0->Fetch, SC0==1->Execute
#define SC1			0x02	// 

// PORT L (inputs to Arduino)
#define N0			0x01	// I/O Control Line 0
#define N1			0x02	// I/O Control Line 1
#define N2			0x04	// I/O Control Line 2
#define TPA			0x08	// Timing Pulse A, MSB of address
#define TPB			0x10	// Timing Pulse B, LSB of address
#define MRD			0x20	// Memory Read, active low
#define MWR			0x40	// Memory Write, active low
#define Q			0x80	// Q flip-flop output

void logState(String note);		// helper to dump current state out to serial port
void portC_ModeInput(void);		// set data bus to HIZ input mode
byte portC_InputValue(void);	// input byte from data bus
void portC_ModeOutput(void);	// set data bus to output mode
void portC_OutputValue(byte value); // output a byte on the data bus