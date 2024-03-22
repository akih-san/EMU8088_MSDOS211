/*
 * Copyright (c) 2023 @hanyazou
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/*
    PIC18F57Q43 ROM RAM and UART emulation firmware
    This single source file contains all code
    Original source code for PIC18F47Q43 ROM RAM and UART emulation firmware
    Designed by @hanyazou https://twitter.com/hanyazou

    Target: EMU8088 - The computer with only 8088/V20 and PIC18F57Q43
    Written by Akihito Honda
*/

#define BOARD_DEPENDENT_SOURCE

#include "../../src/emu88.h"
#include <stdio.h>
#include "../../drivers/SDCard.h"
#include "../../drivers/picregister.h"

#define SPI_PREFIX      SPI_SD
#define SPI_HW_INST     SPI1
#include "../../drivers/SPI.h"

//#define CLK8M

#define I88_DATA        C
#define I88_ADDR_H      F
#define I88_ADDR_L      D

#define I88_A16		A0
#define I88_A17		A1
#define I88_A18		A2
#define I88_A19		A3

#define I88_IOM		B0
#define I88_ALE		B3

#define I88_RESET	E0
#define I88_HOLD	E1		// BUS request
#define I88_HOLDA	A5

// /WR
#define I88_WR		B1
// /RD
#define I88_RD		B2

// CLK
#define I88_CLK		B7

#define I88_READY	B5
#ifndef I88_READY
#define I88_TEST	B5
#endif

#define I88_NMI		B6
#define I88_INTA	A4
#define I88_INTR	B4

#define SPI_SS		E2
#define SPI_SD_POCI	C2

#define SPI_SD_PICO     C0
#define SPI_SD_CLK      C1
#define SPI_SD_SS       SPI_SS

#define CMD_REQ CLC3OUT

#include "emu88_common.c"

void reset_ioreq(void);

static void emu88_57q_sys_init()
{
    emu88_common_sys_init();

	// NMI definition
	WPU(I88_NMI) = 0;     // NMI Week pull down
	PPS(I88_NMI) = 0;     // set as latch port
	LAT(I88_NMI) = 0;     // NMI=0
	TRIS(I88_NMI) = 0;    // Set as output

#ifdef I88_READY
    LAT(I88_READY) = 1;          // set READY
    TRIS(I88_READY) = 0;         // Set as output
#else
    LAT(I88_TEST) = 0;          // set /TEST
    TRIS(I88_TEST) = 0;         // Set as output
#endif

	// HOLD output pin
    LAT(I88_HOLD) = 1;
    TRIS(I88_HOLD) = 0;          // Set as output
	
	// IO/#M
	WPU(I88_IOM) = 1;     // I88_IOM Week pull up
	LAT(I88_IOM) = 0;     // memory /CE active
	TRIS(I88_IOM) = 0;    // Set as onput

	// ALE
	WPU(I88_ALE) = 0;     // I88_ALE Week pull down
    TRIS(I88_ALE) = 1;    // Set as input

	// /WR output pin
	WPU(I88_WR) = 1;		// /WR Week pull up
    LAT(I88_WR) = 1;		// disactive
    TRIS(I88_WR) = 0;		// Set as output
    PPS(I88_WR) = 0x00;	//reset:output PPS is LATCH

	// /RD output pin
	WPU(I88_RD) = 1;		// /WR Week pull up
    LAT(I88_RD) = 1;		// disactive
    TRIS(I88_RD) = 0;		// Set as output
    PPS(I88_RD) = 0x00;	//reset:output PPS is LATCH

	// SPI_SS
	WPU(SPI_SS) = 1;     // SPI_SS Week pull up
	LAT(SPI_SS) = 1;     // set SPI disable
	TRIS(SPI_SS) = 0;    // Set as onput

	LAT(I88_CLK) = 1;	// 8088_CLK = 1
    TRIS(I88_CLK) = 0;	// set as output pin
	

// Setup CLC
//
	//========== CLC pin assign ===========
    CLCIN2PPS = 0x0b;			// assign RB3(ALE)
    CLCIN3PPS = 0x08;			// assign RB0(IO/M#)
	
	//========== CLC3 : IOREQ ==========
	// reset DFF with software(reset_ioreq();)

	CLCSELECT = 2;		// CLC3 select

	CLCnSEL0 = 2;		// CLCIN2PPS : RB3(ALE)
    CLCnSEL1 = 3;		// CLCIN3PPS : RB0(IO/M#)
	CLCnSEL2 = 0x35;	// CLC3OUT
	CLCnSEL3 = 127;		// NC

    CLCnGLS0 = 0x02;	// ALE -> (log1) DFF(CK)
	CLCnGLS1 = 0x08;	// IO/#M - > (log2) OR gate
    CLCnGLS2 = 0x00;	// not gated
    CLCnGLS3 = 0x20;	// CLC3OUT -> (log4) OR gate

    CLCnPOL = 0x00;		// POL=0
    CLCnCON = 0x85;		// 2-Input DFF with R , no interrupt occurs

	// reset CLC3
	reset_ioreq();

	// SPI data and clock pins slew at maximum rate

	SLRCON(SPI_SD_PICO) = 0;
	SLRCON(SPI_SD_CLK) = 0;
	SLRCON(SPI_SD_POCI) = 0;

/*********** CLOCK TIMING ************************
 I88_CLK TIMING REQUIREMENTS(MUST)
 CLK Low Time  : minimum 118ns
 CLK High Time : minimum 69ns
*************************************************/

/**************** PWM3 ********************
// 8088 TIMING REQUIREMENTS(MUST)
// CLK Low Time  : minimum 118ns
// CLK High Time : minimum 69ns
// CLK duty 33%

// P64 = 1/64MHz = 15.625ns
// P5  = 1/5MHz  = 200ns = P64 * 12.8
// P8  = 1/8MHz  = 125nz = P64 * 8
//
// --- 4.92MHz ---
// Set PWM Left Aligned mode
// PR = 12
// P1 = 5 : P64*5 = 78.125ns
// P2 = 8 : P64*8 = 125ns
// MODE = 0
//     high period time: 78.125ns
//     low period time: 125ns
//     78.125 + 125ns = 203.125ns f = 4923076.9230769230769230769230769 Hz
//     duty = 38.4%
// --- 8MHz ---
// Set PWM Left Aligned mode
// PR = 7
// P1 = 3 : P64*3 = 46.875ns
// P2 = 5 : P64*5 = 78.125ns
// MODE = 0
//     high period time: 46.875ns
//     low period time: 78.125ns
//     46.875ns + 78.125 = 125ns f = 8 Hz
//     duty = 37.5%
******************************************/

	PWM3CLK = 0x02;		// Fsoc
	PWM3GIE = 0x00;		// interrupt disable
#ifdef CLK8M
//8MHz
	PWM3PR = 0x0007;	// 8 periods ( 0 - 7 )
	PWM3S1P1 = 0x0005;	// P1 = 3
	PWM3S1P2 = 0x0008;	// P2 = 5
#else
//5MHz
	PWM3PR = 0x000C;	// 13 periods ( 0 - 12 )
	PWM3S1P1 = 0x0005;	// P1 = 5
	PWM3S1P2 = 0x0008;	// P2 = 8
#endif
	PWM3S1CFG = 0x00;	// (POL1, POL2)= 0, PPEN = 0 MODE = 0 (Left Aligned mode)
	PWM3CON = 0x84;		// EN=1, LD=1
	RB7PPS = 0x1C;		// PWM3S1P1_OUT

//************ timer0 setup ******************
	T0CON0 = 0x90;	// timer enable, 16bit counter mode , 1:1 Postscaler
	T0CON1 = 0x80;	// sorce clk:LFINTOSC, 1:1 Prescaler
	LFOEN = 1;		// LFINTOSC is explicitly enabled

	TMR0L = TIMER0_INITCL;
	TMR0H = TIMER0_INITCH;		//	timer counter set to 0x86e8
								//  LFINTOSC = 31Khz!!
    emu88_common_wait_for_programmer();

    //
    // Initialize SD Card
    //
    static int retry;
    for (retry = 0; 1; retry++) {
        if (20 <= retry) {
            printf("No SD Card?\n\r");
            while(1);
        }
//        if (SDCard_init(SPI_CLOCK_100KHZ, SPI_CLOCK_2MHZ, /* timeout */ 100) == SDCARD_SUCCESS)
        if (SDCard_init(SPI_CLOCK_100KHZ, SPI_CLOCK_4MHZ, /* timeout */ 100) == SDCARD_SUCCESS)
//        if (SDCard_init(SPI_CLOCK_100KHZ, SPI_CLOCK_8MHZ, /* timeout */ 100) == SDCARD_SUCCESS)
            break;
        __delay_ms(200);
    }

	GIE = 1;             // Global interrupt enable

}

static void emu88_57q_start_i88(void)
{

    emu88_common_start_i88();

	TMR0IF =0; // Clear timer0 interrupt flag
	TMR0IE = 1;	// Enable timer0 interrupt

	// Unlock IVT
    IVTLOCK = 0x55;
    IVTLOCK = 0xAA;
    IVTLOCKbits.IVTLOCKED = 0x00;

    // Default IVT base address
    IVTBASE = 0x000008;

    // Lock IVT
    IVTLOCK = 0x55;
    IVTLOCK = 0xAA;
    IVTLOCKbits.IVTLOCKED = 0x01;

	TRIS(I88_HOLDA) = 1;    // HOLDA is set as input
	LAT(I88_HOLD) = 0;		// Release HOLD
	// I88 start
    LAT(I88_RESET) = 0;		// Release reset

}

void reset_ioreq(void)
{
	// Release wait (D-FF reset)
	G3POL = 1;
	G3POL = 0;
}

void set_hold_pin(void)
{
	LAT(I88_HOLD) = 1;
	while( !R(I88_HOLDA) ) {}		// wait until bus release
}

void reset_hold_pin(void)
{
	LAT(I88_HOLD) = 0;
	while( R(I88_HOLDA) ) {}		// wait until bus release
}

void nmi_sig_off(void)
{
	LAT(I88_NMI) = 0;
}

void nmi_sig_on(void)
{
	LAT(I88_NMI) = 1;
}

static void bus_hold_req(void) {
	// Set address bus as output
	TRIS(I88_ADDR_L) = 0x00;	// A7-A0
	TRIS(I88_ADDR_H) = 0x00;	// A8-A15
	TRIS(I88_A16) = 0;			// Set as output
	TRIS(I88_A17) = 0;			// Set as output
	TRIS(I88_A18) = 0;			// Set as output
	TRIS(I88_A19) = 0;			// Set as output

	TRIS(I88_RD) = 0;           // output
	TRIS(I88_WR) = 0;           // output
	// SRAM U4, U5 are HiZ

	TRIS(I88_IOM) = 0;    // Set as output

}

static void bus_release_req(void) {
	// Set address bus as input
	TRIS(I88_ADDR_L) = 0xff;    // A7-A0
	TRIS(I88_ADDR_H) = 0xff;    // A8-A15
	TRIS(I88_A16) = 1;    // Set as input
	TRIS(I88_A17) = 1;    // Set as input
	TRIS(I88_A18) = 1;    // Set as input
	TRIS(I88_A19) = 1;    // Set as input

	// Set /RD and /WR as input
	TRIS(I88_RD) = 1;           // input
	TRIS(I88_WR) = 1;           // input
	TRIS(I88_IOM) = 1;          // input
}

//--------------------------------
// event loop ( PIC MAIN LOOP )
//--------------------------------
void board_event_loop(void) {

	while(1) {
		if (CMD_REQ) {					// CLC3OUT =1
			set_hold_pin();				// HOLD = 1, wait until HOLDA = 1
		    bus_hold_req();				// PIC becomes a busmaster
			bus_master_operation();
		    bus_release_req();
			reset_ioreq();				// reset CLC3 (CMD_REQ : CLC3OUT = 0)
			reset_hold_pin();			// HOLD = 0, wait until HOLDA = 0
		}
		if (nmi_flg) nmi_sig_on();
	}
}

void board_init()
{
    emu88_common_init();

    board_sys_init_hook = emu88_57q_sys_init;
    board_start_i88_hook = emu88_57q_start_i88;
}


#include "../../drivers/pic18f57q43_spi.c"
#include "../../drivers/SDCard.c"

