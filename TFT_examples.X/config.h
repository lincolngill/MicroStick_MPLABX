/*
 * PIC32 config.h
 */

#ifndef SYS_FREQ
#warning SYS_FREQ not defined using 40000000
#define SYS_FREQ 40000000
#endif
#if ! (SYS_FREQ == 40000000 | SYS_FREQ == 64000000)
#error Invalid SYS_FREQ definition. Must be 64000000 or 40000000
#endif

#ifndef _CONFIG_H
#define	_CONFIG_H
#define _SUPPRESS_PLIB_WARNING 1
#include <xc.h>                         // Include device specific features
#include <plib.h>                       // Peripheral Library

// PIC32MX250F128B - Microstick ii
//=============================================================
// 64 MHz
// FRC -8MHz-> FPLLIDIV (2) -4MHz-> FPLLMUL (16) -64MHz-> FPLLODIV (1) --64MHz
// --> SYSCLK -> FPBDIV (2) -32MHz-> PBCLK
//
// 40 MHz
// FRC -8MHz-> FPLLIDIV (2) -4MHz-> FPLLMUL (20) -80MHz-> FPLLODIV (2) --40MHz
// --> SYSCLK -> FPBDIV (1) -40MHz-> PBCLK

// DEVCFG2
// Phase Loop Lock Setup
#pragma config FPLLIDIV = DIV_2         // FRC -8MHz-> FPLLIDIV (2) -4MHz->. Must by 4-5MHz
#if SYS_FREQ == 64000000
// Overclocked
#pragma config FPLLMUL = MUL_16         // -4MHz-> FPLLMUL (16) -64MHz->
#pragma config FPLLODIV = DIV_1         // -64MHz-> FPLLODIV (1) -64MHz->
#elif SYS_FREQ == 40000000
#pragma config FPLLMUL = MUL_20         // -4MHz-> FPLLMUL (20) -80MHz->
#pragma config FPLLODIV = DIV_2         // -64MHz-> FPLLODIV (2) -40MHz->
#else
#error Invalid SYS_FREQ. Must be 64000000 or 40000000
#endif

// DEVCFG1
// Clock
#pragma config FNOSC = FRCPLL           // Clock Source = Internal Fast RC oscillator with PLL
#pragma config POSCMOD = OFF            // Primary oscillator mode = OFF (Not used/connected)
#pragma config FSOSCEN = OFF            // Secondary oscillator disabled
#if SYS_FREQ == 64000000
#pragma config FPBDIV = DIV_2           // SYSCLK -64MHz-> FPBDIV (2) -32MHz-> SBCLK
#define PB_FREQ 32000000
#define PB_MHZ 32
#elif SYS_FREQ == 40000000
#pragma config FPBDIV = DIV_1           // SYSCLK -40MHz-> FPBDIV (1) -40MHz-> SBCLK
#define PB_FREQ 40000000
#define PB_MHZ 40
#else
#error Invalid SYS_FREQ. Must be 64000000 or 40000000
#endif
#pragma config FWDTEN = OFF             // Watchdog timer disabled

// DEVCFG0
#pragma config JTAGEN = OFF             // Joint Test Action Group. JTAG Port disabled
#pragma config PWP = OFF                // Program Flash Write Protect Disabled
#pragma config BWP = OFF                // Boot Flash Write Protect Disabled
#pragma config CP = OFF                 // Code Protection Disabled

#endif	/* CONFIG_H */
