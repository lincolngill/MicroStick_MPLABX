/*
 Example from MPLAB XC32 Users Guide for Embedded Engineers
 
 Turn an LED on
 */

#include <xc.h> // Include device specific features

// PIC32MX250F128B - Microstick ii
// LED is on Pin2 - RA0

//=============================================================
// 40 MHz
// FRC -8MHz-> FPLLIDIV (2) -4MHz-> FPLLMUL (20) -80MHz-> FPLLODIV (2) --40MHz
// --> SYSCLK -> FPBDIV (1) -40MHz-> PBCLK

// DEVCFG2
// Phase Loop Lock Setup
#pragma config FPLLIDIV = DIV_2         // FRC -8MHz-> FPLLIDIV (2) -4MHz->. Must by 4-5MHz
#pragma config FPLLMUL = MUL_20         // -4MHz-> FPLLMUL (20) -80MHz->
#pragma config FPLLODIV = DIV_2         // -80MHz-> FPLLODIV (2) -40MHz->

// DEVCFG1
// Clock
#pragma config FNOSC = FRCPLL           // Clock Source = Internal Fast RC oscillator with PLL
#pragma config POSCMOD = OFF            // Primary oscillator mode = OFF (Not used/connected)
#pragma config FSOSCEN = OFF            // Secondary oscillator disabled
#pragma config FPBDIV = DIV_1           // SYSCLK -40MHz-> FPBDIV (1) -40MHz-> SBCLK
#pragma config FWDTEN = OFF             // Watchdog timer disabled

// DEVCFG0
#pragma config JTAGEN = OFF             // Joint Test Action Group. JTAG Port disabled
#pragma config PWP = OFF                // Program Flash Write Protect Disabled
#pragma config BWP = OFF                // Boot Flash Write Protect Disabled
#pragma config CP = OFF                 // Code Protection Disabled

int main(void) {
    // Port A RA0 Pin2 - Microstick ii onboard LED Pin
    ANSELACLR = _PORTA_RA0_MASK;        // Change RA0 from Analog to Digital (0)
    PORTACLR = _PORTA_RA0_MASK;         // Clear RA0 
    TRISACLR = _PORTA_RA0_MASK;         // Set RA0 to output (0)
    PORTAINV = _PORTA_RA0_MASK;          // Toggle RA0
    PORTAINV = _PORTA_RA0_MASK;          // Toggle RA0
    PORTAINV = _PORTA_RA0_MASK;          // Toggle RA0
    return 0;
}