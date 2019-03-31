/*
 Example from MPLAB XC32 Users Guide for Embedded Engineers
 
 Turn an LED on
 */
#define _SUPPRESS_PLIB_WARNING 1
#include <xc.h>                         // Include device specific features
#include <plib.h>                       // Peripheral Library

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

#define SYSCLK 40000000
#define PBCLK SYSCLK

// Blink at 1Hz
#define TIMER10_RELOAD 500               // Reload value for task1_timer (ms)
// Virtual timer for task1
// Volatile because it it changed in the ISR
static volatile unsigned int timer10 = TIMER10_RELOAD;

// timer5 setup to interrupt every 1ms
void __ISR(_TIMER_5_VECTOR,IPL2SOFT) timer5_handler(void) {
    mT5ClearIntFlag();
    if (timer10 > 0) --timer10;
}

// Toggle the LED pin
void task1(void) {
    timer10 = TIMER10_RELOAD;
    PORTAINV = _PORTA_RA0_MASK;          // Toggle RA0
}

int main(void) {
    //Configure Flash wait states and prefetch cache for max performance based on SYSCLK
    // Exclude changing FPBDIV
    SYSTEMConfig(SYSCLK, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);
    
    //Setup Timer5 - For every 1ms
    OpenTimer5(T5_ON | T5_IDLE_STOP | T5_GATE_OFF | T5_PS_1_1 | T5_SOURCE_INT, PBCLK/1000);
    ConfigIntTimer5(T5_INT_ON | T5_INT_PRIOR_2 | T5_INT_SUB_PRIOR_0); 
    mT5ClearIntFlag();
    
    // Enable interupts
    INTEnableSystemMultiVectoredInt();
    
    // Port A RA0 Pin2 - Microstick ii onboard LED Pin
    ANSELACLR = _PORTA_RA0_MASK;        // Change RA0 from Analog to Digital (0)
    PORTACLR = _PORTA_RA0_MASK;         // Clear RA0 
    TRISACLR = _PORTA_RA0_MASK;         // Set RA0 to output (0)
    
    // Very primitive task scheduler
    while(1) {
        if (timer10 == 0) task1();
    }
}