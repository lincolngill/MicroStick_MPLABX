/*
 Bruce Land PT eg2
 */
#define _SUPPRESS_PLIB_WARNING 1
#include <xc.h>                         // Include device specific features
#include <plib.h>                       // Peripheral Library
#include "pt.h"
#include "pt-sem.h"
#include <stdio.h>                      // for printf

// PIC32MX250F128B - Microstick ii
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

#define SYS_FREQ 40000000
#define PB_DIVISOR (1 << OSCCONbits.PBDIV)
#define PB_FREQ SYS_FREQ/PB_DIVISOR     // PBCLK. Also 40MHz
#define PB_MHZ 40                       // PBCLK MHz = 40

#define BAUDRATE 9600

// semaphores for controlling threads
static struct pt_sem control_t1, control_t2;

#define PT_YIELD_TIME(delay_time) \
   do { static int time_thread; \
   time_thread = milliSec + delay_time; \
   PT_YIELD_UNTIL(pt, milliSec >= time_thread); \
   } while(0);

// Timer2 ms timer
volatile int milliSec = 0;              // Volatile because updated in Timer2 ISR
void __ISR(_TIMER_2_VECTOR, IPL2SOFT) Timer2Handler(void)
{
    mT2ClearIntFlag();
    milliSec++;
}

// estimate microSec since start up
// Based on milliSec + value of timer2
long long uSec(void)
{
    return (long long)milliSec * 1000 + (long long)ReadTimer2()/PB_MHZ;
}

// Thread 1
// Wake up every 1s and turn LED off
static PT_THREAD(protothread1(struct pt *pt))
{
    // All vars must be static in pt thread
    // because they are not saved to the stack when a blocking PT call is made
    #define wait_t1 1000                // ms
    PT_BEGIN(pt);                       // Start of thread.
    while (1) {
        //stop until thread 2 signals us
        PT_SEM_WAIT(pt, &control_t1);
        PORTACLR = _PORTA_RA0_MASK;          // LED Off
        printf("P-thread 1 %d %ld\n\r", milliSec, uSec());
        // Signal thread 2
        PT_SEM_SIGNAL(pt, &control_t2);
        // Yield for a bit
        PT_YIELD_TIME(wait_t1);
    }
    PT_END(pt);                         // Should never get here because thread is continuoues
}

// Thread 2
// Wait for signal, turn LED on then signal thread 1
static PT_THREAD(protothread2(struct pt *pt))
{
    PT_BEGIN(pt);
    while (1) {
        // Stop until thread 1 signals us
        PT_SEM_WAIT(pt, &control_t2);
        PORTASET = _PORTA_RA0_MASK;          // LED On
        printf("P-thread 2 %d\n\r", milliSec);
        // Signal thread 1
        PT_SEM_SIGNAL(pt, &control_t1);
    }
    PT_END(pt);
}

// Thread 3
// Just doing your own thing
static PT_THREAD(protothread3(struct pt *pt))
{
    #define wait_t3 4000 //mSec
    PT_BEGIN(pt);
    while (1) {
        printf("P-thread 3 %d\n\r", milliSec);
        // Yield for a bit
        PT_YIELD_TIME(wait_t3);
    }
    PT_END(pt);
}

static struct pt pt1, pt2, pt3;

int main(void)
{
    int bit_rate;
    
    //Configure Flash wait states and prefetch cache for max performance based on SYSCLK
    SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);
    
    ANSELA = 0;                         // Allow Pins Digital
    ANSELB = 0;
    
    // Setup USART IO pins
    PPSInput(2, U2RX, RPB11);           // Pin 22
    PPSOutput(4, RPB10, U2TX);          // Pin 21
    
    UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
    //UARTSetFifoMode() // No Interrupts setup
    UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    bit_rate = UARTSetDataRate(UART2, PB_FREQ, BAUDRATE);
    UARTEnable(UART2, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));  
    printf("pt eg2 started: BAUD=%d\n\r",bit_rate);
    
    // Setuo Timer2 as 1ms counter
    // Enable T2, Use internal clock (PBCLK), Prescalar 1:1
    // Count = 40000000/1000 = 40,000 = 1ms
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, PB_FREQ/1000);
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2 | T2_INT_SUB_PRIOR_0);
    mT2ClearIntFlag();
    INTEnableSystemMultiVectoredInt();
    
    // Port A RA0 Pin2 - Microstick ii onboard LED Pin
    ANSELACLR = _PORTA_RA0_MASK;        // Change RA0 from Analog to Digital (0)
    PORTACLR = _PORTA_RA0_MASK;         // Clear RA0 
    TRISACLR = _PORTA_RA0_MASK;         // Set RA0 to output (0)
    // Init Semophores
    PT_SEM_INIT(&control_t1, 0); // Start t1 blocked
    PT_SEM_INIT(&control_t2, 1); // Start t2 unblocked
    
    // Init threads
    PT_INIT(&pt1);
    PT_INIT(&pt2);
    PT_INIT(&pt3);
    
    while (1) {
        PT_SCHEDULE(protothread1(&pt1));
        PT_SCHEDULE(protothread2(&pt2));
        PT_SCHEDULE(protothread3(&pt3));
    }
}