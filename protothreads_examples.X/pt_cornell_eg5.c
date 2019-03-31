/*
 Bruce Land PT eg5
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

// Some helpful macros
#define clrscr() printf("\x1b[2J")
#define home() printf("\x1b[H")
#define pcr() printf("\r")
#define crlf putchar(0x0a); putchar(0x0d);
#define backspace 0x7f

#define max_chars 32 // Input buffer size

// semaphores for controlling threads
static struct pt_sem control_t1, control_t2;
static struct pt pt1, pt2, pt3, pt4, pt_input, pt_output;

// Turn on and off blinking & wait times
static int cntl_blink = 1;
static int wait_t1 = 1000; //ms
static int wait_t2 = 500; //ms
static int run_t4 = 1;

#define PT_YIELD_TIME(delay_time) \
   do { static int time_thread; \
   time_thread = milliSec + delay_time; \
   PT_YIELD_UNTIL(pt, milliSec >= time_thread); \
   } while(0);

// Macros to manipulate semaphores without blocking
//#define PT_SEM_SET(s) (s)->count=1
//#define PT_SEM_CLEAR(s) (s)->count=0
//#define PT_SEM_READ(s) (s)->count

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

// UART2
// Read input from UART
unsigned char term_buffer[max_chars];
int num_char;
int GetSerialBuffer(struct pt *pt)
{
    static unsigned char character;
    PT_BEGIN(pt);
    num_char = 0;
    while (num_char < max_chars)
    {
        // Yield until a character has been received
        PT_YIELD_UNTIL(pt, UARTReceivedDataIsAvailable(UART2));
        character = UARTGetDataByte(UART2);
        PT_YIELD_UNTIL(pt, UARTTransmitterIsReady(UART2));
        UARTSendDataByte(UART2, character);
        if (character == '\r') {
            term_buffer[num_char] = '\0'; //Terminate string
            PT_YIELD_UNTIL(pt, UARTTransmitterIsReady(UART2));
            UARTSendDataByte(UART2, '\n');
            break;
        }
        else if (character == backspace) {
            //putchar(' '); // Write a blank
            //putchar(backspace);
            num_char--;
            if (num_char < 0) num_char = 0;
        }
        else {
            term_buffer[num_char++] = character;
        }
    }
    // Kill this thread to allow spawning thread to execute
    PT_EXIT(pt);
    PT_END(pt); // End thread
}

// Send a string to UART2
unsigned char send_buffer[max_chars];
int num_send_chars;
int PutSerialBuffer(struct pt *pt)
{
    PT_BEGIN(pt);
    num_send_chars = 0;
    while (send_buffer[num_send_chars] != '\0') {
        PT_YIELD_UNTIL(pt, UARTTransmitterIsReady(UART2));
        UARTSendDataByte(UART2, send_buffer[num_send_chars]);
        num_send_chars++;
    }
    PT_EXIT(pt);
    PT_END(pt);
}

// Thread 1
// Wake up every 1s and turn LED off
static PT_THREAD(protothread1(struct pt *pt))
{
    // All vars must be static in pt thread
    // because they are not saved to the stack when a blocking PT call is made
    PT_BEGIN(pt);                       // Start of thread.
    while (1) {
        //stop until thread 2 signals us
        PT_SEM_WAIT(pt, &control_t1);
        mPORTAToggleBits(BIT_0); // Pin 2 
        PT_SEM_SIGNAL(pt, &control_t2);         // Tell thread 2 to go
        PT_YIELD_UNTIL(pt, cntl_blink);         // Thread 3 controls blinking
        PT_YIELD_TIME(wait_t1);                 // Yield for a bit
    }
    PT_END(pt);                         // Should never get here because thread loops for ever
}

// Thread 2
// Wait for signal, turn LED on then signal thread 1
static PT_THREAD(protothread2(struct pt *pt))
{
    PT_BEGIN(pt);
    while (1) {
        // Stop until thread 1 signals us
        PT_SEM_WAIT(pt, &control_t2);
        mPORTAToggleBits(BIT_1);                // Blink LED 2 (Pin 3)
        PT_SEM_SIGNAL(pt, &control_t1);         // Signal thread 1
    }
    PT_END(pt);
}

// Thread 3
// Just doing your own thing
static unsigned char cmd[16];
static int value;
static PT_THREAD(protothread3(struct pt *pt))
{
    PT_BEGIN(pt);
    while (1) { 
        sprintf(send_buffer,"cmd>");            // Send prompt
        PT_SPAWN(pt, &pt_output, PutSerialBuffer(&pt_output));
        PT_SPAWN(pt, &pt_input, GetSerialBuffer(&pt_input));
        
        sscanf(term_buffer, "%s %d", cmd, &value);
        if (cmd[0]=='t' && cmd[1]=='1') wait_t1 = value;
        if (cmd[0]=='t' && cmd[1]=='2') wait_t2 = value;
        if (cmd[0]=='g' && cmd[1]=='1') cntl_blink = 1;
        if (cmd[0]=='s' && cmd[1]=='1') cntl_blink = 0;
        if (cmd[0]=='g' && cmd[1]=='2') run_t4 = 1;
        if (cmd[0]=='s' && cmd[1]=='2') run_t4 = 0;
        if (cmd[0]=='k') PT_EXIT(pt);
        if (cmd[0]=='p') {
            sprintf(send_buffer, "t1=%d t2=%d\n\r", wait_t1, wait_t2);
            PT_SPAWN(pt, &pt_output, PutSerialBuffer(&pt_output));
        }
    }
    PT_END(pt);
}

static PT_THREAD(protothread4(struct pt *pt))
{
    PT_BEGIN(pt);
    while (1) {
        mPORTBToggleBits(BIT_0);
        PT_YIELD_TIME(wait_t2);
    }
    PT_END(pt);
}

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
    printf("pt eg5 started: BAUD=%d\n\r",bit_rate);
    
    // Setup Timer2 as 1ms counter
    // Enable T2, Use internal clock (PBCLK), Prescalar 1:1
    // Count = 40000000/1000 = 40,000 = 1ms
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, PB_FREQ/1000);
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2 | T2_INT_SUB_PRIOR_0);
    mT2ClearIntFlag();
    INTEnableSystemMultiVectoredInt();
    
    // Setup port pins
    // Microstick ii - Pin 2 and 3. And pin 4
    mPORTASetBits(BIT_0 | BIT_1);       // Clear bits to ensure LEDs are off
    mPORTASetPinsDigitalOut(BIT_0 | BIT_1); // Set pins as output
    mPORTBSetBits(BIT_0);               // Clear bits to ensure LED is off
    mPORTBSetPinsDigitalOut(BIT_0);     // Set pin as output
    
    // Init Semophores
    PT_SEM_INIT(&control_t1, 0); // Start t1 blocked
    PT_SEM_INIT(&control_t2, 1); // Start t2 unblocked
    //PT_SEM_INIT(&send_sem, 1); // Start ready to send
    
    // Init threads
    PT_INIT(&pt1);
    PT_INIT(&pt2);
    PT_INIT(&pt3);
    PT_INIT(&pt4);
    
    while (1) {
        PT_SCHEDULE(protothread1(&pt1));
        PT_SCHEDULE(protothread2(&pt2));
        if (run_t4) PT_SCHEDULE(protothread4(&pt4));
        if (cmd[0] != 'k') PT_SCHEDULE(protothread3(&pt3));
    }
}