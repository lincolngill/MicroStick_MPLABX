/*
 Bruce Land PT eg7 - DMA
 */
#define _SUPPRESS_PLIB_WARNING 1
#include <xc.h>                         // Include device specific features
#include <plib.h>                       // Peripheral Library
#include "pt.h"
#include "pt-sem.h"
#include <stdio.h>                      // for printf

// PIC32MX250F128B - Microstick ii
//=============================================================
// 64 MHz
// FRC -8MHz-> FPLLIDIV (2) -4MHz-> FPLLMUL (16) -64MHz-> FPLLODIV (1) --64MHz
// --> SYSCLK -> FPBDIV (2) -32MHz-> PBCLK

// DEVCFG2
// Phase Loop Lock Setup
#pragma config FPLLIDIV = DIV_2         // FRC -8MHz-> FPLLIDIV (2) -4MHz->. Must by 4-5MHz
#pragma config FPLLMUL = MUL_16         // -4MHz-> FPLLMUL (16) -64MHz->
#pragma config FPLLODIV = DIV_1         // -64MHz-> FPLLODIV (1) -64MHz->

// DEVCFG1
// Clock
#pragma config FNOSC = FRCPLL           // Clock Source = Internal Fast RC oscillator with PLL
#pragma config POSCMOD = OFF            // Primary oscillator mode = OFF (Not used/connected)
#pragma config FSOSCEN = OFF            // Secondary oscillator disabled
#pragma config FPBDIV = DIV_2           // SYSCLK -64MHz-> FPBDIV (2) -32MHz-> SBCLK
#pragma config FWDTEN = OFF             // Watchdog timer disabled

// DEVCFG0
#pragma config JTAGEN = OFF             // Joint Test Action Group. JTAG Port disabled
#pragma config PWP = OFF                // Program Flash Write Protect Disabled
#pragma config BWP = OFF                // Boot Flash Write Protect Disabled
#pragma config CP = OFF                 // Code Protection Disabled

#define SYS_FREQ 64000000
#define PB_DIVISOR (1 << OSCCONbits.PBDIV)
#define PB_FREQ SYS_FREQ/PB_DIVISOR     // PBCLK. 32MHz

#define BAUDRATE 9600

// Some helpful macros
#define clrscr() printf("\x1b[2J")
#define home() printf("\x1b[H")
#define pcr() printf("\r")
#define crlf putchar(0x0a); putchar(0x0d);
#define backspace 0x7f

#define max_chars 64 // Input buffer size

// semaphores for controlling threads
static struct pt_sem control_t1, control_t2;
static struct pt pt1, pt2, pt3, pt4, pt_input, pt_DMA_output;

// Turn on and off blinking & wait times
static int cntl_blink = 1;
static int wait_t1 = 1000; //ms
static int wait_t2 = 64000; //us
static int run_t4 = 1;

#define PT_YIELD_TIME_msec(delay_time) \
   do { static int time_thread; \
   time_thread = milliSec + delay_time; \
   PT_YIELD_UNTIL(pt, milliSec >= time_thread); \
   } while(0);

// Macro for usec wait
// delay_time < 64000
// Reads timer3
#define PT_YIELD_TIME_usec(delay_time) \
   do { static unsigned int time_thread, T3, c; \
   T3 = ReadTimer3(); \
   time_thread = T3 + delay_time; \
   c = 0; \
   if (time_thread >= 0xffff) { c = 0xffff-T3; } \
   PT_YIELD_UNTIL(pt, ((ReadTimer3()+c) & 0xffff) >= ((time_thread+c) & 0xffff)); \
   } while (0);

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
    return (long long)milliSec * 1000 + (long long)ReadTimer2();
}

// UART2
// Read input from UART
unsigned char term_buffer[max_chars];
int GetSerialBuffer(struct pt *pt)
{
    static unsigned char character;
    static int num_char;
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

// DMA send string
// Send a string to UART2
unsigned char send_buffer[max_chars];
static int dma_cnt = 0; // count the number of time the DMA channel enable is called.
int DMA_PutSerialBuffer(struct pt *pt)
{
    PT_BEGIN(pt);
    // Need to wait until the UART FIFO buffer is completely empty.
    // Otherwise interrupts for inprogress FIFO bytes cause DMA to overrun FIFO buffers
    PT_YIELD_UNTIL(pt, UARTTransmissionHasCompleted(UART2));
    DmaChnEnable(DMA_CHANNEL1);                  // Enabling channel causes transfer to start
    dma_cnt++; 
    PT_YIELD_UNTIL(pt, DmaChnGetEvFlags(DMA_CHANNEL1) & DMA_EV_BLOCK_DONE); // Wait until DMA block has been done
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
        PT_YIELD_TIME_msec(wait_t1);                 // Yield for a bit
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
        PT_SPAWN(pt, &pt_DMA_output, DMA_PutSerialBuffer(&pt_DMA_output));
        PT_SPAWN(pt, &pt_input, GetSerialBuffer(&pt_input));
        
        cmd[0] = '\0';          // If term_buffer is empty then sscanf doesn't change cmd or value
        sscanf(term_buffer, "%s %d", cmd, &value);
        if (cmd[0]=='t' && cmd[1]=='1') wait_t1 = value;
        if (cmd[0]=='t' && cmd[1]=='2') wait_t2 = value;
        if (cmd[0]=='g' && cmd[1]=='1') cntl_blink = 1;
        if (cmd[0]=='s' && cmd[1]=='1') cntl_blink = 0;
        if (cmd[0]=='g' && cmd[1]=='2') run_t4 = 1;
        if (cmd[0]=='s' && cmd[1]=='2') run_t4 = 0;
        if (cmd[0]=='k') PT_EXIT(pt);
        if (cmd[0]=='p') {
            sprintf(send_buffer, "LED1Timer=%d LED2Timer=%d dma_cnt=%d\n\r", wait_t1, wait_t2, dma_cnt);
            PT_SPAWN(pt, &pt_DMA_output, DMA_PutSerialBuffer(&pt_DMA_output));
        }
    }
    PT_END(pt);
}

static PT_THREAD(protothread4(struct pt *pt))
{
    PT_BEGIN(pt);
    while (1) {
        mPORTBToggleBits(BIT_0);
        PT_YIELD_TIME_usec(wait_t2);
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
    printf("\n\rpt eg7 started: BAUD=%d\n\r",bit_rate);
    
    // Setup Timer2 as 1ms counter
    // Enable T2, Use internal clock (PBCLK), Prescalar 1:32
    // Tick = 32000000/32 = 1000000 (1usec)
    // 1000 counts = 1ms
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_32, 1000);
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2 | T2_INT_SUB_PRIOR_0);
    mT2ClearIntFlag();
    
    // Timer3
    // Tick = 32000000/32 (postscalar) = 1000000 (1 usec)
    // Just overflow. no interrupt. Value read for PT_YIELD_TIME_usec
    OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_32, 0xffff);
    
    // Setup port pins
    // Microstick ii - Pin 2 and 3. And pin 4
    mPORTASetBits(BIT_0 | BIT_1);       // Clear bits to ensure LEDs are off
    mPORTASetPinsDigitalOut(BIT_0 | BIT_1); // Set pins as output
    mPORTBSetBits(BIT_0);               // Clear bits to ensure LED is off
    mPORTBSetPinsDigitalOut(BIT_0);     // Set pin as output
    
    // DMA Setup
    DmaChnOpen(DMA_CHANNEL1, DMA_CHN_PRI2, DMA_OPEN_MATCH);
    // UTXISEL<1:0> = 0b00 - UART2 Interrupt generated when TX buffer contains at least one space
    // Interrupt triggers cell transfer (I.e. 1 byte)
    DmaChnSetEventControl(DMA_CHANNEL1, DMA_EV_START_IRQ_EN | 
                                        DMA_EV_MATCH_EN | 
                                        DMA_EV_START_IRQ(_UART2_TX_IRQ));
    // vSrcAdd = send_buffer (&send_buffer[0])
    // vDstAdd = UART2 TX Reg (cast to void pointer as function requires)
    // srcSize = max_chars (send_buffer[0..31])
    // dstSize = 1 (TX reg)
    // cellSize = 1 (byte at a time)
    DmaChnSetTxfer(DMA_CHANNEL1, send_buffer, (void*)&U2TXREG, max_chars, 1, 1);
    DmaChnSetEvEnableFlags(DMA_CHANNEL1, DMA_EV_BLOCK_DONE); // Signal when block done
    DmaChnSetMatchPattern(DMA_CHANNEL1, '\0');
    
    INTEnableSystemMultiVectoredInt();
    
    // Init Semophores
    PT_SEM_INIT(&control_t1, 0); // Start t1 blocked
    PT_SEM_INIT(&control_t2, 1); // Start t2 unblocked
    
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