/*
 * pt-extended.h - Extended Protothread library
 * 
 * Define configuration options E.g.
#define SYS_FREQ 64000000
#define PT_USE_UART_SERIAL
#define PT_UART_BAUD 9600
#define PT_UART_INPUT_BUFFER_SIZE 128
#define PT_UART_OUTPUT_BUFFER_SIZE 128
#define PT_USE_TIMER2_MSEC
#define PT_USE_TIMER5_MSEC
#define PT_USE_TIMER45_USEC
#define PT_USE_RATE_SCHED
#define PT_USE_DEBUG_PIN

#include "pt-extended.h"
 */

#ifndef _PT_EXTENDED
#define _PT_EXTENDED

#include "config.h"                     // Configure clock and Configuration registers
#include "pt.h"                         // Original Protothread library
#include "pt-sem.h"                     // Original Protothread semaphore library

//-----------------------------------------------------
#ifdef PT_USE_UART_SERIAL
#include <stdio.h>                      // for sprintf & sscanf

#ifndef PT_UART_PRINT_STARTUP_MSG
#define PT_UART_PRINT_STARTUP_MSG() printf("PT UART Started: BAUD=%d\n\r",pt_uart_baud_actual)
#endif // PT_UART_PRINT_STARTUP_MSG

// Default Baud rate = 9600
#ifndef PT_UART_BAUD
#warning PT_UART_BAUD not defined using 9600
#define PT_UART_BAUD 9600
#endif // PT_UART_BAUD

// Default input buffer size = 128
#ifndef PT_UART_INPUT_BUFFER_SIZE
#warning PT_UART_INPUT_BUFFER_SIZE not defined using 128
#define PT_UART_INPUT_BUFFER_SIZE 128
#endif // PT_UART_INPUT_BUFFER_SIZE

// Default output buffer size = 128
#ifndef PT_UART_OUTPUT_BUFFER_SIZE
#warning PT_UART_OUTPUT_BUFFER_SIZE not defined using 128
#define PT_UART_OUTPUT_BUFFER_SIZE 128
#endif // PT_UART_OUTPUT_BUFFER_SIZE

// Some helpful serial termainal macros
#define clrscr() printf("\x1b[2J")
#define home() printf("\x1b[H")
#define pcr() printf("\r")
#define crlf putchar(0x0a); putchar(0x0d);
#define backspace 0x7f

// UART2 RX - Read input line from UART2
// Spawn protothread to get a line of input into pt_term_buffer E.g.
// PT_SPAWN(pt, &pt_input, PT_GetSerialBuffer(&pt_input));
unsigned char pt_term_buffer[PT_UART_INPUT_BUFFER_SIZE];
int PT_GetSerialBuffer(struct pt *pt)
{
    static unsigned char character;
    static int num_char;
    PT_BEGIN(pt);
    num_char = 0;
    while (num_char < PT_UART_INPUT_BUFFER_SIZE)
    {
        // Yield until a character has been received
        PT_YIELD_UNTIL(pt, UARTReceivedDataIsAvailable(UART2));
        character = UARTGetDataByte(UART2);
        PT_YIELD_UNTIL(pt, UARTTransmitterIsReady(UART2));
        UARTSendDataByte(UART2, character);
        if (character == '\r') {
            pt_term_buffer[num_char] = '\0'; //Terminate string
            PT_YIELD_UNTIL(pt, UARTTransmitterIsReady(UART2));
            UARTSendDataByte(UART2, '\n');
            break;
        }
        else if (character == backspace) {
            num_char--;
            if (num_char < 0) num_char = 0;
        }
        else {
            pt_term_buffer[num_char++] = character;
        }
    }
    // Kill this thread to allow spawning thread to execute
    PT_EXIT(pt);
    PT_END(pt); // End thread
}

// UART2 TX - Send a string to UART2 via DMA channel 1
// Spawn protothread to initiate DMA transfer and wait for it to complete. E.g.
// PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output));
unsigned char pt_send_buffer[PT_UART_OUTPUT_BUFFER_SIZE];
static int pt_dma_cnt = 0; // count the number of time the DMA channel enable is called.
int PT_DMA_PutSerialBuffer(struct pt *pt)
{
    PT_BEGIN(pt);
    // Need to wait until the UART FIFO buffer is completely empty.
    // Otherwise interrupts for inprogress FIFO bytes cause DMA to overrun FIFO buffers
    PT_YIELD_UNTIL(pt, UARTTransmissionHasCompleted(UART2));
    DmaChnEnable(DMA_CHANNEL1);                 // Enabling channel kicks transfer off
    pt_dma_cnt++;                               // Count the number of DMA transfers done
    PT_YIELD_UNTIL(pt, DmaChnGetEvFlags(DMA_CHANNEL1) & DMA_EV_BLOCK_DONE); // Wait until DMA block has been done
    PT_EXIT(pt);
    PT_END(pt);
}

#endif // PT_USE_UART_SERIAL

//--------------------------------------------------------------
#ifdef PT_USE_TIMER2_MSEC
// Setup Timer2 as a ms timer
volatile int pt_milliSec = 0;              // Volatile because updated in Timer2 ISR
void __ISR(_TIMER_2_VECTOR, IPL2SOFT) Timer2Handler(void)
{
    mT2ClearIntFlag();
    pt_milliSec++;
}

// estimate microSec since start up
// Based on milliSec + value of timer2
long long uSec(void) {
#if PB_MHZ == 40
    // When PB_CLK 40MHz timer counter is 0 - 40000
    return (long long)pt_milliSec * 1000 + (long long)ReadTimer2()/PB_MHZ;
#elif PB_MHZ == 32
    // When PB_CLK 32MHz timer counter is 0 - 1000
    return (long long)pt_milliSec * 1000 + (long long)ReadTimer2();
#else
#error Invalid PB_MHZ. Must be 32 or 40
#endif
}
#endif // USE_TIMER2_MSEC

#ifdef PT_USE_TIMER5_MSEC
// Setup Timer5 as a ms timer
volatile int pt_milliSec = 0;              // Volatile because updated in Timer2 ISR
void __ISR(_TIMER_5_VECTOR, IPL2SOFT) Timer5Handler(void)
{
    mT5ClearIntFlag();
    pt_milliSec++;
}
#endif // PT_USE_TIMER5_MSEC

#define PT_YIELD_TIME_msec(delay_time) \
   do { static unsigned int time_thread; \
   time_thread = pt_milliSec + (unsigned int)delay_time; \
   PT_YIELD_UNTIL(pt, pt_milliSec >= time_thread); \
   } while(0);

#define PT_GET_TIME() (pt_milliSec)


//---------------------------------------------------
#ifdef PT_USE_TIMER45_USEC
// Macro for usec wait using 32-bit counter
// delay_time < 200e30
// Reads timer45
#define PT_YIELD_TIME_usec(delay_time) \
   do { static unsigned int time_thread; \
   time_thread = ReadTimer45() + delay_time; \
   PT_YIELD_UNTIL(pt, ReadTimer45() >= time_thread); \
   } while (0);
#endif // PT_USE_TIMER45_USEC

//-----------------------------------------------------------
#ifdef PT_USE_DEBUG_PIN
int CVRCON_setup;
// Level = 0-15
// duration for loop count*7
#define PT_DEBUG_VALUE(level, duration) \
do { static unsigned int i; \
   CVRCON = CVRCON_setup | (level & 0xf); \
   if (duration>0) { \
      for (i=0; i<duration*60; i++); \
      CVRCON = CVRCON_setup; \
   } \
} while(0);
#endif //PT_USE_DEBUG_PIN

//----------------------------------------------------------
#ifdef PT_USE_RATE_SCHED

int pt_loop_priority = 0;
#define PT_RATE_LOOP() pt_loop_priority = ++pt_loop_priority & 0b1111

// Thread priority scheduler - Priorities
// Rates:
// 0 - Every time through loop
// 1 - Every 2nd time through loop
// 2 - Every 4th time
// 3 - Every 8th time
// 4 - Every 16th time
#define PT_RATE_SCHEDULE(f,rate) \
   if ((rate==0) | \
   (rate==1 && ((pt_loop_priority & 0b1)==0) ) | \
   (rate==2 && ((pt_loop_priority & 0b11)==0) ) | \
   (rate==3 && ((pt_loop_priority & 0b111)==0) ) | \
   (rate==4 && ((pt_loop_priority & 0b1111)==0) )) \
      PT_SCHEDULE(f);

#endif // PT_USE_RATE_SCHED

//--------------------------------------------------
// Protothread setup function
void PT_setup(void)
{   
    //Configure Flash wait states and prefetch cache for max performance based on SYSCLK
    SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);
    
#ifdef PT_USE_UART_SERIAL
    int pt_uart_baud_actual; //Store calculated BAUD rate when initialised
    // Setup USART IO pins
    PPSInput(2, U2RX, RPB11); // Pin 22
    PPSOutput(4, RPB10, U2TX); // Pin 21
    
    UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
    //UARTSetFifoMode() // No Interrupts setup
    UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    pt_uart_baud_actual = UARTSetDataRate(UART2, PB_FREQ, PT_UART_BAUD);
    UARTEnable(UART2, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX)); 
    clrscr(); home();
    PT_UART_PRINT_STARTUP_MSG();
    
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
    DmaChnSetTxfer(DMA_CHANNEL1, pt_send_buffer, (void*)&U2TXREG, PT_UART_OUTPUT_BUFFER_SIZE, 1, 1);
    DmaChnSetEvEnableFlags(DMA_CHANNEL1, DMA_EV_BLOCK_DONE); // Signal when block done
    DmaChnSetMatchPattern(DMA_CHANNEL1, '\0');
#endif // PT_USE_UART_SERIAL
    
#ifdef PT_USE_TIMER2_MSEC
    // Setup Timer2 as 1ms counter
#if PB_MHZ == 32
    // Enable T2, Use internal clock (PBCLK), Prescalar 1:32
    // Tick = 32000000/32 = 1000000 (1usec)
    // 1000 counts = 1ms
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_32, 1000);
#elif PB_MHZ == 40
    // Prescalar 1:1
    // 40000 counts = 1ms
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, PB_FREQ/1000);
#else
#error Invalid PB_MHZ. Must be 32 or 40
#endif // PB_MHZ
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2 | T2_INT_SUB_PRIOR_0);
    mT2ClearIntFlag();
#endif // PT_USE_TIMER2_MSEC
    
#ifdef PT_USE_TIMER5_MSEC
    // Setup Timer5 as 1ms counter
#if PB_MHZ == 32
    // Enable T5, Use internal clock (PBCLK), Prescalar 1:32
    // Tick = 32000000/32 = 1000000 (1usec)
    // 1000 counts = 1ms
    OpenTimer5(T5_ON | T5_SOURCE_INT | T5_PS_1_32, 1000);
#elif PB_MHZ == 40
    // Prescalar 1:1
    // 40000 counts = 1ms
    OpenTimer5(T5_ON | T5_SOURCE_INT | T5_PS_1_1, PB_FREQ/1000);
#else
#error Invalid PB_MHZ. Must be 32 or 40
#endif // PB_MHZ
    ConfigIntTimer5(T5_INT_ON | T5_INT_PRIOR_2 | T5_INT_SUB_PRIOR_0);
    mT5ClearIntFlag();
#endif // PT_USE_TIMER5_MSEC
    
#ifdef PT_USE_TIMER45_USEC
#if PB_MHZ != 32
#error PB_MHZ must be 32 to use Timer45 as a uSec timer. Set SYS_FREQ=64000000
#endif // PB_MHZ
    // Timer45
    // Tick = 32000000/32 (postscalar) = 1000000 (1 usec)
    // Just overflow. no interrupt. Value read for PT_YIELD_TIME_usec
    OpenTimer45(T45_ON | T45_SOURCE_INT | T45_PS_1_32, 0xffffffff);
#endif //PT_USE_TIMER45_USEC
    
#ifdef PT_USE_DEBUG_PIN
    // Setup Vref pin for debugging. Use as a 4-bit DAC
    // Range = 0.8 - 2.5V in 0.1V steps
    // CVREFOUT on Pin 25
    // CVRCON = 0x8060
    // ON=0b1 - Module Enabled
    // VREFSEL=0b0 - Voltage Reference Select bit - From resistor network
    // BGSEL=0b00 - Band Gap Ref Source - IV_REF=1.2V
    // CVROE=0b1 - CV_REFOUT enable - Voltage level is output on CV_REFOUT pin
    // CVRR=0b0 - CV_REF Range - 0.25 CV_RSRC to 0.75 CV_RSRC with CV_RSRC/32 step size
    // CVRSS=0b0 - CV_REF Source - CV_RSRC = AV_DD - AV_SS
    // CVR=0b0000 - CV_REF Value select 0 <= CVR<3:0> <= 15 bits - CV_REF = 1/4*CV_RSRC + CVR<3:0>/32*CV_RSRC
    CVREFOpen( CVREF_ENABLE | CVREF_OUTPUT_ENABLE | CVREF_RANGE_LOW | CVREF_SOURCE_AVDD | CVREF_STEP_0 );
    CVRCON_setup = CVRCON; // 0x8060
#endif //PT_USE_DEBUG_PIN
    
    INTEnableSystemMultiVectoredInt();
}

#endif // _PT_EXTENDED