/*
  Protothread Timer, Compare & Capture E.g. 1
 * 
 * Timer2 pulse -Pin4->
 *   |                         Timer3
 *   |                           |
 *   -> OC2 -Pin14-  --          v
 *   |              or |-Pin6-> IC1 -> capture1
 *   -> OC3 -Pin18-  --
 * 
 */
#define SYS_FREQ 64000000
#define PT_USE_UART_SERIAL
#define PT_UART_BAUD 9600
#define PT_UART_INPUT_BUFFER_SIZE 64
#define PT_UART_OUTPUT_BUFFER_SIZE 128
#define PT_USE_TIMER5_MSEC

#include "pt-extended.h"

// semaphores for controlling threads
static struct pt_sem uart_free_sem;
static struct pt pt_print, pt_cmd, pt_time, pt_input, pt_DMA_output;

int sys_time_seconds = 0;

// The measured period of the wave
short capture1, last_capture1=0, capture_period, min_period=32000, max_period=0;

// The actual period of the wave
int generate_period=10000;

// Printing state
int printing=0;

// Timer 2 ISR
// Toggle timing probe pin (blip)
void __ISR(_TIMER_2_VECTOR, IPL2SOFT) Timer2Handler(void)
{
    mPORTBSetBits(BIT_0);                       // Set Pin 4
    mT2ClearIntFlag();
    mPORTBClearBits(BIT_0);                     // Clear Pin 4
}

// Capture 1 ISR
// Check capture for consistency
void __ISR(_INPUT_CAPTURE_1_VECTOR, IPL3SOFT) C1Handler(void)
{
    capture1 = mIC1ReadCapture();
    capture_period = capture1 - last_capture1;
    // Avoid start up error
    if (capture_period > max_period && last_capture1>0) max_period = capture_period;
    if (capture_period < min_period && last_capture1>0 && capture_period>0) min_period = capture_period;
    last_capture1 = capture1;
    mIC1ClearIntFlag();
}

static unsigned char cmd[16];
static unsigned int value;
// Command Thread
static PT_THREAD(protothread_cmd(struct pt *pt))
{
    PT_BEGIN(pt);
    while (1) {
        PT_SEM_WAIT(pt, &uart_free_sem);                // Wait for uart to be free
        sprintf(pt_send_buffer,"cmd>");
        PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output));
        PT_SEM_SIGNAL(pt, &uart_free_sem);
        PT_SPAWN(pt, &pt_input, PT_GetSerialBuffer(&pt_input));
        
        cmd[0] = '\0';          // If term_buffer is empty then sscanf doesn't change cmd or value
        sscanf(pt_term_buffer, "%s %d", cmd, &value);   
        if (cmd[0]=='v') {
            printing=1;
            PT_WAIT_UNTIL(pt, ! printing);              // Stop thread until printing complete
        }
        if (cmd[0]=='p') {
            generate_period = value;
            CloseTimer2();
            OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, generate_period);
            ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
            mT2ClearIntFlag();
            CloseOC2();
            OpenOC2(OC_ON | OC_TIMER2_SRC | OC_CONTINUE_PULSE, generate_period>>1, generate_period>>2);
            CloseOC3();
            OpenOC3(OC_ON | OC_TIMER2_SRC | OC_CONTINUE_PULSE, generate_period-1, generate_period>>1);
            last_capture1=0; min_period=32000; max_period=0;        // Reset capture Stats
        }
    }
    PT_END(pt)
} // Command thread

// Print thread
static PT_THREAD(protothread_print(struct pt *pt))
{
    PT_BEGIN(pt);
    while (1) {
        PT_WAIT_UNTIL(pt, printing);            // Wait until command to print 'v'
        PT_SEM_WAIT(pt, &uart_free_sem);            // Wait until UART is free
        sprintf(pt_send_buffer, "gen=%d cap=%d min=%d max=%d secs=%d\n\r", generate_period, capture_period, min_period, max_period, sys_time_seconds);
        PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output));
        PT_SEM_SIGNAL(pt, &uart_free_sem);
        printing=0;                             // Stop this thread
    }
    PT_END(pt);
} // Print thread

// One Second Thread
static PT_THREAD(protothread_time(struct pt *pt))
{
    PT_BEGIN(pt);
    while (1) {
        PT_YIELD_TIME_msec(1000);
        sys_time_seconds++;
    }
    PT_END(pt);
} // Time thread

int main(void)
{
    ANSELA = 0;                         // Make all Pins Digital
    ANSELB = 0;  
    
    // Setup timer2 for wave period pulse generation
    // timer2 reset on full period interval.
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, generate_period);
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    mT2ClearIntFlag();

    // Setup output compare 3
    // Dual compare Mode: Continuous Output Pulses
    // OC3CON OCM<2;0> = 0b101 - Initialise OC3 low; Generate continuous output pulses on OC3
    // OC3R (compare reg)            = generate_period>>1 (50% of timer period) - Go HIGH
    // OC3SR (secondary compare reg) = generate_period-1  (End of timer period) - Go LOW
    // Wave = (0011) __--__--__--
    OpenOC3(OC_ON | OC_TIMER2_SRC | OC_CONTINUE_PULSE, generate_period-1, generate_period>>1);
    PPSOutput(4, RPB9, OC3);            // PPS group 4, Pin 18
    
    // Setup output compare 2
    // OC2R  = generate_period>>2 (25% of timer period) - Go HIGH
    // OC2SR = generate_period>>1 (50% of timer period) - Go LOW
    // Wave = (0100) _-___-___-__
    OpenOC2(OC_ON | OC_TIMER2_SRC | OC_CONTINUE_PULSE, generate_period>>1, generate_period>>2);
    PPSOutput(2, RPB5, OC2);            // PPS group 2, Pin 14
    
    // Timer 3 - free running
    // Source for input capture
    // just overflow so continuous readings
    OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_1, 0xffff);
    
    // Setup input capture
    OpenCapture1( IC_EVERY_RISE_EDGE | IC_INT_1CAPTURE | IC_TIMER3_SRC | IC_ON );
    ConfigIntCapture1( IC_INT_ON | IC_INT_PRIOR_3 | IC_INT_SUB_PRIOR_3 );
    INTClearFlag(INT_IC1);
    PPSInput(3, IC1, RPB2);             // PPS group 3, Pin 6
    mPORTBSetPinsDigitalIn(BIT_2);      // Set Pin 6 as input
    
    // Pulse pin for Timer2 ISR
    // Not strictly needed for anything
    mPORTBSetPinsDigitalOut(BIT_0);      // Set Pin 4 as output
    
    PT_setup();
    
    // Init Semophores
    PT_SEM_INIT(&uart_free_sem, 1);         // Start ready to send
    
    // Init threads
    PT_INIT(&pt_print);
    PT_INIT(&pt_cmd);
    PT_INIT(&pt_time);

    while (1) {
        PT_SCHEDULE(protothread_print(&pt_print));
        PT_SCHEDULE(protothread_cmd(&pt_cmd));
        PT_SCHEDULE(protothread_time(&pt_time));
    }
}