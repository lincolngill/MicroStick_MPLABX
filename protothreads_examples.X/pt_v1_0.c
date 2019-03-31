/*
  Protothread V1.0
 */
#define SYS_FREQ 64000000
#define PT_USE_UART_SERIAL
#define PT_UART_BAUD 9600
#define PT_UART_INPUT_BUFFER_SIZE 128
#define PT_UART_OUTPUT_BUFFER_SIZE 128
#define PT_USE_TIMER2_MSEC
#define PT_USE_TIMER45_USEC
#define PT_USE_RATE_SCHED
#define PT_USE_DEBUG_PIN

#include "pt-extended.h"

// semaphores for controlling threads
static struct pt_sem control_t1, control_t2;
static struct pt pt1, pt2, pt3, pt4, pt5, pt_input, pt_DMA_output;

// Turn on and off blinking & wait times
static int cntl_blink = 1;
static int wait_t1 = 1000; //ms
static int wait_t2 = 500000; //us
static int run_t4 = 1;

int sys_time_seconds = 0;

// Thread priority scheduler - Priorities
// Rates:
// 0 - Every time through loop
// 1 - Every 2nd time through loop
// 2 - Every 4th time
// 3 - Every 8th time
// 4 - Every 16th time
int t1_rate=3, t2_rate=3, t3_rate=0, t4_rate=4;

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
        PT_DEBUG_VALUE(5,10);                    // 2ms pulse on pin 25 at 1.1V
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
        PT_DEBUG_VALUE(10,10);                    // 5ms pulse on pin 25 at 1.4V
        mPORTAToggleBits(BIT_1);                // Blink LED 2 (Pin 3)
        PT_SEM_SIGNAL(pt, &control_t1);         // Signal thread 1
    }
    PT_END(pt);
}

// Thread 3
// DMA output
static unsigned char cmd[16];
static int value;
static PT_THREAD(protothread3(struct pt *pt))
{
    PT_BEGIN(pt);
    while (1) { 
        sprintf(pt_send_buffer,"cmd>");            // Send prompt
        PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output));
        PT_SPAWN(pt, &pt_input, PT_GetSerialBuffer(&pt_input));
        
        cmd[0] = '\0';          // If term_buffer is empty then sscanf doesn't change cmd or value
        sscanf(pt_term_buffer, "%s %d", cmd, &value);
        if (cmd[0]=='t' && cmd[1]=='1') wait_t1 = value;
        if (cmd[0]=='t' && cmd[1]=='2') wait_t2 = value;
        if (cmd[0]=='g' && cmd[1]=='1') cntl_blink = 1;
        if (cmd[0]=='s' && cmd[1]=='1') cntl_blink = 0;
        if (cmd[0]=='g' && cmd[1]=='2') run_t4 = 1;
        if (cmd[0]=='s' && cmd[1]=='2') run_t4 = 0;
        if (cmd[0]=='k') PT_EXIT(pt);
        if (cmd[0]=='p' && cmd[1]=='1') t1_rate = value;
        if (cmd[0]=='p' && cmd[1]=='2') t2_rate = value;
        if (cmd[0]=='p' && cmd[1]=='3') t3_rate = value;
        if (cmd[0]=='p' && cmd[1]=='4') t4_rate = value;
        if (cmd[0]=='p' && cmd[1]=='\0') {
            sprintf(pt_send_buffer, "LED1Timer=%d LED2Timer=%d t1r=%d t2r=%d t3r=%d t4r=%d looppri=0x%X dma_cnt=%d\n\r", \
                    wait_t1, wait_t2, t1_rate, t2_rate, t3_rate, t4_rate, pt_loop_priority, pt_dma_cnt);
            PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output));
        }
        if (cmd[0]=='t' && cmd[1]=='\0') {
            sprintf(pt_send_buffer, "PT_GET_TIME=%d\tsys_time_seconds=%d\tCVRCON=0x%X\n\r", PT_GET_TIME(), sys_time_seconds, CVRCON_setup);
            PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output));
        }
    }
    PT_END(pt);
}

static PT_THREAD(protothread4(struct pt *pt))
{
    PT_BEGIN(pt);
    while (1) {
        mPORTBToggleBits(BIT_0);
        PT_DEBUG_VALUE(15,10);                   // 10ms pulse on pin 25 at 1V
        PT_YIELD_TIME_usec(wait_t2);
    }
    PT_END(pt);
}

// Thread 5 - system time keeper - tick every sec
static PT_THREAD (protothread5(struct pt *pt))
{
    PT_BEGIN(pt);
    while (1) {
        PT_YIELD_TIME_msec(1000);
        sys_time_seconds++;
    }
    PT_END(pt);
} // thread 5

int main(void)
{
    ANSELA = 0;                         // Make all Pins Digital
    ANSELB = 0;    
    
    // Setup port pins
    // Microstick ii - Pin 2 and 3. And pin 4
    mPORTASetBits(BIT_0 | BIT_1);       // Clear bits to ensure LEDs are off
    mPORTASetPinsDigitalOut(BIT_0 | BIT_1); // Set pins as output
    mPORTBSetBits(BIT_0);               // Clear bits to ensure LED is off
    mPORTBSetPinsDigitalOut(BIT_0);     // Set pin as output
    
    PT_setup();
    
    // Init Semophores
    PT_SEM_INIT(&control_t1, 0); // Start t1 blocked
    PT_SEM_INIT(&control_t2, 1); // Start t2 unblocked
    
    // Init threads
    PT_INIT(&pt1);
    PT_INIT(&pt2);
    PT_INIT(&pt3);
    PT_INIT(&pt4);
    PT_INIT(&pt5);

    while (1) {
        PT_RATE_SCHEDULE(protothread1(&pt1), t1_rate);
        PT_RATE_SCHEDULE(protothread2(&pt2), t2_rate);
        if (cmd[0] != 'k') PT_RATE_SCHEDULE(protothread3(&pt3), t3_rate);
        if (run_t4) PT_RATE_SCHEDULE(protothread4(&pt4), t4_rate);
        PT_RATE_SCHEDULE(protothread5(&pt5), 0);
        PT_RATE_LOOP();
    }
}