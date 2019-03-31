/*
  Protothread Timer, Compare & Capture E.g. 3
 * PWM 500kHz out OC3 and PWM on-time set by a Sin wave Direct Digital Synthesis (DDS)
 * 
 * Timer2 pulse -Pin4->
 *   | 
 *   -> OC3 -Pin18->  
 * 
 */
#define SYS_FREQ 64000000
#define PT_USE_UART_SERIAL
#define PT_UART_BAUD 9600
#define PT_UART_INPUT_BUFFER_SIZE 64
#define PT_UART_OUTPUT_BUFFER_SIZE 128
#define PT_UART_PRINT_STARTUP_MSG() printf("PT Compare Eg3 PWM/DDS Started: BAUD=%d\n\r",pt_uart_baud_actual)
#define PT_USE_TIMER5_MSEC

#include "pt-extended.h"
#include <math.h>

// semaphores for controlling threads
static struct pt_sem uart_free_sem;
static struct pt pt_scale, pt_cmd, pt_time, pt_input, pt_DMA_output;

// System time since startup (secs)
int sys_time_seconds = 0;

// The actual period of the wave
// 32MHz * 64 cycles = 32,000,000 / 64 = 500,000 or 500 kHz 
unsigned int generate_period= 63;
unsigned int pwm_on_time = 32;

// Sine lookup table
volatile signed char sine_table[64];
unsigned int sine_index = 0;

// DDS variables
// Starting DDS increment so Freq=1000Hz
// 500kHz timer
// 2^32 / 500 = 8,590,000
volatile unsigned int DDS_accumulator=0, DDS_increment=8590000, DDS_amp=0;

// play a scale
// Flag to kick of play scale thread
int play_scale=0;
/*
 * C4   262 Hz (middle C)
 * D4   294
 * E4   330
 * F4   349
 * G4   392
 * A4   440
 * B4   494
 * C5   523
 */
// Scale freq in Hz
int notes[9]={262, 294, 330, 349, 392, 440, 494, 523, 0};

// Timer 2 ISR
// Toggle timing probe pin (blip)
void __ISR(_TIMER_2_VECTOR, IPL2SOFT) Timer2Handler(void)
{
    mPORTBSetBits(BIT_0);                       // Set Pin 4
    // Change the PWM duty cycle
    // 500 kHz = 2us period
    DDS_accumulator += DDS_increment;
    sine_index = DDS_accumulator>>26;
    pwm_on_time = (sine_table[sine_index]>>DDS_amp) + 32;
    SetDCOC3PWM(pwm_on_time);
    mT2ClearIntFlag();
    mPORTBClearBits(BIT_0);                     // Clear Pin 4
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
        if (cmd[0]=='f') {
            // increment = freq * (2^32)/(PWM rate)
            // PWM rate = 500 kHz
            DDS_increment = (unsigned int)(value*8590);
        }
        if (cmd[0]=='s') {
            // play scale
            play_scale = 1;
        }
    }
    PT_END(pt)
} // Command thread

// Play scale thread
static PT_THREAD(protothread_scale(struct pt *pt))
{
    PT_BEGIN(pt);
    while (1) {
        static int i, save_DDS;
        PT_YIELD_UNTIL(pt,play_scale);              // Wait for signal to play
        save_DDS = DDS_increment;
        for (i=0; i<9; i++) {
            DDS_increment = (unsigned int)(notes[i]*8590);
            PT_YIELD_TIME_msec(250);
        }
        play_scale=0;
        // restore freq
        DDS_increment = save_DDS;
    }
    PT_END(pt);
}

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
    // period = 63 cycle = 508kHz
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, generate_period);
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    mT2ClearIntFlag();

    // Setup output compare 3 for PWM
    // PWM period = 63 cycles = 508kHz
    // PWN duty cycle = 32 
    // OC3CON OCM<2:0> = 0b110 = PWM mode on OC3; Fault pin disabled
    // OC3R = slave duty cycle (RO)
    // OC3RS = Buffer reg to update PWM duty cycle (Write)
    // When TMR2 = OC3R (match) - (end of PWM period)) OC3R set to OC3RS
    // TyIF interrupt flag is asserted at each PWN period boundary
    // Set the initial PWM duty cycle = pwm_on_time. i.e. OC3R = OC3RS = pwm_on_time = 32
    OpenOC3(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, pwm_on_time, pwm_on_time);
    PPSOutput(4, RPB9, OC3);            // PPS group 4, Pin 18
    
    // Pulse pin for Timer2 ISR
    // Not strictly needed for anything
    mPORTBSetPinsDigitalOut(BIT_0);      // Set Pin 4 as output
    
    // build sine table
    // 64 8bit values
    // An amplitude value for each clock cycle of the PWM period
    // -32 to 32 (6 bits))
    // sine_table[0] = 0
    // sine_table[16] = 32
    // sine_table[32] = 0
    // sine_table[48] = -32
    int i;
    for (i=0; i<64; i++) {
        // 8 bit amp, 64 samples
        sine_table[i] = (signed char) (32.0 * sin((float)i*6.283/(float)64));
    }
    
    PT_setup();
    
    // Init Semophores
    PT_SEM_INIT(&uart_free_sem, 1);         // Start ready to send
    
    // Init threads
    PT_INIT(&pt_cmd);
    PT_INIT(&pt_time);

    while (1) {
        PT_SCHEDULE(protothread_cmd(&pt_cmd));
        PT_SCHEDULE(protothread_time(&pt_time));
        PT_SCHEDULE(protothread_scale(&pt_scale));
    }
}