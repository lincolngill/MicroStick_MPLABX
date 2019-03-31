/*
 * Generates 2 square wave at a fixed frequency - With no code. Just setup the Timer and OC
 * First wave is 50% +duty cycle
 * The other is 25% +duty cycle
 * Also flashes the LED - RA0 Pin2
 * 
 * Uses Proto thread library
 *    - T5 - 1ms timer.
 * T2 - Direct source to OC1 and OC2
 *    - No T2 interrupt
 * OC1 - outputs the 50% duty cycle - RB4 Pin 12
 * OC2 - outputs the 25% duty cycle - RB5 Pin 14
 * 
 * T3 - Free running timer - IC2 captures for timing
 * IC2 - Interrupt on raising edge - RB10 Pin 21
 *     - ISR captures T3 and works out diff between captures
 *     - Use a 300ohm resister to protect pin
 *     - Floating pin can cause repeated resets!
 * 
 * TFT - SPI1
 * Pin 4: DC/RS
 * Pin 5: CS
 * Pin 6: RESET
 * Pin 22: MOSI
 * Pin 25: SCK
 * 
 * Pin 2: LED on board
 * Pin 12: OC1 output - 50% duty square wave
 * Pin 14: OC2 output - 25% duty square wave
 * Pin 21: IC2
 */

#define VERSION "v1.0"

// 40MHz SYSCLK and PBCLK
// Timer5 for ms timer
#define SYS_FREQ 40000000
#define PT_USE_TIMER5_MSEC
#include "pt-extended.h"

// graphics libraries for TFT
#include "tft_master.h"
#include "tft_gfx.h"

// For sprintf
#include <stdio.h>

// Square wave HZ
#define T2_HZ 4000
uint16_t t2_ticks; // ticks/period

// LED = RA0 Pin 2
// Jumper on Microstick needs to be closed.
// PORTA BIT_0
#define LED_INIT      (TRISACLR = 0x01, ANSELACLR = 0x01) // Set LOW and Make an output pin
#define LED_HIGH      (LATASET = 0x01)
#define LED_LOW       (LATACLR = 0x01)
#define LED_TOGGLE    (LATAINV = 0x01)
#define LED_TOGGLE_MS 500

static struct pt pt_led, pt_timer, pt_print;
// Seconds since system startup. Updated by protothread_timer
int sys_time_seconds = 0;
// string buffer for TFT output
char buffer[60];

// IC2 Interrupt Handler
// Update ic2_tick_diff with number of ticks since last capture. Should be 10,000 or 10,001
uint16_t ic2_ticks=0, ic2_tick_diff=0, ic2_last_ticks=0;
void __ISR(_INPUT_CAPTURE_2_VECTOR, IPL3AUTO) IC2Handler(void)
{
    ic2_ticks = mIC2ReadCapture();
    ic2_tick_diff = ic2_ticks - ic2_last_ticks;
    ic2_last_ticks = ic2_ticks;
    mIC2ClearIntFlag();
}

// Toggle LED pin
static PT_THREAD (protothread_led(struct pt *pt))
{
    PT_BEGIN(pt);
    while(1) {
        PT_YIELD_TIME_msec(LED_TOGGLE_MS) ;
        LED_TOGGLE;
    }
    PT_END(pt);
}

// === Timer Thread =================================================
// update a 1 second tick counter
// Writes header to TFT on initial entry
static PT_THREAD (protothread_timer(struct pt *pt))
{
    PT_BEGIN(pt);
    tft_setCursor(0, 0);
    tft_setTextColor(ILI9340_WHITE);  tft_setTextSize(2);
    sprintf(buffer, "SquareWave - %s", VERSION);
    tft_writeString(buffer);
    while(1) {
        // yield time 1 second
        PT_YIELD_TIME_msec(1000) ;
        sys_time_seconds++ ;
        
        // draw sys_time
        tft_fillRect(0,20, ILI9340_TFTWIDTH, 16, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_setCursor(0, 20);
        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
        sprintf(buffer,"%d", sys_time_seconds);
        tft_writeString(buffer);
        // NEVER exit while
      } // END WHILE(1)
  PT_END(pt);
} // timer thread

// Print the wave period ticks and the captured tick counter
static PT_THREAD (protothread_print(struct pt *pt))
{
    PT_BEGIN(pt);
    tft_setCursor(0, 60);
    tft_setTextColor(ILI9340_WHITE);  tft_setTextSize(1);
    sprintf(buffer, "Pin 21 <-300ohm- Pin 11 | Pin 14");
    tft_writeString(buffer);
    tft_setCursor(0, 80);
    tft_setTextColor(ILI9340_RED); tft_setTextSize(2);
    sprintf(buffer,"Gen=%d", t2_ticks);
    tft_writeString(buffer);
    while(1) {
        PT_YIELD_TIME_msec(200) ;
        tft_fillRect(0,100, ILI9340_TFTWIDTH, 16, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_setCursor(0, 100);
        tft_setTextColor(ILI9340_RED); tft_setTextSize(2);
        sprintf(buffer,"Diff=%d T3=%d", ic2_tick_diff, ic2_ticks);
        tft_writeString(buffer);
      } // END WHILE(1)
  PT_END(pt);
} // timer thread

// Setup the TFT display
void tft_initialise (void)
{    // init the display
    tft_init_hw();
    tft_begin();
    tft_fillScreen(ILI9340_BLACK);
    //240x320 vertical display
    tft_setRotation(0); // Use tft_setRotation(1) for 320x240
}

int main(void)
{
    ANSELA = 0; // Make all Pins Digital
    ANSELB = 0;
    
    // Init Screen
    tft_initialise();
    
    // Init Protothreads
    // Start T5
    PT_setup();
    // Init threads
    PT_INIT(&pt_timer);
    PT_INIT(&pt_led);
    PT_INIT(&pt_print);
    
    // Init LED Pin as output
    LED_INIT;
    
    // Setup wave timer T2 - No interrupt
    t2_ticks = SYS_FREQ/T2_HZ;
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, t2_ticks);
    
    // OC1 <- T2 (10,000 ticks)
    // value1 = 5,000 - Go HIGH
    // value2 = 9,999 - Go LOW
    // Output RB4 - Pin 11
    OpenOC1( OC_ON | OC_TIMER_MODE16 | OC_TIMER2_SRC | OC_CONTINUE_PULSE, t2_ticks, t2_ticks>>1);
    PPSOutput(1, RPB4, OC1);
    
    // OC2 <- T2
    // value1 = 2,500 - Go HIGH
    // value2 = 5,000 - Go LOW
    // Output RB5 - Pin 14
    OpenOC2( OC_ON | OC_TIMER_MODE16 | OC_TIMER2_SRC | OC_CONTINUE_PULSE, t2_ticks>>1, t2_ticks>>2);
    PPSOutput(2, RPB5, OC2);
    
    // T3 free running timer- No interrupt
    OpenTimer3( T3_ON | T3_SOURCE_INT | T3_PS_1_1, 0xFFFF);
    
    // IC2 
    // Input RB10 - Pin 21
    OpenCapture2( IC_ON | IC_CAP_16BIT | IC_TIMER3_SRC | IC_INT_1CAPTURE |  IC_EVERY_RISE_EDGE);
    ConfigIntCapture2( IC_INT_ON | IC_INT_PRIOR_3 | IC_INT_SUB_PRIOR_3 );
    mIC2ClearIntFlag();
    PPSInput(4, IC2, RPB10);
    
    // Enable Interrupts
    INTEnableSystemMultiVectoredInt();
    
    // Scheduler
    while (1) {
        PT_SCHEDULE(protothread_timer(&pt_timer));
        PT_SCHEDULE(protothread_led(&pt_led));
        PT_SCHEDULE(protothread_print(&pt_print));
    }
}