/*
 * Lab01 - Capacity Meter 
 * https://people.ece.cornell.edu/land/courses/ece4760/labs/f2016/lab1.html
 * 
 * Pin 7 (RB3, C1INA): Capacitor voltage input reading. +ve input
 * Vref 1.2V: -ve input to comparator
 * Pin 18 (RPB9, C1OUT): Output from comparator. Linked to Pin 24
 * Pin 24 (RPB13, IC1): Input capture
 * Timer3: Captured by IC1
 * 
 * Timer5: 1ms timer
 * 
 * TFT - SPI1
 * Pin 4: DC/RS
 * Pin 5: CS
 * Pin 6: RESET
 * Pin 22: MOSI
 * Pin 25: SCK
 * 
 */

#define VERSION "v2.2"
#define DEBUG

// Capacitor range 1 - 100 nF
// Discharge ms needs to be >= 5RC of discharge circuit
// 5 * 100 (ohms) * 100 (nF)  = 50us
// 10ms is heaps
#define DISCHARGE_MS 10

// Wait between taking readings (ms)
#define PAUSE_MS 1200

// Minimum number of timer ticks for a valid reading.
// Anything lower than this, then capacitance is too low or there is no cap!
#define MIN_TICKS 100

// RC circuit info used to calculate capacitance
// R = Charging Resistor (ohms)
// VS = Source Voltage
// VREF = Reference voltage used for comparator -ve input.
#define R 47000
#define VS 3.3
#define VREF 1.2

// 40MHz SYSCLK and PBCLK
// Timer5 for ms timer
#define SYS_FREQ 40000000
#define PT_USE_TIMER5_MSEC
#include "pt-extended.h"

// graphics libraries for TFT
#include "tft_master.h"
#include "tft_gfx.h"

// For flost maths
#include <math.h>
#include <stdio.h>

// protothread
static struct pt pt_led, pt_measure;

// System time since startup (secs)
int sys_time_seconds = 0;

volatile unsigned char got_update = 0;          // Flag a capture happened.
volatile unsigned char timer_expired = 0;       // Flag for timer reaching max value
volatile unsigned short capture1_tick = 0;      // Captured timer value.
unsigned int tick_ns;                           // period of each timer tick (ns)
unsigned int capture_ns;                        // Calculated capture time (ns)
// Reading Status of the measurement
// 0 - Pending 1st reading
// 1 - Good reading (capture occurred between MIN_TICKS and timer expiring)
// 2 - Captured ticks too low. Probably no Cap!
// 3 - Timer expired. Capacitance too large for timer range.
unsigned char reading_status = 0;
short led_colour = ILI9340_GREEN;
float capacitance = 0.0;                        // Calculated capacitance. For display

// string buffer for TFT output
char buffer[60];

// Timer 3 ISR
// Timer3 used for input capture 1
// If Timer3 max value reached then no reading captured within timer range.
void __ISR(_TIMER_3_VECTOR, IPL2SOFT) Timer3Handler(void)
{
    timer_expired = 1;
    mT3ClearIntFlag();
}

// Capture 1 ISR
// Flag protothread_measure thread that we have a measurement.
void __ISR(_INPUT_CAPTURE_1_VECTOR, IPL3SOFT) C1Handler(void)
{
    capture1_tick = mIC1ReadCapture();
    got_update = 1;
    mIC1ClearIntFlag();
}

// Output the static parts of the display
void tft_initialise (void)
{    // init the display
    tft_init_hw();
    tft_begin();
    tft_fillScreen(ILI9340_BLACK);
    //240x320 vertical display
    tft_setRotation(0); // Use tft_setRotation(1) for 320x240
    //
    // Display static text
    tft_setCursor(0,5);
    tft_setTextColor(ILI9340_WHITE);
    tft_setTextSize(1);
    sprintf(buffer, "Capacitor Meter %s", VERSION);
    tft_writeString(buffer);
#ifdef DEBUG        
    tft_setTextColor(ILI9340_WHITE);
    tft_setTextSize(1);
    tft_fillRoundRect(0,60, 175,60, 1, ILI9340_BLUE);// x,y,w,h,radius,color
    tft_setCursor(0, 60);
    sprintf(buffer,"%-10s: %10d ns", "Timer", capture_ns);
    tft_writeString(buffer);
    tft_setCursor(0, 70);
    sprintf(buffer,"%-10s: %10d", "Ticks", capture1_tick);
    tft_writeString(buffer);
    tft_setCursor(0, 80);
    sprintf(buffer,"%-10s: %10d", "Expired", timer_expired);
    tft_writeString(buffer);
    tft_setCursor(0, 90);
    sprintf(buffer,"%-10s: %10d", "Captured", got_update);
    tft_writeString(buffer);
    tft_setCursor(0, 100);
    sprintf(buffer,"%-10s: %10d", "Status", reading_status);
    tft_writeString(buffer);
    tft_setCursor(0, 110);
    sprintf(buffer,"%-10s: %10d Secs", "Uptime", sys_time_seconds);
    tft_writeString(buffer);
#endif 
}

// Update displayed values
void tft_refresh_values (void)
{    
    static unsigned char last_status = 255;    
    // Output Capacitor reading
    tft_setCursor(0, 20);
    tft_setTextColor(ILI9340_YELLOW);
    tft_setTextSize(2);
    //sprintf(buffer,"%10.4e F", capacitance);
    sprintf(buffer,"%10.2f nF", capacitance);
    tft_fillRoundRect(0,20, 240, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
    tft_writeString(buffer);

    // Output status
    if (reading_status != last_status) {
        tft_setCursor(0, 40);
        tft_setTextSize(2);
        tft_fillRoundRect(0,40, 240, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        if (reading_status == 0) {
            tft_setTextColor(ILI9340_RED);
            tft_writeString("Error: No Reading");
        }
        if (reading_status == 1) {
            tft_setTextColor(ILI9340_GREEN);
            tft_writeString("Status: OK");
        }
        if (reading_status == 2) {
            tft_setTextColor(ILI9340_RED);
            tft_writeString("Error: Cap Too Low");
        }
        if (reading_status == 3) {
            tft_setTextColor(ILI9340_RED);
            tft_writeString("Error: Cap Too High");
        }
        last_status = reading_status;
    }
    // output debug
#ifdef DEBUG
    tft_setTextColor(ILI9340_WHITE);
    tft_setTextSize(1);
    tft_fillRoundRect(65,60, 70,60, 1, ILI9340_BLUE);// x,y,w,h,radius,color
    tft_setCursor(65, 60);
    sprintf(buffer," %10d", capture_ns);
    tft_writeString(buffer);
    tft_setCursor(65, 70);
    sprintf(buffer," %10d", capture1_tick);
    tft_writeString(buffer);
    tft_setCursor(65, 80);
    sprintf(buffer," %10d", timer_expired);
    tft_writeString(buffer);
    tft_setCursor(65, 90);
    sprintf(buffer," %10d", got_update);
    tft_writeString(buffer);
    tft_setCursor(65, 100);
    sprintf(buffer," %10d", reading_status);
    tft_writeString(buffer);
    tft_setCursor(65, 110);
    sprintf(buffer," %10d", sys_time_seconds);
    tft_writeString(buffer);
#endif   
}
// Cap Measure and display thread
static PT_THREAD(protothread_measure(struct pt *pt))
{
    PT_BEGIN(pt);
    tft_initialise();                               // Initialise TFT
    while (1) {
        // Discharge Cap
        mPORTBSetPinsDigitalOut(BIT_3);             //Set Pin7 C1INA to output LOW
        mPORTBClearBits(BIT_3);
        PT_YIELD_TIME_msec(DISCHARGE_MS);           // Wait >= 5RC to discharge
        
        // Charge Cap
        got_update = 0;                             // Reset measurement flags
        timer_expired = 0;
        // Start timer before charging otherwise may capture old value before 
        // we get a chance to reset.
        WriteTimer3(0);                             // Reset Timer 3 - Start counting from 0
        mPORTBSetPinsDigitalIn(BIT_3);              // Set Pin7 C1INA to high impedance input pin. Charging starts.
        mT3ClearIntFlag();                          // Clear any outstanding timer3 interrupts
        EnableIntT3;                                // Enable timer3 interrupts so we get notified if timer expires
        PT_YIELD_UNTIL(pt, got_update | timer_expired); // Wait for measurement or timer expiration.
        DisableIntT3;                               // Stop timer3 interrupts so timer_expired flag won't get set after here. 
        
        // Process measurement result
        if (timer_expired) {                        // No reading. Timer expired
            capture_ns = 0xFFFF * tick_ns;          // Update so looks right on display
            capacitance = 0.0;
            reading_status = 3;                     // No good. Timer expired
            led_colour = ILI9340_RED;
        } else {                                    // Got a reading
            //capture1_tick = mIC1ReadCapture();
            capture_ns = capture1_tick * tick_ns;
            if (capture1_tick < MIN_TICKS) {        // Reading too low
                capacitance = 0.0;
                reading_status = 2;                 // Too low. No Cap!
                led_colour = ILI9340_CYAN;
            } else {                                // Yeah baby! We got something useful.
                // Work out capacitance value based on capture time
                //capacitance = (capture_ns * 1.0e-9) / (R * -logf((VS-VREF)/VS));
                capacitance = (capture_ns) / (R * -logf((VS-VREF)/VS)); // nF
                reading_status = 1;                 // Got a good reading
                led_colour = ILI9340_GREEN;
            }   
        }
        tft_refresh_values();                       // Update Display
        PT_YIELD_TIME_msec(PAUSE_MS);               // Wait a bit before taking another reading
    }
    PT_END(pt);
}

// One Second Thread
// Flash a fake LED on the TFT
static PT_THREAD(protothread_led(struct pt *pt))
{
    static unsigned char led_on = 0;
    PT_BEGIN(pt);
    while (1) {
        PT_YIELD_TIME_msec(1000);
        sys_time_seconds++;
        // Toggle led_on and off
        if (led_on ^= 1) {
            tft_fillCircle(120,160, 16, led_colour); //x, y, radius, color
        } else {
            tft_fillCircle(120,160, 16, ILI9340_BLACK); //x, y, radius, color
        }
    }
    PT_END(pt);
} // led thread

int main(void)
{
    ANSELA = 0;                             // Make all Pins Digital
    ANSELB = 0; 
    
    //set up compare 1
    // Use Vref as -ve input
    // Pin 7: +ve input
    // Pin 18: Compare result output
    CMP1Open(CMP_ENABLE | CMP_OUTPUT_ENABLE | CMP1_NEG_INPUT_IVREF);
    PPSOutput(4, RPB9, C1OUT);              // Output result on pin18
    mPORTBSetPinsDigitalIn(BIT_3); //Set port as input (pin 7 is RB3)
    
    // Timer 3 - Max range
    // Value captured on input capture IC1
    // PBCLK = 40MHz
    // PB Period = 25 ns
    // Precaler = 1:2
    // Tick Period = 50ns
    // Timer Range (ms) = 0..3.3
    // Set interrupt to signal expired timer
    OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_2, 0xffff);
    ConfigIntTimer3(T3_INT_PRIOR_2);        // Setup with interrupts initially disabled.
    mT3ClearIntFlag();
    tick_ns = 50;                           // Set tick period (ns) based on above Timer3 configuration
    
    // Setup input capture
    // Pin 24: Rising edge
    // Capture timer3 value
    OpenCapture1( IC_CAP_16BIT | IC_EVERY_RISE_EDGE | IC_INT_1CAPTURE | IC_TIMER3_SRC | IC_ON );
    ConfigIntCapture1( IC_INT_ON | IC_INT_PRIOR_3 | IC_INT_SUB_PRIOR_3 );
    mIC1ClearIntFlag();
    PPSInput(3, IC1, RPB13);                // PPS group 3, Pin 24
    mPORTBSetPinsDigitalIn(BIT_13);         // Set Pin 24 as input
    
    PT_setup();
    
    // Init threads
    PT_INIT(&pt_measure);
    PT_INIT(&pt_led);

    while (1) {
        PT_SCHEDULE(protothread_measure(&pt_measure));
        PT_SCHEDULE(protothread_led(&pt_led));
    }
}