/*
 * Lab02 - DTMF Dailer
 * https://people.ece.cornell.edu/land/courses/ece4760/labs/f2016/lab2_dtmf.html
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

#define VERSION "v0.2"

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

// protothread
static struct pt pt_led, pt_keypad;

// System time since startup (secs)
int sys_time_seconds = 0;
short led_colour = ILI9340_GREEN;

// string buffer for TFT output
char buffer[60];

#define MAX_DIGITS 15
unsigned char digit_buffer[MAX_DIGITS];
int digit_cnt;                       // Number of digits in the buffer
unsigned char buttons[] = "0123456789*#";
int keypad_codes[] = {         0x0108,
                       0x0081, 0x0101, 0x0201,
                       0x0082, 0x0102, 0x0202,
                       0x0084, 0x0104, 0x0204,
                       0x0088,         0x0208 };
int keypad_index;
int keypad_code;


void clear_digits (void)
{
    digit_cnt = 0;
    digit_buffer[0] = '\0';
    tft_fillRoundRect(0,30, 240, 24, 1, ILI9340_BLUE);// x,y,w,h,radius,color
}

void display_digit (int buf_index)
{
    tft_drawChar(buf_index*10, 34, digit_buffer[buf_index], ILI9340_YELLOW, ILI9340_BLUE, 2); // x,y, char, colour, bg_colour, size
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
    tft_setTextSize(2);
    sprintf(buffer, "DTMF Dialer %s", VERSION);
    tft_writeString(buffer);
    clear_digits();
}

// Scan the keypad
// Returns the keycode
// keycode = 0 if no key pressed
int scan_keypad(void)
{
    static int pattern, i, keycode;
    mPORTAClearBits(  BIT_0 | BIT_1 | BIT_2 | BIT_3 );
    pattern = 1;
    mPORTASetBits(pattern);                     // Light up row 1
    for (i=0; i<4; i++) {
        keycode = mPORTBReadBits( BIT_7 | BIT_8 | BIT_9 );
        if (keycode != 0) {
            keycode |= pattern;            // Found a key pressed
            break;
        }
        mPORTAClearBits(pattern);
        pattern <<= 1;                          // Move to next row
        mPORTASetBits(pattern);                 // Light up next row
    }
    mPORTAClearBits(  BIT_0 | BIT_1 | BIT_2 | BIT_3 );
    return keycode;
}

// Called when a key has been pressed.
void key_pressed(int keycode)
{
    static int i;
    // Search for keycode
    keypad_index = -1;
    for (i=0; i<12; i++) {                      // Search for keypad_code in table
        if (keypad_codes[i] == keycode) {
            keypad_index = i;
            if (keypad_index == 10) {           // '*' key - erase buffer
                clear_digits();
            } else if (digit_cnt != MAX_DIGITS) { // If room add digit to buffer and display
                digit_buffer[digit_cnt] = buttons[keypad_index];
                display_digit(digit_cnt);
                digit_cnt++;
            }
            break;
        }
    }
}

// Keypad input thread
// Screen output
// State machine for keypad press and release debounce
// States
#define KP_RELEASED 0
#define KP_MAYBE_PRESSED 1
#define KP_STILL_PRESSED 2
#define KP_MAYBE_RELEASED 3
static PT_THREAD(protothread_keypad(struct pt *pt))
{
    static int kp_state, scan_code;
    PT_BEGIN(pt);
    tft_initialise();                               // Initialise TFT
    // Setup Keypad pins
    mPORTASetPinsDigitalOut( BIT_0 | BIT_1 | BIT_2 | BIT_3 );
    mPORTBSetPinsDigitalIn( BIT_7 | BIT_8 | BIT_9 );
    EnablePullDownB( BIT_7 | BIT_8 | BIT_9 );
    kp_state = KP_RELEASED;
    scan_code = 0;
    while (1) {
        PT_YIELD_TIME_msec(30);
        switch (kp_state) {
            case KP_RELEASED:                       // State
                if (scan_code==0) {                 // Test
                    kp_state = KP_RELEASED;         // Next state
                } else {
                    kp_state = KP_MAYBE_PRESSED;    // Next state
                    keypad_code = scan_code;        // Transition action
                }
                break;
            case KP_MAYBE_PRESSED:
                if (scan_code == keypad_code) {     // Been pushed to 2 consecutive scans
                    kp_state = KP_STILL_PRESSED;
                    // Key confirmed as pressed - Do key pressed actions
                    key_pressed(keypad_code);
                } else {  
                    kp_state = KP_RELEASED;         // Press still bouncing
                }
                break;
            case KP_STILL_PRESSED:                  // Debounce release
                if (scan_code != 0) {
                    kp_state = KP_STILL_PRESSED;
                } else {
                    kp_state = KP_MAYBE_RELEASED;
                }
                break;
            case KP_MAYBE_RELEASED:
                if (scan_code != 0) {
                    kp_state = KP_STILL_PRESSED;
                } else {
                    kp_state = KP_RELEASED;         // Been released for 2 consecutive scans
                }
                break;
        }
        scan_code = scan_keypad();          // Common transition action 
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
            //tft_fillCircle(120,160, 16, led_colour); //x, y, radius, color
            tft_fillCircle(20,300, 16, led_colour); //x, y, radius, color
        } else {
            tft_fillCircle(20,300, 16, ILI9340_BLACK); //x, y, radius, color
        }
    }
    PT_END(pt);
} // led thread

int main(void)
{
    ANSELA = 0;                             // Make all Pins Digital
    ANSELB = 0;
    
    PT_setup();
    
    // Init threads
    PT_INIT(&pt_keypad);
    PT_INIT(&pt_led);

    while (1) {
        PT_SCHEDULE(protothread_keypad(&pt_keypad));
        PT_SCHEDULE(protothread_led(&pt_led));
    }
}