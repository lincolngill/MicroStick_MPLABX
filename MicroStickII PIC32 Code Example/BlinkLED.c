/*********************************************************************
 *
 *                  LED example file
 *
 *********************************************************************
 * FileName:        BlinkLED.c
 * Dependencies:    plib.h
 *
 * Processor:       PIC32
 *
 * Complier:        MPLAB C32 v2.01 or higher
 *                  MPLAB IDE v8.73 or higher
 * Company:         Microchip Technology Inc.
 *
 * Software License Agreement
 *
 * The software supplied herewith by Microchip Technology Incorporated
 * (the “Company”) for its PIC Microcontroller is intended
 * and supplied to you, the Company’s customer, for use solely and
 * exclusively on Microchip PIC Microcontroller products.
 * The software is owned by the Company and/or its supplier, and is
 * protected under applicable copyright laws. All rights are reserved.
 * Any use in violation of the foregoing restrictions may subject the
 * user to criminal sanctions under applicable laws, as well as to
 * civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *
 * Author	Date		Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * MKatic	7/28/2011	Led should blink on an off.
 * 
 ********************************************************************/
 
#include <plib.h>

#define SYS_FREQ 		(40000000L)
 
 main() {
	SYSTEMConfig(SYS_FREQ, 0);
	
	mPORTAClearBits(BIT_0);					//Clear bits to ensure light is off.
	mPORTASetPinsDigitalOut(BIT_0);			//Set port as output

	int i;
	int j;
	
	while(1) {								//This loop determines the initial time between blinks.
		
		j = 100000;
		
		while (j){							//This loop controlls how quickely blinking speeds up.
			mPORTAToggleBits(BIT_0);		//Toggle light status. (Can be viewed in LATA SFR)
			
			i = j;							//Time to wait in between toggle.
			while(i--) {}					//Kill time.
			
			j = j - 1000;					//Increase constant to increas blinking speed faster.
		}
	}
} 
