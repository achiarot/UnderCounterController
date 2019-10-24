/*
 * UndercabinetLEDController.cpp
 *
 * Created: 2019-10-12 2:45:41 PM
 * Author : antho
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

/* Configuration settings
	The following will include tuning parameters for LED max and min brightness for your  specific setup.
	PB3 - Input pin for on-time configuration
	PB4 - Input pin for Brightness control
	PB2 - Input pin for PIR sensor
	PB1 - PWM output for the LED driver
*/
#define TIMESW			PINB3
#define BRSW			PINB4
#define PIR				PINB2
#define LED				PINB1
#define LEDBR			OCR0B

#define MAXPWM			255
#define MINPWM			0
#define BR_STEP			25												//Brightness decrease per button press (must be positive)
#define P_FADE			3												//Seconds to fade LED on and off
#define MAXTIME_SEC		600												//Maximum amount of time on in seconds
#define MINTIME_SEC		10												//Minimum amount of time on in seconds
#define ONTIME_STEP_SEC	30												//Time on increase per button press (must be positive)
/* End of Configuration */

#define F_CPU			1200000LL
#define PRESCALER		1
#define F_PWM			((F_CPU/256)/PRESCALER)		
#define ONTIME_TICK_SEC	10												//Resolution of the onTime counter
#define P_OVFL			(ONTIME_TICK_SEC*F_PWM)							//Length of each ticks on the time counter
#define	COUNTDIVFADE	((P_FADE*F_PWM)/(MAXPWM))						//Calculated divisor to meet the fade period
#define COUNTBLINK		(F_PWM/5)										//Half a second blink
#define MAXTIME			(MAXTIME_SEC/ONTIME_TICK_SEC)
#define MINTIME			(MINTIME_SEC/ONTIME_TICK_SEC)
#define ONTIME_STEP		(uint8_t)(ONTIME_STEP_SEC/ONTIME_TICK_SEC)		//number of onTime ticks to last for
#define BR_ADDR			(uint8_t*)0x01									//address to store the current brightness in EEPROM for non volatile storage
#define OT_ADDR			(uint8_t*)0x02									//EEPROM address for onTime

/* Globally shared variable
	This includes variables shared or used by ISR's
*/
//
uint8_t PIRFlag;
volatile uint8_t BRFlag;
volatile uint8_t OTFlag;
volatile uint8_t ALWAYSONFlag;
									
volatile uint8_t lastSwitchState;

volatile uint8_t brightness;
volatile uint8_t onTime;										//LED on-time in 10 second increments

volatile uint16_t counterOverflow;								//Used for keeping track of on-time since we only have one timer
volatile uint8_t onCount;										//used for counting the 10 second intervals

int startTimer(){
	counterOverflow=0;											//Zero out the counters used to keep on time
	onCount=0;

	return 1;
}


int checkPIR(){
	if((PINB & (1<<PIR)) && !PIRFlag){
		PIRFlag = 1;
		startTimer();
		while(counterOverflow/COUNTDIVFADE < brightness){
			LEDBR = (255 - counterOverflow/COUNTDIVFADE);
		}
	}
	if(PINB & (1<<PIR)){
		startTimer();
	}
	if(PIRFlag){
		LEDBR = 255 - brightness;
	}
	if(PIRFlag && (onCount >= onTime) && !(onTime==MAXTIME)){
		PIRFlag = 0;
		startTimer();
		while(counterOverflow/COUNTDIVFADE < brightness){
			LEDBR = (255 - brightness + counterOverflow/COUNTDIVFADE);
		}
		LEDBR = 255;
	}
	if(ALWAYSONFlag){
		for(uint8_t i=0; i<2; i++){
			startTimer();
			LEDBR = 255;
			while(counterOverflow<COUNTBLINK ){}
			startTimer();
			LEDBR = 255-brightness;
			while(counterOverflow<COUNTBLINK ){}
		}
		ALWAYSONFlag=0;
	}
	return 1;
}

int main(void)
{
	//Setup inputs and outputs, as well as timers, PWM and interrupts
	DDRB = (DDRB | (1<<LED)) & ~((1<<PIR)|(1<<TIMESW)|(1<<BRSW));				//Set outputs and inputs
	PORTB = (PORTB | (1<<TIMESW) | (1<<BRSW)) & ~((1<<PIR)|(1<<LED));			//Set pullups for the buttons and explicitly not for the PIR
	
	TCCR0A |= (1<<COM0B1)|(1<<COM0B0)|(1<<WGM00)|(1<<WGM01);								//Set the timer for fast PWM
	TCCR0B |= (1<<CS00);												//Set the clock with no prescaler
    
	//Setup Pin change interrupt
	GIMSK |= (1<<PCIE);															//Enable in mask register
	PCMSK |= (1<<TIMESW)|(1<<BRSW);												//Mask all pins other than the two buttons and PIR
	
	/* DEBUGGING*/
	DDRB |= (1<<PINB0); //Make PB0 an output
	/* DEBUGGING End*/
	
	brightness = eeprom_read_byte(BR_ADDR);							//Read the stored value for brightness
	if(!((brightness>=MINPWM) && (brightness<=MAXPWM))){						//Sanity check the value, if not in the acceptable bounds, set to maximum value
		brightness = MAXPWM;
	}
	onTime = eeprom_read_byte(OT_ADDR);	
	if(!((onTime>=MINTIME) && (onTime<=MAXTIME))){								//Sanity check the value, if not in the acceptable bounds, set to minimum value
		onTime = MINTIME;
	}		
	counterOverflow=0;															//Zero out the counters used to keep on time
	onCount=0;
	PIRFlag=0;
	BRFlag=0;
	OTFlag=0;
	ALWAYSONFlag=0;
	lastSwitchState=0;

	LEDBR = 255;	
	TIMSK0 |= (1<<TOIE0);															//Default LED on-time is 30 seconds
	
	//Start interrupts
	sei();
	
    while (1)
    {
		checkPIR();
		if(BRFlag){
			eeprom_write_byte(BR_ADDR, brightness);
			BRFlag = 0;
		}
		if(OTFlag){
			eeprom_write_byte(OT_ADDR, onTime);
			OTFlag = 0;
		}
    }
}

ISR(TIM0_OVF_vect){
	
	if(counterOverflow>=P_OVFL){
		counterOverflow = 0;
		
		if(onCount>=onTime){
			onCount=onTime;											//Hold at the max time so you can deal with it in the main function
		}
		else{
			onCount++;
		}
	}
	
	else{
		counterOverflow++;
	}
}

ISR(PCINT0_vect, ISR_NOBLOCK){
	
	uint8_t state = PINB;						//Read the state of the port immediately
	
	/*DEBUGGING*/
	PORTB |= (1<<PINB0);
	/*DEBUGGING END*/
	
	//Split into the two possible interrupt sources and handle each
	if((~state & (1<<TIMESW)) && (lastSwitchState & (1<<TIMESW))){
		//The time switch has been pressed
		if(onTime == MAXTIME){
			onTime = MINTIME;
		}
		else if(onTime >= ((MAXTIME-ONTIME_STEP))){
			onTime = MAXTIME;
			ALWAYSONFlag = 1;
		}
		else{
			onTime += ONTIME_STEP;
		}
		OTFlag=1;
	}
	
	if((~state & (1<<BRSW)) && (lastSwitchState & (1<<BRSW))){
		//The brightness has been adjusted
		if(brightness == 0){
			brightness = MAXPWM;
		}
		else if(brightness == MINPWM){
			brightness = 0;
		}
		else if(brightness<=(BR_STEP+MINPWM)){
			brightness = MINPWM;
		}
		else{ 
			brightness -= BR_STEP;
		}
		BRFlag = 1;
	}
	lastSwitchState=state;
	/*DEBUGGING*/
	PORTB &= ~(1<<PINB0);
	/*DEBUGGING END*/
}

