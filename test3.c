/********************************************
 *
 *  Name: Parthib Nag
 *  Email: pnag@usc.edu
 *  Section: TTh 2-3:20
 *  Assignment: Final Project - Thermostat
 *
 ********************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>


#include "lcd.h"
#include "ds18b20.c"

void play_note(uint16_t);
void variable_delay_us(int16_t);
volatile uint8_t changed = 0;  // Flag for state change
volatile uint8_t new_state, old_state;
volatile uint8_t ambTempWhole;
volatile uint8_t ambTempDec;
volatile uint8_t low = 60;
volatile uint8_t high = 90;
volatile uint8_t prevLow = 0;
volatile uint8_t prevHigh= 0;
volatile uint8_t a, b;
volatile int lowChanged = 0;
volatile int adjHiLow = 2;
volatile uint32_t transitions;
volatile uint16_t width = 23;
volatile uint8_t prevAmbWhole= 0;
volatile uint8_t playing = 0;
volatile uint32_t buzzer = 175;
volatile int buzzerFlag = 0;

int16_t convertToFahrenheit(uint8_t msb, uint8_t lsb) {
    int16_t temperature = (msb << 8);
	temperature |= lsb;
	temperature *= 10;
	temperature = temperature >> 4;
    // Perform desired arithmetic operation (convert to Fahrenheit)
    temperature = temperature * 9;
	temperature = temperature / 5;
	temperature = temperature + (320);

    return temperature;
}

// Function to map temperature to OCR2A using a linear equation
int16_t temperatureToOCR2A(int temperature) {
    int minTemperature = 100;
    int maxTemperature = 40;
    int minOCR2A = 11;
    int maxOCR2A = 35;

    int16_t slope = ((maxOCR2A - minOCR2A) << 8) / (minTemperature - maxTemperature);
    int16_t yIntercept = ((int32_t)minOCR2A << 8) - ((int32_t)slope * maxTemperature);

    int16_t result = (slope * temperature + yIntercept) >> 8;

    return 46 - result - 4;
}


// Frequencies for natural notes from middle C (C4)
// up one octave to C5.
uint16_t frequency[8] = { 262, 294, 330, 349, 392, 440, 494, 523 };


int main(void) {

	width = 23;

    // Initialize DDR and PORT registers and LCD
	DDRC &= ~((1 << PC1) | (1 << PC2)); //Rotary Encoder Initialization
	PORTC |= (1 << PC1) | (1 << PC2);

	PORTC |= (1 << PC4);

    PORTD |= (1 << PD2);
    PORTD |= (1 << PD3);

	DDRB |= (1 << PB3);

	PORTC |= ((1 << PC3)|(1 << PC4)|(1 << PC5));

	DDRC |= (1 << PC3);
	DDRC |= (1 << PC4);
	DDRC |= (1 << PC5);

	DDRB |= (1 << PB5);
	
	unsigned char t[2];

    if (ds_init() == 0) {    // Initialize the DS18B20
        // Sensor not responding
    }

    ds_convert();    // Start first temperature conversion
	lcd_init();
	lcd_writecommand(1);
    // Write a spash screen to the LCD
	char splashscreen[15];
    lcd_moveto(0, 0);
    snprintf(splashscreen, 15, "Final Project");
    lcd_stringout(splashscreen);
    lcd_moveto(1, 0);
    lcd_stringout("Parthib Nag");
    _delay_ms(2000);
    lcd_writecommand(1);

	// ds_init();
	// ds_convert();
    // Read the A and B inputs to determine the intial state.
    // In the state number, B is the MSB and A is the LSB.
    // Warning: Do NOT read A and B separately.  You should read BOTH inputs
    // at the same time, then determine the A and B values from that value.    
	a = (PINC & (1 << PC1)) >> PC1;
    b = (PINC & (1 << PC2)) >> PC2;  

    if (!b && !a)
	old_state = 0;
    else if (!b && a)
	old_state = 1;
    else if (b && !a)
	old_state = 2;
    else
	old_state = 3;

    new_state = old_state;

	unsigned char lowVal;
	unsigned char highVal;
	lowVal = eeprom_read_byte((void *) 100);
	// if(lowVal < 50 || lowVal > 90 || lowVal > highVal){
	// 	lowVal = 50;
	// }
	highVal = eeprom_read_byte((void *) 200);
	if(highVal < 50 || highVal > 90 || highVal < lowVal){
		highVal = 90;
	}
	low = lowVal;
	high = highVal;
	
	PCICR |= (1 << PCIE1);
	PCMSK1 |= ((1 << PCINT9)| (1 << PCINT10));
	timer1_init();
	timer0_init();
	timer2_init();
	sei();
	


    
    char ambTemp[16];
    snprintf(ambTemp, sizeof(ambTemp), "Temp:");
    lcd_moveto(0,0);
    lcd_stringout(ambTemp);

    char lowTemp[7];
    snprintf(lowTemp, sizeof(lowTemp), "Low=%2d", low);
    lcd_moveto(1,0);
    lcd_stringout(lowTemp);

    char HighTemp[8];
    snprintf(HighTemp, sizeof(HighTemp), "High=%2d", high);
    lcd_moveto(1,8);
    lcd_stringout(HighTemp);

	char widthArr[4];
    snprintf(widthArr, sizeof(widthArr), "%2d", width);
    lcd_moveto(0,11);
    lcd_stringout(widthArr);

	// char Yes[1];
    


	int16_t temperature;
	
    while (1) {                 // Loop forever

		// snprintf(Yes, sizeof(Yes), "%d", playing);
		// lcd_moveto(1,14);
		// lcd_stringout(Yes);

		if(ds_temp(t)){    // True if conversion complete
            temperature = convertToFahrenheit(t[1], t[0]);
			ambTempWhole = temperature / 10;
			ambTempDec = temperature % 10;
			snprintf(ambTemp, sizeof(ambTemp), "Temp:%2d.%d", ambTempWhole, ambTempDec);
    		lcd_moveto(0,0);
    		lcd_stringout(ambTemp);
			width = temperatureToOCR2A(ambTempWhole);
			// OCR2A = width;
			if(prevAmbWhole != ambTempWhole){
				width = temperatureToOCR2A(ambTempWhole);
				prevAmbWhole = ambTempWhole;
			}
            ds_convert();   // Start next conversion
        }

		if((PIND & (1 << PD2)) == 0){ //low button
			// width = temperatureToOCR2A(low);
			// OCR2A = width;
			adjHiLow = 1;
		}
		if((PIND & (1 << PD3)) == 0){ //high button
			// width = temperatureToOCR2A(high);
			// OCR2A = width;
			adjHiLow = 0;
		}

		if(adjHiLow == 0){
			snprintf(lowTemp, 8, "Low=%2d", low);
			lcd_moveto(1,0);
			lcd_stringout(lowTemp);
			snprintf(HighTemp, 8, "High?%2d", high);
			lcd_moveto(1,8);
			lcd_stringout(HighTemp);
			if(high <= low){
				high = low;
			}
			if(high != prevHigh){
				snprintf(HighTemp, 8, "High?%2d", high);
				lcd_moveto(1,8);
				lcd_stringout(HighTemp);
				prevHigh = high;
				eeprom_update_byte((void *) 200, high);
				width = temperatureToOCR2A(high);
				if(width >= 35){
					width = 35;
					snprintf(widthArr, sizeof(widthArr), "%2d", width);
					lcd_moveto(0,11);
					lcd_stringout(widthArr);
					prevAmbWhole = ambTempWhole;
					OCR2A = width;
				}
				else{
					snprintf(widthArr, sizeof(widthArr), "%2d", width);
					lcd_moveto(0,11);
					lcd_stringout(widthArr);
					prevAmbWhole = ambTempWhole;
					width = width;
				}
			}
		}
		else if(adjHiLow == 1){
			snprintf(HighTemp, 8, "High=%2d", high);
			lcd_moveto(1,8);
			lcd_stringout(HighTemp);
			snprintf(lowTemp, 8, "Low?%2d", low);
			lcd_moveto(1,0);
			lcd_stringout(lowTemp);
			if(low >= high){
				low = high;
			}
			if(low != prevLow){
				snprintf(lowTemp, 8, "Low?%2d", low);
				lcd_moveto(1,0);
				lcd_stringout(lowTemp);
				prevLow = low;
				eeprom_update_byte((void *) 100, low);
				width = temperatureToOCR2A(low);
				if(width <= 11){
					width = 11;
					snprintf(widthArr, sizeof(widthArr), "%2d", width);
					lcd_moveto(0,11);
					lcd_stringout(widthArr);
					prevAmbWhole = ambTempWhole;
					
				}
				else{
					snprintf(widthArr, sizeof(widthArr), "%2d", width);
					lcd_moveto(0,11);
					lcd_stringout(widthArr);
					prevAmbWhole = ambTempWhole;
					width = width;
				}
			}
		}

		if(ambTempWhole < low - 3 && buzzerFlag == 0){
			TCCR0B |= (1 << CS12);
			buzzerFlag = 1;
		}
		if(ambTempWhole > high + 3 && buzzerFlag == 0){
			TCCR0B |= (1 << CS12);
			buzzerFlag = 1;
		}
		if(ambTempWhole + 3 <= high && ambTempWhole - 3 >= low){
			buzzerFlag = 0;
		}
		if(ambTempWhole < low){
			PORTC &= ~(1 << 3);
			PORTC |= (1 << 5);
			PORTC |= (1 << 4);
		}
		else if(ambTempWhole > high){
			PORTC &= ~(1 << 5);
			PORTC |= (1 << 3);
			PORTC |= (1 << 4);
		}
		else if(ambTempWhole >= low && ambTempWhole <= high){
			PORTC &= ~(1 << 4);
			PORTC |= (1 << 5);
			PORTC |= (1 << 3);
		}
		
		snprintf(widthArr, sizeof(widthArr), "%2d", width);
		lcd_moveto(0,11);
		lcd_stringout(widthArr);
    }
}

ISR(PCINT1_vect){
    // In Task 6, add code to read the encoder inputs and determine the new
	int x = PINC;
	a = (x & (1 << PC1)) >> PC1;
	b = (x & (1 << PC2)) >> PC2;
    
	if(adjHiLow == 1){ //adjusting low temp
		if (old_state == 0) { 
			if (a){
				if(low >= high){
					low = low;
					new_state = 1;
				}
				else{
					new_state = 1;
					low++;
				}
			}
			else if (b){
				if(low <= 50){
					low = 50;
					new_state = 2;
				}
				else{
					new_state = 2;
					low--;
				}
			}
		}
		else if (old_state == 1) { 
			if (b){
				if(low >= high){
					low = high;
					new_state = 3;
				}
				else{
					new_state = 3;
					low++;
				}
			}
			else if (!a){
				if(low <= 50){
					low = 50;
					new_state = 0;
				}
				else{
					new_state = 0;
					low--;
				}
			}
		}
		else if (old_state == 2) { 
			if (!b){  
				if(low >= high){
					low = high;
					new_state = 0;
				}
				else{
					new_state = 0;
					low++;
				}
			}
			else if (a){
				if(low <= 50){
					low = 50;
					new_state = 3;
				}
				else{
					new_state = 3;
					low--;
				}
			}
		}
		else{   
			if (!a){ 
				if(low >= high){
					low = high;
					new_state = 2;
				}
				else{
					new_state = 2;
					low++;
				}
			}
			else if (!b){
				if(low <= 50){
					low = 50;
					new_state = 1;
				}
				else{
					new_state = 1;
					low--;
				}
			}
		}
	}
	
	if(adjHiLow == 0){
		if (old_state == 0) { 
			if (a){ 
				if(high >= 90){
					high = 90;
					new_state = 1;
				}
				else{
					new_state = 1;
					high++;
				}
			}
			else if (b){ 
				if(high <= low){
					high = low;
					new_state = 2;
				}
				else{
					new_state = 2;
					high--;
				}
			}
		}
		else if (old_state == 1) { 
			if (b){ 
				if(high >= 90){
					high = 90;
					new_state = 3;
				}
				else{
					new_state = 3;
					high++;
				}
			}
			else if (!a){
				if(high <= low){
					high = low;
					new_state = 0;
				}
				else{
					new_state = 0;
					high--;
				}
			}
		}
		else if (old_state == 2) { 
			if (!b){  
				if(high >= 90){
					high = 90;
					new_state = 0;
				}
				else{
					new_state = 0;
					high++;
				}
			}
			else if (a){ 
				if(high <= low){
					high = low;
					new_state = 3;
				}
				else{
					new_state = 3;
					high--;
				}
			}
		}
		else {   
			if (!a) {
				if(high >= 90){
					high = 90;
					new_state = 2;
				}
				else{
					new_state = 2;
					high++;
				}
			}
			else if (!b) {
				if(high <= low){
					high = low;
					new_state = 1;
				}
				else{
					new_state = 1;
					high--;
				}
			}
		}
	}

	if (new_state != old_state) {
		old_state = new_state;
	}
}

/*
  Play a tone at the frequency specified for one second
*/
void play_note(uint16_t freq)
{

	//uint32_t period = 1000000 / freq;;
	transitions = freq * 2;

	//uint16_t ocr1aVal = (16000000 / (2 * freq));

	OCR0A = 8000000/freq - 1;
	TCNT1 = 0;
	TCCR0B |= (1 << CS00);

    // uint32_t period;

    // period = 1000000 / freq;    // Period of note in microseconds

    // while (freq--) {
	// 	PORTB |= (1 << PB4);    // Buzzer output high
	// 	variable_delay_us(period / 2);  // Delay for half the period
	// 	PORTB &= ~(1 << PB4);   // Buzzer output log
	// 	variable_delay_us(period / 2);  // Delay for half the period
	// }
}

/*
    variable_delay_us - Delay a variable number of microseconds
*/
void variable_delay_us(int delay)
{
    int i = (delay + 5) / 10;

    while (i--)
        _delay_us(10);
}

void timer0_init(){
	TCNT0 = 0;
	TCCR0B |= (1 << WGM02);
	TIMSK0 |= (1 << OCIE0A);
	OCR0A = 8000000 / 349 - 1;
}

ISR(TIMER0_COMPA_vect)
{
	if (buzzer == 0)
	{
		TCCR0B &= ~(1 << CS12); // timer0 off
		buzzer = 175;
	}
	else
	{
		PORTB ^= (1 << PB5);
		buzzer--;
	}
}

void timer1_init()
{
   
}

ISR(TIMER1_COMPA_vect)
{
    // In Task 7, add code to change the output bit to the buzzer, and to turn
    // off the timer after enough periods of the signal

}

void timer2_init(void){
	TIMSK1 |= (1 << OCIE1A);
    TCCR2A |= (0b11 << WGM20);  // Fast PWM mode, modulus = 256
	//TCCR2B &= ~(1 << WGM22);
    TCCR2A |= (0b10 << COM2A0); // Turn D11 on at 0x00 and off at OCR2A
    // OCR2A = 256;
	// OCR2A = 12;               // Initial pulse duty cycle of 50%
    TCCR2B |= (0b111 << CS20);  // Prescaler = 1024 for 16ms period
	
}

ISR(TIMER2_COMPA_vect)
{
    OCR2A = width;
}



