/********************************************
 *
 *  Name: Parthib Nag
 *  Email: pnag@usc.edu
 *  Section: TTh 2-3:20
 *  Assignment: Project: Thermostat
 *
 ********************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>

#include "lcd.h"

void play_note(uint16_t);
void variable_delay_us(int16_t);
volatile uint8_t changed = 0;  // Flag for state change
volatile uint8_t new_state, old_state;
volatile int16_t count = 0;		// Count to display
volatile uint8_t a, b;

// Frequencies for natural notes from middle C (C4)
// up one octave to C5.
uint16_t frequency[8] =
    { 262, 294, 330, 349, 392, 440, 494, 523 };

int main(void) {

    // Initialize DDR and PORT registers and LCD
	
	lcd_init();

    // Write a spash screen to the LCD
	lcd_writecommand(1);
	char splashscreen[15];
    unsigned char cNum = 109;
    unsigned char lNum = 7;
    lcd_moveto(0, 0);
    snprintf(splashscreen, 15, "EE Project");
    lcd_stringout(splashscreen);
    lcd_moveto(1, 0);
    lcd_stringout("Parthib Nag");
    _delay_ms(2000);
    lcd_writecommand(1);

    // Read the A and B inputs to determine the intial state.
    // In the state number, B is the MSB and A is the LSB.
    // Warning: Do NOT read A and B separately.  You should read BOTH inputs
    // at the same time, then determine the A and B values from that value.    
	a = (PINC & (1 << PC1)) >> PC1;
    b = (PINC & (1 << PC5)) >> PC5;  

    if (!b && !a)
	old_state = 0;
    else if (!b && a)
	old_state = 1;
    else if (b && !a)
	old_state = 2;
    else
	old_state = 3;

    new_state = old_state;

	char count_str[16];
    snprintf(count_str, sizeof(count_str), "%4d", count);
    lcd_stringout(count_str);

	PCICR |= (1 << PCIE1);
	PCMSK1 |= ((1 << PCINT9)| (1 << PCINT13));
	sei();

    while (1) {                 // Loop forever
		
		if (changed) { // Did state change?
			changed = 0; // Reset changed flag

			// Output count to LCD
			lcd_moveto(0, 0);
			snprintf(count_str, sizeof(count_str), "%4d", count);
			lcd_stringout(count_str);
		}
    }
}

/*
  Play a tone at the frequency specified for one second
*/
void play_note(uint16_t freq)
{
    uint32_t period;

    period = 1000000 / freq;    // Period of note in microseconds

    while (freq--) {
		PORTB |= (1 << PB4);    // Buzzer output high
		variable_delay_us(period / 2);  // Delay for half the period
		PORTB &= ~(1 << PB4);   // Buzzer output log
		variable_delay_us(period / 2);  // Delay for half the period
	}
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

ISR(PCINT1_vect)
{
    // In Task 6, add code to read the encoder inputs and determine the new
		int x = PINC;
		a = (x & (1 << PC1)) >> PC1;
		b = (x & (1 << PC5)) >> PC5;

		if (old_state == 0) { 
			// Handle A and B inputs for state 0
			if (a) { // 01 
				new_state = 1;
				changed = 1;
				count++;
			}
			else if (b) { // 10 
				new_state = 2;
				changed = 1;
				count--;
			}
		}
		else if (old_state == 1) { 
			// Handle A and B inputs for state 1
			if (b) { // 11 
				new_state = 3;
				changed = 1;
				count++;
			}
			else if (!a) { // 00
				new_state = 0;
				changed = 1;
				count--;
			}
		}
		else if (old_state == 2) { 
			// Handle A and B inputs for state 2
			if (!b) { // 00 
				new_state = 0;
				changed = 1;
				count++;
			}
			else if (a) { // 11
				new_state = 3;
				changed = 1;
				count--;
			}
		}
		else {   // old_state = 3
			// Handle A and B inputs for state 3
			if (!a) { // 10
				new_state = 2;
				changed = 1;
				count++;
			}
			else if (!b) { // 01
				new_state = 1;
				changed = 1;
				count--;
			}
		}
		// If state changed, update the value of old_state,
		// and set a flag that the state has changed.
		if (new_state != old_state) {
			changed = 1;
			old_state = new_state;
		}
}


void timer1_init()
{
    // In Task 7, add code to inititialize TIMER1, but don't start it counting
	// Set to CTC mode
    TCCR1B |= (1 << WGM12);

    // Enable Timer Interrupt
    TIMSK1 |= (1 << OCIE1A);
}

ISR(TIMER1_COMPA_vect)
{
    // In Task 7, add code to change the output bit to the buzzer, and to turn
    // off the timer after enough periods of the signal

}
