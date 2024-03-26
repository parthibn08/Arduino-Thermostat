

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
volatile uint32_t transitions;

// Frequencies for natural notes from middle C (C4)
// up one octave to C5.
uint16_t frequency[8] =
    { 262, 294, 330, 349, 392, 440, 494, 523 };

int main(void) {
    // Initialize DDR and PORT registers and LCD
	DDRC &= ~((1 << PC1) | (1 << PC5));
	PORTC |= (1 << PC1) | (1 << PC5);
	DDRB |= (1 << PB4);
	lcd_init();

    // Write a spash screen to the LCD
	char splashscreen[12];
    unsigned char classNum = 109;
    unsigned char labNum = 6;
    lcd_moveto(0, 0);
    snprintf(splashscreen, 12, "EE%d lab %d", classNum, labNum);
    lcd_stringout(splashscreen);
    lcd_moveto(1, 0);
    lcd_stringout("Haadi Razzak");
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

	//initalize count on LCD
	char count_str[16];
    snprintf(count_str, sizeof(count_str), "%4d", count);
    lcd_stringout(count_str);

	PCICR |= (1 << PCIE1);
	PCMSK1 |= ((1 << PCINT9)| (1 << PCINT13));
	sei();
	timer1_init();
    while (1) {                 // Loop forever

        if (changed) { // Did state change?
	    changed = 0; // Reset changed flag

	    // Output count to LCD
		lcd_moveto(0, 0);
		snprintf(count_str, sizeof(count_str), "%4d", count);
		lcd_stringout(count_str);


	    // Do we play a note?
	    if ((count % 8) == 0) {
		// Determine which note (0-7) to play
		int index = ((abs(count) % 64) / 8);

		// Find the frequency of the note
		int freq = frequency[index];

		// Call play_note and pass it the frequency
		play_note(freq);

	    }
        }
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

	OCR1A = 8000000/freq - 1;
	TCNT1 = 0;
	TCCR1B |= (1 << CS10);

	//uint32_t isr_count = 0;

    // while (isr_count < transitions) {
	// // PORTB |= (1 << PB4);    // Buzzer output high
	// // variable_delay_us(period / 2);  // Delay for half the period
	// // PORTB &= ~(1 << PB4);   // Buzzer output log
	// 	variable_delay_us(period / 2);  // Delay for half the period
	// 	isr_count++;
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

ISR(PCINT1_vect)
{
    // In Task 6, add code to read the encoder inputs and determine the new
    // count value
	int x = PINC;

	// Read the input bits and determine A and B.
	a = (x & (1 << PC1)) >> PC1;
	b = (x & (1 << PC5)) >> PC5; 

	if (old_state == 0) { 
	    // Handle A and B inputs for state 0

		if (!b && a) { // 01 
			new_state = 1;
			changed = 1;
			count++;
		}
		else if (b && !a) { // 10 
			new_state = 2;
			changed = 1;
			count--;
		}

	}
	else if (old_state == 1) { 

	    // Handle A and B inputs for state 1

		if (b && a) { // 11 
			new_state = 3;
			changed = 1;
			count++;
		}
		else if (!b && !a) { // 00
			new_state = 0;
			changed = 1;
			count--;
		}

	}
	else if (old_state == 2) { 

	    // Handle A and B inputs for state 2

		if (!b && !a) { // 01 
			new_state = 0;
			changed = 1;
			count++;
		}
		else if (b && a) { // 11
			new_state = 3;
			changed = 1;
			count--;
		}
	}
	else {   // old_state = 3

	    // Handle A and B inputs for state 3

		if (b && !a) { // 10
			new_state = 2;
			changed = 1;
			count++;
		}
		else if (!b && a) { // 01
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
	TCCR1B |= (1 << WGM12);

	TIMSK1 |= (1 << OCIE1A);
}
ISR(TIMER1_COMPA_vect)
{
    // In Task 7, add code to change the output bit to the buzzer, and to turn
    // off the timer after enough periods of the signal
	if (transitions == 0) {
        TCCR1B &= ~((0b111 << CS10));
    }
	else {
		PORTB ^= (1 << PB4);
		transitions--;
	}

}
