/*
 * harkkaMEGA.c
 *
 * Created: 23.4.2020 14.59.55
 * Author : Elias, Aleksi, Aku
 */

// State machine macros defined.
#define ALARM_BUZZING 0
#define ARMED 1
#define DISAMERD 2
#define CHANGE_PW 3

// SPI communications defined.
#define MOTION_DETECTED 1
#define NO_MOTION 0

// Display message macros defined.
#define MSG_ALARM_BUZZING 0
#define MSG_ARMED 1
#define MSG_DISAMERD 2
#define MSG_CHANGE_PW 3
#define MSG_PW_CORRECT 4
#define MSG_PW_INCORRECT 5
#define MSG_PW_TIMEOUT 6

// Timeout macros defined.
#define INPUT_TIMEOUT_STEPS 100

#define F_CPU 16000000UL
#define MaxEepromSize 4096

#include <avr/io.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>
// LCD Library by Peter Fleury.
#include "lcd.h"
// Keypad Library by https://www.exploreembedded.com/wiki/AVR_C_Library
#include "keypad.h"

int8_t g_state;
bool pwm_timer_on = false;
bool wrong_pass = false;
uint16_t input_timeout_step = 0;
char keypad_input[5];
char key_input[2];
char password[5] = "1234";
int8_t keypad_input_index = 0;
int revived = NO_MOTION;
int counter = 0;

void display_message_vol2(int msg_number);
void spi_init();
int spi_communicate();
void led_init();
void led_on();
void led_off();
void init_pwm_timer();
void turn_off_pwm_timer();
void turn_on_pwm_timer();
bool compare_password();
void reset_input();
void log_in();
void change_password();
void take_user_input();

int main(void)
{
	// Enabling global interrupts.
	sei();
	// LCD, SPI and Keypad initialization.
	lcd_init(LCD_DISP_ON);
	KEYPAD_Init();
	spi_init();
	led_init();

	/* Init timer for PWM. */
	init_pwm_timer();
	memset(keypad_input, ' ', 4);
	g_state = ARMED;
	led_on();

	while (1)
	{
		// This is to ensure user doesn't take too long to input stuff.
		input_timeout_step++;

		key_input[0] = KEYPAD_GetKey();

		if (!(0 == strcmp((char *)&key_input[0], "z")))
		{
			take_user_input();
		}

		if (input_timeout_step > INPUT_TIMEOUT_STEPS)
		{
			if (keypad_input_index > 0)
			{
				reset_input();
				display_message_vol2(MSG_PW_TIMEOUT);
				_delay_ms(1000);
			}
			else
			{
				reset_input();
			}
		}

		switch (g_state)
		{
		case ALARM_BUZZING:
			display_message_vol2(MSG_ALARM_BUZZING);
			break;
		case ARMED:
			display_message_vol2(MSG_ARMED);
			// Checks if Uno has detected movement.
			revived = spi_communicate();
			if (revived == MOTION_DETECTED)
			{
				g_state = ALARM_BUZZING;
				turn_on_pwm_timer();
			}
			break;
		case DISAMERD:
			display_message_vol2(MSG_DISAMERD);
			break;
		case CHANGE_PW:
			display_message_vol2(MSG_CHANGE_PW);
			break;
		}
		// This is important variable, changes essentially the refresh rate of the main loop.
		_delay_ms(50);
	}
}

void reset_input()
{
	keypad_input_index = 0;
	input_timeout_step = 0;
	strcpy(keypad_input, "    ");
}

void change_password()
{
	strcpy(password, keypad_input);
	lcd_clrscr();
	lcd_puts("Pw changed");
	_delay_ms(500);
	g_state = DISAMERD;
}

void log_in()
{
	reset_input();
	turn_off_pwm_timer();
	led_off();
	g_state = DISAMERD;
}

void take_user_input()
{
	input_timeout_step = 0;

	if (0 == strcmp((char *)&key_input[0], "*"))
	{
		KEYPAD_WaitForKeyRelease();
		if (keypad_input_index > 0)
		{
			keypad_input_index--;
		}
	}
	else if (0 == strcmp((char *)&key_input[0], "A"))
	{
		KEYPAD_WaitForKeyRelease();
	}
	else if (0 == strcmp((char *)&key_input[0], "B"))
	{
		KEYPAD_WaitForKeyRelease();
		if (g_state == DISAMERD)
		{
			g_state = CHANGE_PW;
			reset_input();
		}
	}
	else if (0 == strcmp((char *)&key_input[0], "C"))
	{
		KEYPAD_WaitForKeyRelease();
	}
	else if (0 == strcmp((char *)&key_input[0], "D"))
	{
		KEYPAD_WaitForKeyRelease();
	}
	else if (0 == strcmp((char *)&key_input[0], "#"))
	{
		KEYPAD_WaitForKeyRelease();
		if (g_state == CHANGE_PW)
		{
			if (4 == keypad_input_index)
			{
				// User is logged in and wants to change password
				change_password();
				reset_input();
			}
			else
			{
				reset_input();
			}
		}
		else if (g_state == ARMED)
		{
			if (true == compare_password())
			{
				log_in();
				wrong_pass = false;
			}
			else
			{
				g_state = ALARM_BUZZING;
				wrong_pass = true;
				turn_on_pwm_timer();
				reset_input();
			}
		}
		else if (g_state == ALARM_BUZZING)
		{
			if (true == compare_password())
			{
				log_in();
				wrong_pass = false;
			}
		}
		else if (g_state == DISAMERD)
		{
			if (true == compare_password())
			{
				g_state = ARMED;
				led_on();
				reset_input();
			}
		}
	}
	else
	{
		keypad_input[keypad_input_index] = KEYPAD_GetKey();
		KEYPAD_WaitForKeyRelease();
		if (keypad_input_index < 4)
		{
			keypad_input_index++;
		}
	}
}

/* New function for displaying messages and taking one argument.
keypad_input_index is used to figure out if user has typed anything.*/
void display_message_vol2(int msg_number)
{
	lcd_clrscr();
	switch (msg_number)
	{
	case MSG_PW_CORRECT:
		lcd_puts("Pw correct");
		break;
	case MSG_PW_INCORRECT:
		lcd_puts("Pw incorrect");
		break;
	case MSG_PW_TIMEOUT:
		lcd_puts("Pw timed out");
		break;
	case MSG_ARMED:
		if (1 == keypad_input_index)
		{
			lcd_puts("Alarm armed");
			lcd_gotoxy(0, 1);
			lcd_puts("User input: *");
		}
		else if (2 == keypad_input_index)
		{
			lcd_puts("Alarm armed");
			lcd_gotoxy(0, 1);
			lcd_puts("User input: **");
		}
		else if (3 == keypad_input_index)
		{
			lcd_puts("Alarm armed");
			lcd_gotoxy(0, 1);
			lcd_puts("User input: ***");
		}
		else if (4 == keypad_input_index)
		{
			lcd_puts("Alarm armed");
			lcd_gotoxy(0, 1);
			lcd_puts("User input: ****");
		}
		else if (0 == keypad_input_index)
		{
			lcd_puts("Alarm armed");
			lcd_gotoxy(0, 1);
			lcd_puts("User input:");
		}
		break;
	case MSG_DISAMERD:
		if (1 == keypad_input_index)
		{
			lcd_puts("Alarm disarmed");
			lcd_gotoxy(0, 1);
			lcd_puts("User input: *");
		}
		else if (2 == keypad_input_index)
		{
			lcd_puts("Alarm disarmed");
			lcd_gotoxy(0, 1);
			lcd_puts("User input: **");
		}
		else if (3 == keypad_input_index)
		{
			lcd_puts("Alarm disarmed");
			lcd_gotoxy(0, 1);
			lcd_puts("User input: ***");
		}
		else if (4 == keypad_input_index)
		{
			lcd_puts("Alarm disarmed");
			lcd_gotoxy(0, 1);
			lcd_puts("User input: ****");
		}
		else if (0 == keypad_input_index)
		{
			lcd_puts("Alarm disarmed");
			lcd_gotoxy(0, 1);
			lcd_puts("User input:");
		}
		break;
	case MSG_CHANGE_PW:
		if (1 == keypad_input_index)
		{
			lcd_puts("Changing pw");
			lcd_gotoxy(0, 1);
			lcd_puts("New pw: *");
		}
		else if (2 == keypad_input_index)
		{
			lcd_puts("Changing pw");
			lcd_gotoxy(0, 1);
			lcd_puts("New pw: **");
		}
		else if (3 == keypad_input_index)
		{
			lcd_puts("Changing pw");
			lcd_gotoxy(0, 1);
			lcd_puts("New pw: ***");
		}
		else if (4 == keypad_input_index)
		{
			lcd_puts("Changing pw");
			lcd_gotoxy(0, 1);
			lcd_puts("New pw: ****");
		}
		else if (0 == keypad_input_index)
		{
			lcd_puts("Changing pw");
			lcd_gotoxy(0, 1);
			lcd_puts("New pw:");
		}
		break;
	case MSG_ALARM_BUZZING:
		if (wrong_pass == true)
		{
			if (1 == keypad_input_index)
			{
				lcd_puts("Wrong pw");
				lcd_gotoxy(0, 1);
				lcd_puts("User input: *");
			}
			else if (2 == keypad_input_index)
			{
				lcd_puts("Wrong pw");
				lcd_gotoxy(0, 1);
				lcd_puts("User input: **");
			}
			else if (3 == keypad_input_index)
			{
				lcd_puts("Wrong pw");
				lcd_gotoxy(0, 1);
				lcd_puts("User input: ***");
			}
			else if (4 == keypad_input_index)
			{
				lcd_puts("Wrong pw");
				lcd_gotoxy(0, 1);
				lcd_puts("User input: ****");
			}
			else if (0 == keypad_input_index)
			{
				lcd_puts("Wrong pw");
				lcd_gotoxy(0, 1);
				lcd_puts("User input:");
			}
		}
		else
		{
			if (1 == keypad_input_index)
			{
				lcd_puts("Motion detected");
				lcd_gotoxy(0, 1);
				lcd_puts("User input: *");
			}
			else if (2 == keypad_input_index)
			{
				lcd_puts("Motion detected");
				lcd_gotoxy(0, 1);
				lcd_puts("User input: **");
			}
			else if (3 == keypad_input_index)
			{
				lcd_puts("Motion detected");
				lcd_gotoxy(0, 1);
				lcd_puts("User input: ***");
			}
			else if (4 == keypad_input_index)
			{
				lcd_puts("Motion detected");
				lcd_gotoxy(0, 1);
				lcd_puts("User input: ****");
			}
			else if (0 == keypad_input_index)
			{
				lcd_puts("Motion detected");
				lcd_gotoxy(0, 1);
				lcd_puts("User input:");
			}
		}

		break;
	default:
		lcd_puts("Unknown msg");
	}
}

/* initialization of SPI communication by setting ports */
void spi_init()
{
	/* set SS, MOSI and SCK as output, pins 53 (PB0), 51 (PB2) and 52 (PB1) */
	DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2); // SS as output
	/* set SPI enable and master/slave select, making MEGA the master */
	SPCR |= (1 << 6) | (1 << 4);
	/* set SPI clock rate to 1 MHz */
	SPCR |= (1 << 0);
}

/* SPI communication master to slave */
int spi_communicate()
{
	int spi_send_data[1];
	int spi_receive_data[1];
	spi_send_data[0] = 1;
	/* send byte to slave and receive a byte from slave */
	PORTB &= ~(1 << PB0); // SS LOW
	for (int8_t spi_data_index = 0;
		 spi_data_index < sizeof(spi_send_data);
		 spi_data_index++)
	{
		// send byte using SPI data register
		SPDR = spi_send_data[spi_data_index];

		while (!(SPSR & (1 << SPIF)))
		{
			/* wait until the transmission is complete */
			;
		}
		// receive byte from the SPI data register
		spi_receive_data[spi_data_index] = SPDR;
	}
	/* End communication */
	PORTB |= (1 << PB0); // SS HIGH

	return spi_receive_data[0];
}

void led_init()
{
	//Set pin 37 as output
	DDRC |= (1 << PC0);
}

void led_on()
{
	//Set pin 37 High
	PORTC |= (1 << PC0);
}

void led_off()
{
	//Set pin 37 Low
	PORTC &= ~(1 << PC0);
}

bool compare_password()
{
	if (0 == strcmp(password, keypad_input))
	{
		display_message_vol2(MSG_PW_CORRECT);
		_delay_ms(1000);
		return true;
	}
	else
	{
		display_message_vol2(MSG_PW_INCORRECT);
		_delay_ms(1000);
		return false;
	}
}

void init_pwm_timer()
{
	/* Based on exercise 7 solution. 
	   Setting digital pin 6 as PWM output. OC4A output is on the pin 6.*/
	DDRH |= (1 << PH3);
	/* set up the 16-bit timer/counter4, mode 9 */
	TCCR4B = 0; // reset timer/counter 4
	TCNT4 = 0;
	// set compare output mode to toggle.
	// This will output the PWM signal on pin 6 (PH3)
	TCCR4A |= (1 << 6);

	// mode 9 phase correct
	TCCR4A |= (1 << 0); // set register A WGM[1:0] bits
	TCCR4B |= (1 << 4); // set register B WBM[3:2] bits

	TIMSK4 |= (1 << 1); // enable compare match A interrupt
	OCR4A = 8000;		// Should be about 1000 hz.
}

void turn_off_pwm_timer()
{
	/* This should turn off timer counter. */
	TCCR4B &= 0b00000000;
	pwm_timer_on = false;
}

void turn_on_pwm_timer()
{
	/* Enabling timer 4 with no prescaling. */
	if (pwm_timer_on)
	{
		return;
	}
	TCNT4 = 0;
	TCCR4B |= (1 << 4); // set register B WBM[3:2] bits
	TCCR4B |= (1 << 0);
	pwm_timer_on = true;
}

/* timer/counter4 compare match A interrupt vector.
Used for PWM for the buzzer. Needs to be reset manually. */
ISR(TIMER4_COMPA_vect)
{
	TCNT4 = 0;
}