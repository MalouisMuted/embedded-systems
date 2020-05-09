/*
 * harkkaMEGA.c
 *
 * Created: 23.4.2020 14.59.55
 * Author : Elias, Aleksi, Aku
 */

// State machine macros
#define ALARM_BUZZING 0
#define ARMED 1
#define DISAMERD 2
#define CHANGE_PW 3

// Defined for SPI communications
#define MOTION_DETECTED 1
#define NO_MOTION 0

// Display message macros
#define MSG_ALARM_BUZZING 0
#define MSG_ARMED 1
#define MSG_DISAMERD 2
#define MSG_CHANGE_PW 3

// Timeout macros
#define INPUT_TIMEOUT_STEPS 100
#define RESET_LOGGED_IN_STEPS 400

#define F_CPU 16000000UL
#define MaxEepromSize 4096

//UART for debugging
#define FOSC 16000000UL // Clock Speed
#define BAUD 9600
#define MYUBRR (FOSC / 16 / BAUD - 1)
//--------------

#include <avr/io.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <util/delay.h>
#include "lcd.h"	// LCD Library by Peter Fleury.
#include "keypad.h" // Keypad Library by https://www.exploreembedded.com/wiki/AVR_C_Library

//UART for debugging
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/setbaud.h>
#include <stdio.h>

static void USART_init(uint16_t ubrr)
{
	/* Set baud rate in the USART Baud Rate Registers (UBRR) */
	UBRR0H = (unsigned char)(ubrr >> 8);
	UBRR0L = (unsigned char)ubrr;

	/* Enable receiver and transmitter on RX0 and TX0 */
	UCSR0B |= (1 << RXEN0) | (1 << TXEN0);

	/* Set frame format: 8 bit data, 2 stop bit */
	UCSR0C |= (1 << USBS0) | (3 << UCSZ00);
}

static void USART_Transmit(unsigned char data, FILE *stream)
{
	/* Wait until the transmit buffer is empty*/
	while (!(UCSR0A & (1 << UDRE0)))
	{
		;
	}

	/* Puts the data into a buffer, then sends/transmits the data */
	UDR0 = data;
}

static char USART_Receive(FILE *stream)
{
	/* Wait until the transmit buffer is empty*/
	while (!(UCSR0A & (1 << UDRE0)))
	{
		;
	}

	/* Get the received data from the buffer */
	return UDR0;
}
// Setup the stream functions for UART
FILE uart_output = FDEV_SETUP_STREAM(USART_Transmit, NULL, _FDEV_SETUP_WRITE);
FILE uart_input = FDEV_SETUP_STREAM(NULL, USART_Receive, _FDEV_SETUP_READ);
// ----------------------------------

int8_t g_state;
bool pwm_timer_on = false;
bool time_out_timer_on = false;
uint16_t input_timeout_step = 0;
uint16_t reset_logged_in_step = 0;
char keypad_input[5];
char key_input[2];
int8_t keypad_input_index = 0;
int revived = NO_MOTION;

int counter = 0;
uint16_t memory_address_max = 32; //for EEPROM, NOTE: not actual max, just there is no need for more

/* Function declarations */
void display_message_vol2(int msg_number);
void SPI_init();
int SPI_communicate();
void EEPROM_write(char write_data[32]);
void EEPROM_read(char *memory_data);
/* Counter for PWM buzzer. */
void init_pwm_timer();
void turn_off_pwm_timer();
void turn_on_pwm_timer();

void compare_password();
void reset_input();
void log_in();
void change_pw();
void take_user_input();

int main(void)
{
	// Enabling global interrupts.
	sei();
	// LCD, SPI and Keypad initialization.
	lcd_init(LCD_DISP_ON);
	KEYPAD_Init();
	SPI_init();

	/* Init timer for PWM. */
	init_pwm_timer();

	// INIT of UART
	memset(keypad_input, " ", 4);
	EEPROM_write("1234");
	USART_init(MYUBRR);
	g_state = ARMED;
	stdout = &uart_output;
	stdin = &uart_input;
	//----------------

	while (1)
	{
		input_timeout_step++;
		reset_logged_in_step++;
		
		key_input[0] = KEYPAD_GetKey();
		
				
		if (!(0 == strcmp((char*)&key_input[0],"z")))
		{
			printf((char*)&key_input[0]);
			printf(" nyt painettu\n");
			
			take_user_input();
		}

		if (input_timeout_step > INPUT_TIMEOUT_STEPS)
		{
			reset_input();
		}

		switch (g_state)
		{
		case ALARM_BUZZING:
			display_message_vol2(MSG_ALARM_BUZZING);
			break;
		case ARMED:
			display_message_vol2(MSG_ARMED);
			//Checks if uno has detected movement. There are two responses 1."detected motion\n" 2."no motion\n"
			revived = SPI_communicate();
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
		_delay_ms(50);
	}
}

void reset_input()
{
	keypad_input_index = 0;
	input_timeout_step = 0;
	strcpy(keypad_input, "    ");
}

void change_pw()
{
	EEPROM_write(&keypad_input);
}

void log_in()
{
	reset_logged_in_step = 0;
	reset_input();
	turn_off_pwm_timer();
	g_state = DISAMERD;
}


void take_user_input()
{
	keypad_input[keypad_input_index] = KEYPAD_GetKey();
	KEYPAD_WaitForKeyRelease();
	input_timeout_step = 0;
	keypad_input_index++;

	if (4 == keypad_input_index && g_state == CHANGE_PW)
	{
		// User is logged in and wants to change password
		change_pw();
	}
	else if (4 == keypad_input_index && g_state == ARMED)
	{
		// User is logged out and inputted password
		printf("%s tassa koko salasana\n",keypad_input);
		compare_password();
	}
	else if (g_state == DISAMERD)
	{
		// User logged in, 3 actions
		if (0 == strcmp("B", keypad_input[0]))
		{
			// Action change pw
			g_state == CHANGE_PW;
			reset_input();
		}
	}
}

/* New funktion for displaying messages and taking one argument*/
void display_message_vol2(int msg_number)
{
	lcd_clrscr();
	switch (msg_number)
	{
	case MSG_ARMED:
		if (keypad_input_index == 1) {
			lcd_puts("Alarm armed");
			lcd_gotoxy(0, 1);
			lcd_puts("User input: *");
		} else if (keypad_input_index == 2) {
			lcd_puts("Alarm armed");
			lcd_gotoxy(0, 1);
			lcd_puts("User input: **");
		} else if (keypad_input_index == 3) {
			lcd_puts("Alarm armed");
			lcd_gotoxy(0, 1);
			lcd_puts("User input: ***");
		} else if (keypad_input_index == 4) {
			lcd_puts("Alarm armed");
			lcd_gotoxy(0, 1);
			lcd_puts("User input: ****");
		} else if (keypad_input_index == 0) {
			lcd_puts("Alarm armed");
			lcd_gotoxy(0, 1);
			lcd_puts("User input:");
		}
		break;
	case MSG_DISAMERD:
		lcd_puts("Alarm disarmed");
		break;
	case MSG_CHANGE_PW:
		if (keypad_input_index == 1) {
			lcd_puts("Changing pw");
			lcd_gotoxy(0, 1);
			lcd_puts("New pw: *");
		} else if (keypad_input_index == 2) {
			lcd_puts("Changing pw");
			lcd_gotoxy(0, 1);
			lcd_puts("New pw: **");
		} else if (keypad_input_index == 3) {
			lcd_puts("Changing pw");
			lcd_gotoxy(0, 1);
			lcd_puts("New pw: ***");
		} else if (keypad_input_index == 4) {
			lcd_puts("Changing pw");
			lcd_gotoxy(0, 1);
			lcd_puts("New pw: ****");
		} else if (keypad_input_index == 0) {
			lcd_puts("Changing pw");
			lcd_gotoxy(0, 1);
			lcd_puts("New pw:");
		}
		break;
	case MSG_ALARM_BUZZING:
		if (keypad_input_index == 1) {
			lcd_puts("Motion detected");
			lcd_gotoxy(0, 1);
			lcd_puts("User input: *");
		} else if (keypad_input_index == 2) {
			lcd_puts("Motion detected");
			lcd_gotoxy(0, 1);
			lcd_puts("User input: **");
		} else if (keypad_input_index == 3) {
			lcd_puts("Motion detected");
			lcd_gotoxy(0, 1);
			lcd_puts("User input: ***");
		} else if (keypad_input_index == 4) {
			lcd_puts("Motion detected");
			lcd_gotoxy(0, 1);
			lcd_puts("User input: ****");
		} else if (keypad_input_index == 0) {
			lcd_puts("Motion detected");
			lcd_gotoxy(0, 1);
			lcd_puts("User input:");
		}
		break;
	default:
		lcd_puts("Unknown msg");
	}
}

/* initialization of SPI communication by setting ports */
void SPI_init()
{
	/* set SS, MOSI and SCK as output, pins 53 (PB0), 51 (PB2) and 52 (PB1) */
	DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2); // SS as output
	/* set SPI enable and master/slave select, making MEGA the master */
	SPCR |= (1 << 6) | (1 << 4);
	/* set SPI clock rate to 1 MHz */
	SPCR |= (1 << 0);
}

/* SPI communication master to slave */
int SPI_communicate()
{
	int spi_send_data[1];
	int spi_receive_data[1];
	spi_send_data[0] = 1;
	/* send byte to slave and receive a byte from slave */
	PORTB &= ~(1 << PB0); // SS LOW
	for (int8_t spi_data_index = 0; spi_data_index < sizeof(spi_send_data); spi_data_index++)
	{

		SPDR = spi_send_data[spi_data_index]; // send byte using SPI data register

		while (!(SPSR & (1 << SPIF)))
		{
			/* wait until the transmission is complete */
			;
		}

		spi_receive_data[spi_data_index] = SPDR; // receive byte from the SPI data register
	}
	/* End communication */
	PORTB |= (1 << PB0); // SS HIGH

	//Debugging sending received message to UART
	//printf("%d\n",spi_receive_data[0]);	
	
	return spi_receive_data[0];
}

void EEPROM_write(char write_data[32])
{
	for (uint16_t address_index = 0; address_index < sizeof(write_data); address_index++)
	{
		while (EECR & (1 << 1))
		{
			/* wait for the previous write operation to end */
		}

		EEAR = address_index;
		EEDR = write_data[address_index];
		EECR |= (1 << 2); // master programming enable
		EECR |= (1 << 1); // EEPROM programming enable
	}
}

void EEPROM_read(char *memory_data)
{
	for (uint16_t address_index = 0; address_index < memory_address_max; address_index++)
	{
		while (EECR & (1 << 1))
		{
			/* wait for the previous write operation to end */
		}

		EEAR = address_index;
		EECR |= 0x01; // enable EEPROM read
		memory_data[address_index] = EEDR;
	}
}

void compare_password()
{
	char valid_pw = malloc(32);
	EEPROM_read(valid_pw);
	if (0 == strcmp(valid_pw, keypad_input))
	{
		log_in();
	}
	else
	{
		// Invalid pw
		g_state = ALARM_BUZZING;
		turn_on_pwm_timer();
	}
	free(valid_pw);
}

void init_pwm_timer()
{
	/* Based on exercise 7 solution. 
	   Setting digital pin 6 as PWM output. OC4A output is on the pin 6.*/
	DDRH |= (1 << PH3);
	/* set up the 16-bit timer/counter4, mode 9 */
	TCCR4B = 0; // reset timer/counter 4
	TCNT4 = 0;

	TCCR4A |= (1 << 6); // set compare output mode to toggle. This will output the PWM signal on pin 6 (PH3)

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

/* timer/counter4 compare match A interrupt vector. Used for PWM for the buzzer. Needs to be reset manually. */
ISR(TIMER4_COMPA_vect)
{
	TCNT4 = 0;
}