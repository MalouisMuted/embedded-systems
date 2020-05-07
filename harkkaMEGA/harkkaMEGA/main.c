/*
 * harkkaMEGA.c
 *
 * Created: 23.4.2020 14.59.55
 * Author : Elias, Aleksi, Aku
 */ 

/* State machine states. */
#define CHECK_SENSOR 0
#define ALARM 1


#define F_CPU 16000000UL
#define MaxEepromSize 4096

//UART for debugging
#define FOSC 16000000UL // Clock Speed
#define BAUD 9600
#define MYUBRR (FOSC/16/BAUD-1)
//--------------

#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include "lcd.h" // LCD Library by Peter Fleury.
#include "keypad.h" // Keypad Library by https://www.exploreembedded.com/wiki/AVR_C_Library


//UART for debugging 
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/setbaud.h>
#include <stdio.h>
#include <stdbool.h>

static void USART_init(uint16_t ubrr)
{
	/* Set baud rate in the USART Baud Rate Registers (UBRR) */
	UBRR0H = (unsigned char) (ubrr >> 8);
	UBRR0L = (unsigned char) ubrr;
	
	/* Enable receiver and transmitter on RX0 and TX0 */
	UCSR0B |= (1 << RXEN0) | (1 << TXEN0);
	
	/* Set frame format: 8 bit data, 2 stop bit */
	UCSR0C |= (1 << USBS0) | (3 << UCSZ00);
	
}

static void USART_Transmit(unsigned char data, FILE *stream)
{
	/* Wait until the transmit buffer is empty*/
	while(!(UCSR0A & (1 << UDRE0)))
	{
		;
	}
	
	/* Puts the data into a buffer, then sends/transmits the data */
	UDR0 = data;
}

static char USART_Receive(FILE *stream)
{
	/* Wait until the transmit buffer is empty*/
	while(!(UCSR0A & (1 << UDRE0)))
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

/* LCD messages. */
const char message1_connection_established[20] = "SPI OK";
const char message2_movement_detected[20] = "Movement found";
const char message3_give_password[20] = "Type password";
const char message4_1_password_correct[20] = "Password given,";
const char message4_2_password_correct[20] = "alarm reset";
const char message5_1_password_time_out[20] = "Password time-";
const char message5_2_password_time_out[20] = "out error";

int8_t g_state = 0;
bool pwm_timer_on = false;
int counter = 0;
uint16_t memory_address_max = 32; //for EEPROM, NOTE: not actual max, just there is no need for more

/* Function declarations */
void display_message(int message_number, int password_length); 
void SPI_init();
char *SPI_communicate();
void EEPROM_write(char write_data[32]);
char *EEPROM_read();
/* Counter for PWM buzzer. */
void init_pwm_timer();
void turn_off_pwm_timer();
void turn_on_pwm_timer(); 


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
	USART_init(MYUBRR);
	stdout = &uart_output;
	stdin = &uart_input;
	//----------------
	
    while (1) 
    {
		switch (g_state) 
		{
			/* Keypad should be read here? */
						/*Todo*/
			/*
			// this works to get the key press recorded 
			char input_key[2];
			input_key[0] = KEYPAD_GetKey();
			*/
			case CHECK_SENSOR:
				display_message(0, 0);
				//Checks if uno has detected movement. There are two responses 1."detected motion\n" 2."no motion\n"
				char *recived = SPI_communicate();
				turn_on_pwm_timer(); // On just for debug.
				free(recived);
				/*Todo*/
				break;
			case ALARM:

				break;
		}
		_delay_ms(500);
		display_message(0, 0);
    }
}


/* Displays the string matching the message number. Also displays password as '*' depending on password_length. */
void display_message(int message_number, int password_length) 
{
	/* Clearing the LCD before displaying new message. */
	lcd_clrscr();
	switch (message_number)
	{
		case 0:
			lcd_puts("test");
			break;
		case 1:
			lcd_puts(message1_connection_established);
			break;
		case 2:
			lcd_puts(message2_movement_detected);
			break;
		case 3:
			lcd_puts(message3_give_password);
			lcd_gotoxy(0,1);
			switch (password_length)
			{
				case 0:
				lcd_puts("Password: ");
				break;
				case 1:
				lcd_puts("Password: *");
				break;
				case 2:
				lcd_puts("Password: **");
				break;
				case 3:
				lcd_puts("Password: ***");
				break;
				case 4:
				lcd_puts("Password: ****");
				break;
				default:
				lcd_puts("Password: Error");
			}
			break;
		case 4:
			lcd_puts(message4_1_password_correct);
			lcd_gotoxy(0,1);
			lcd_puts(message4_2_password_correct);
			break;
		case 5:
			lcd_puts(message5_1_password_time_out);
			lcd_gotoxy(0,1);
			lcd_puts(message5_2_password_time_out);
			break;
		default:
			lcd_puts("Unknown value");
			lcd_gotoxy(0,1);
			lcd_puts("to LCD function");
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
char *SPI_communicate()
{
	char spi_send_data[20] = "master to slave\n";
	char* spi_receive_data = malloc(20);
	/* send byte to slave and receive a byte from slave */
	PORTB &= ~(1 << PB0); // SS LOW
	for(int8_t spi_data_index = 0; spi_data_index < sizeof(spi_send_data); spi_data_index++)
	{
		
		SPDR = spi_send_data[spi_data_index]; // send byte using SPI data register
		
		while(!(SPSR & (1 << SPIF)))
		{
			/* wait until the transmission is complete */
			;
		}
		
		spi_receive_data[spi_data_index] = SPDR; // receive byte from the SPI data register
	}
	/* End communication */
	PORTB |= (1 << PB0); // SS HIGH
	
	//Debugging sending received message to UART
	printf(spi_receive_data);
	
	return spi_receive_data;
}

void EEPROM_write(char write_data[32])
{
	for (uint16_t address_index = 0; address_index < sizeof(write_data); address_index++)
	{
		while(EECR & (1 << 1))
		{
			/* wait for the previous write operation to end */
		}
		
		EEAR = address_index;
		EEDR = write_data[address_index];
		EECR |= (1 << 2); // master programming enable
		EECR |= (1 << 1); // EEPROM programming enable
	}
}

char *EEPROM_read()
{
	char* memory_data = malloc(32);
	for (uint16_t address_index = 0; address_index < memory_address_max; address_index++)
	{
		while(EECR & (1 << 1))
		{
			/* wait for the previous write operation to end */
		}
		
		EEAR  = address_index;
		EECR |= 0x01; // enable EEPROM read
		memory_data[address_index] = EEDR;
	}
	return memory_data;
}


void init_pwm_timer() {
	/* Based on exercise 7 solution. 
	   Setting digital pin 6 as PWM output. OC4A output is on the pin 6.*/
	DDRH |= (1 << PH3); 
	/* set up the 16-bit timer/counter4, mode 9 */
	TCCR4B = 0; // reset timer/counter 4
	TCNT4  = 0;
	
	TCCR4A |= (1 << 6); // set compare output mode to toggle. This will output the PWM signal on pin 6 (PH3)
	
	// mode 9 phase correct
	TCCR4A |= (1 << 0); // set register A WGM[1:0] bits
	TCCR4B |= (1 << 4); // set register B WBM[3:2] bits

	TIMSK4 |= (1 << 1); // enable compare match A interrupt
	OCR4A = 8000; // Should be about 1000 hz.
}

void turn_off_pwm_timer() {
	/* This should turn off timer counter. */
	TCCR4B &= 0b00000000;
	pwm_timer_on = false; 
}

void turn_on_pwm_timer() {
	/* Enabling timer 4 with no prescaling. */
	if (pwm_timer_on)
	{
		return;
	}
	TCNT4  = 0;
	TCCR4B |= (1 << 4); // set register B WBM[3:2] bits
	TCCR4B |= (1 << 0);
	pwm_timer_on = true; 
}


/* timer/counter4 compare match A interrupt vector. Used for PWM for the buzzer. Needs to be reset manually. */
ISR
(TIMER4_COMPA_vect)
{
	TCNT4 = 0; 
}