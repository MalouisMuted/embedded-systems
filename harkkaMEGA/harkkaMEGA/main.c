/*
 * harkkaMEGA.c
 *
 * Created: 23.4.2020 14.59.55
 * Author : elias
 */ 

#define F_CPU 16000000UL
#define FOSC 16000000UL // Clock Speed
/*****/
/*for usb serial*/
#define BAUD 9600
#define MYUBRR (FOSC/16/BAUD-1)
/*****/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/setbaud.h>
#include <stdio.h>

/*****/
/*for usb serial*/
static void
USART_init(uint16_t ubrr)
{
	/* Set baud rate in the USART Baud Rate Registers (UBRR) */
	UBRR0H = (unsigned char) (ubrr >> 8);
	UBRR0L = (unsigned char) ubrr;
	
	/* Enable receiver and transmitter on RX0 and TX0 */
	UCSR0B |= (1 << RXEN0) | (1 << TXEN0);
	
	/* Set frame format: 8 bit data, 2 stop bit */
	UCSR0C |= (1 << USBS0) | (3 << UCSZ00);
	
}

static void
USART_Transmit(unsigned char data, FILE *stream)
{
	/* Wait until the transmit buffer is empty*/
	while(!(UCSR0A & (1 << UDRE0)))
	{
		;
	}
	
	/* Puts the data into a buffer, then sends/transmits the data */
	UDR0 = data;
}

static char
USART_Receive(FILE *stream)
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

/*****/

void SPI_MasterInit(void)
{
	/* Set pins SS(pin 50), MOSI(pin 51), SCK(pin 52) for output */ 
	DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2);
	/* Set SPI enabled active and master/slave select to master */
	SPCR |= (1 << 6) | (1 << 4);
	/* Set SPI clock rate to 1 MHz */
	SPCR |= (1 << 0);
}

char SPI_MasterTransmit(char send[20])
{
	char recive[20];
	/* Start SPI communication by pulling SS to LOW */
	PORTB &= ~(1 << PB0);
	/* Start sending and receiving bytes until */
	for(int8_t spi_data_index = 0; spi_data_index < sizeof(send); spi_data_index++)
	{
		SPDR = send[spi_data_index]; /* Send byte */
		while(!(SPSR & (1 << SPIF)))
		{
			; /* Wait to recive byte */
		}
		
		recive[spi_data_index] = SPDR; /* Record received byte */
	}
	/* End SPI communication by pulling SS to HIGH */
	PORTB |= (1<<PB0);
	/* Return received data */ 
	return recive;
}

int main(void)
{
    char SPI_send_data[20] = "Status";
	char SPI_recive_data[20] = "No connection";
	/**/
	/*for usb serial*/
	USART_init(MYUBRR);
	stdout = &uart_output;
	stdin = &uart_input;
	/**/
	SPI_MasterInit();
	printf(SPI_recive_data);
    while (1) 
    {
		SPI_recive_data[20] = SPI_MasterTransmit(SPI_send_data);
		/* Wait to before next check */
		printf(SPI_recive_data);
		_delay_ms(2000);
		
    }
	return 0;
}

