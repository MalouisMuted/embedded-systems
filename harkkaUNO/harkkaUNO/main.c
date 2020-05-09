/*
 * harkkaUNO.c
 *
 * Created: 23.4.2020 14.58.40
 * Author : Elias, Aleksi, Aku
 */ 

#include <avr/io.h>
#include <string.h>

int main(void)
{
	/* set MISO as output, pin 12 (PB4)*/
	DDRB  = (1 << PB4);
	/* set SPI enable */
	SPCR  = (1 << 6);
	/* set motion sensor pin as input */
	DDRD &= ~(1 << PD7);
	
	char spi_send_data[20] = "slave to master\n";
	char spi_receive_data[20];
	int8_t motion_sensor_status = 0;
	
	/* send message to master and receive message from master */
	while (1)
	{
		/* read motion sensor */
		motion_sensor_status = (PIND & (1<<PD7));
		if (0 != motion_sensor_status) {
			//sensor high
			strcpy(spi_send_data,"detected motion\n");
			} else {
			// sensor low
			strcpy(spi_send_data,"no motion\n");
		}
		/* transmit data */
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
	}
	return 0;
}


//#define F_CPU 16000000UL
//#define FOSC 16000000UL // Clock Speed
//#define BAUD 9600
//#define MYUBRR (FOSC/16/BAUD-1)
//
//#include <avr/io.h>
//#include <util/delay.h>
//#include <util/setbaud.h>
//#include <stdio.h>
//
//static void
//USART_init(uint16_t ubrr) // unsigned int
//{
	///* Set baud rate in the USART Baud Rate Registers (UBRR) */
	//UBRR0H = (unsigned char) (ubrr >> 8);
	//UBRR0L = (unsigned char) ubrr;
	//
	///* Enable receiver and transmitter on RX0 and TX0 */
	//UCSR0B |= (1 << RXEN0) | (1 << TXEN0); //NOTE: the ATmega328p has 1 UART: 0
	//// UCSR0B |= (1 << 4) | (1 << 3);
	//
	///* Set frame format: 8 bit data, 2 stop bit */
	//UCSR0C |= (1 << USBS0) | (3 << UCSZ00);
	//// UCSR0C |= (1 << 3) | (3 << 1);
	//
//}
//
//static void
//USART_Transmit(unsigned char data, FILE *stream)
//{
	///* Wait until the transmit buffer is empty*/
	//while(!(UCSR0A & (1 << UDRE0)))
	//{
		//;
	//}
	//
	///* Put the data into a buffer, then send/transmit the data */
	//UDR0 = data;
//}
//
//static char
//USART_Receive(FILE *stream)
//{
	///* Wait until the transmit buffer is empty*/
	//while(!(UCSR0A & (1 << UDRE0)))
	//{
		//;
	//}
	//
	///* Get the received data from the buffer */
	//return UDR0;
//}
//
//// Setup the stream functions for UART
//FILE uart_output = FDEV_SETUP_STREAM(USART_Transmit, NULL, _FDEV_SETUP_WRITE);
//FILE uart_input = FDEV_SETUP_STREAM(NULL, USART_Receive, _FDEV_SETUP_READ);
//
//int
//main(void)
//{
	//// initialize the UART with 9600 BAUD
	//USART_init(MYUBRR);
	//
	//// redirect the stdin and stdout to UART functions
	//stdout = &uart_output;
	//stdin = &uart_input;
	//
	///* set motion sensor pin as input */
	//DDRD &= ~(1 << PD7);
	//
	//char spi_send_data[20] = "no motion\n";
	//char spi_receive_data[20];
	//int8_t motion_sensor_status = 0;
	//
	//while (1)
	//{
		//// print out using the UART. This can be accessed via terminals such as PuTTY.
		///* read motion sensor */
		//motion_sensor_status = (PIND & (1<<PD7));
		//if (0 != motion_sensor_status) {
			////sensor high
			//strcpy(spi_send_data,"detected motion\n");
			//} else {
			//// sensor low
			//strcpy(spi_send_data,"no motion\n");
		//}
		//printf(spi_send_data);
		//_delay_ms(200);
	//}
	//
	//return 0;
//}