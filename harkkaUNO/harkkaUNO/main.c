/*Aleksi, Elias, Aku*/

#define F_CPU 16000000UL
#define FOSC 16000000UL // Clock Speed
#define BAUD 9600
#define MYUBRR (FOSC/16/BAUD-1)

#include <avr/io.h>
#include <util/delay.h>
#include <util/setbaud.h>
#include <stdio.h>
#include <string.h>

static void
USART_init(uint16_t ubrr) // unsigned int
{
	/* Set baud rate in the USART Baud Rate Registers (UBRR) */
	UBRR0H = (unsigned char) (ubrr >> 8);
	UBRR0L = (unsigned char) ubrr;
	
	/* Enable receiver and transmitter on RX0 and TX0 */
	UCSR0B |= (1 << RXEN0) | (1 << TXEN0); //NOTE: the ATmega328p has 1 UART: 0
	
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
	
	/* Put the data into a buffer, then send/transmit the data */
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



void SPI_SlaveInit(void)
{
    /* Set MISO output, all others input */
    DDRB  = (1 << PB4);
    /* Enable SPI */
    SPCR  = (1 << 6);
}
char SPI_SlaveReceive(char spi_send_data[20])
{
	char spi_receive_data[20];
	/* alkaa menemään lähetettävää viestiä läpi bittibitiltä */
	for(int8_t spi_data_index = 0; spi_data_index < sizeof(spi_send_data); spi_data_index++){
		/* lataa send bufferiin lähetettävän bitin*/
		SPDR = spi_send_data[spi_data_index];
		while (!(SPSR & (1 << SPIF))){
			; /* odottaa että saa luvan lähettää ja samalla vastaanottaa bitin */
		}
		spi_receive_data[spi_data_index] = SPDR; /* tallettaa saatua dataa bitti bitiltä*/
	}
    /* Return Data Register */
    return spi_receive_data;
}
void MotionSensorInit(void)
{
	/* Set pin7(PD7) as input for motion sensor */
	DDRD &= ~(1 << PD7);
	
}
char MotionSensorCheck(void)
{
	int8_t motion_value = 0;
	char status[20];
	/* checking if there has been motion */
	motion_value = (PIND & (1<<PD7));
	/* returning dependent on value */
	if (0 != motion_value) {
		strcpy(status,"motion");
	} else {
		strcpy(status,"still");
	}
	return status;
}

// Setup the stream functions for UART
FILE uart_output = FDEV_SETUP_STREAM(USART_Transmit, NULL, _FDEV_SETUP_WRITE);
FILE uart_input = FDEV_SETUP_STREAM(NULL, USART_Receive, _FDEV_SETUP_READ);

int main(void)
{
    // enable global interrupts
	char send[20];
	char recive[20];
	//usb serial
	USART_init(MYUBRR);
	stdout = &uart_output;
	stdin = &uart_input;
	SPI_SlaveInit();
	MotionSensorInit();
	// Loop
    while (1)
    {
		/* Check sensor and then transmit to SPI master */
		send[20] = MotionSensorCheck();
		recive[20] = SPI_SlaveReceive(send);
		printf(recive);
    }
	return 0;
}
