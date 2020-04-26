/*
 * mega_SPI_master_basics.c
 *
 * Created: 4.7.2019 8:52:14
 * Author : Glutex
 */ 

/* ATmega2560 (MEGA2560) is master */

#define F_CPU 16000000UL
#define FOSC 16000000UL // Clock Speed
#define BAUD 9600
#define MYUBRR (FOSC/16/BAUD-1)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/setbaud.h>
#include <stdio.h>

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

int 
main(void)
{
    /* set SS, MOSI and SCK as output, pins 53 (PB0), 51 (PB2) and 52 (PB1) */
    DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2); // SS as output
    /* set SPI enable and master/slave select, making MEGA the master */
    SPCR |= (1 << 6) | (1 << 4);
    /* set SPI clock rate to 1 MHz */
    SPCR |= (1 << 0);
    
    // initialize the UART with 9600 BAUD
    USART_init(MYUBRR);
        
    // redirect the stdin and stdout to UART functions
    stdout = &uart_output;
    stdin = &uart_input;
    
    unsigned char spi_send_data[20] = "master to slave\n";
    unsigned char spi_receive_data[20];
    
    /* send message to slave and receive message from slave */
    while (1) 
    {
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
            
        PORTB |= (1 << PB0); // SS HIGH
        
        printf(spi_receive_data);
        _delay_ms(2000);

    }
    
    return 0;
}