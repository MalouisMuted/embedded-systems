/*Aleksi, Elias, Aku*/

#include <avr/io.h>
//#include <avr/interrupt.h>
#define F_CPU 16000000UL

void SPI_SlaveInit(void)
{
    /* Set MISO output, all others input */
    DDR_SPI = (1 << DD_MISO);
    /* Enable SPI */
    SPCR = (1 << SPE);
}
char SPI_SlaveReceive(void)
{
    /* Wait for reception complete */
    while (!(SPSR & (1 << SPIF)))
        ;
    /* Return Data Register */
    return SPDR;
}

int main(void)
{
    // enable global interrupts
    // sei();
    while (1)
    {
    }
}
