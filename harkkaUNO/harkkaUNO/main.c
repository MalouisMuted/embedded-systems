/*
 * harkkaUNO.c
 *
 * Created: 23.4.2020 14.58.40
 * Author : Elias, Aleksi, Aku
 */
// Defined for SPI communications
#define MOTION_DETECTED 1
#define NO_MOTION 0

#include <avr/io.h>
#include <string.h>

int main(void)
{
	/* set MISO as output, pin 12 (PB4)*/
	DDRB = (1 << PB4);
	/* set SPI enable */
	SPCR = (1 << 6);
	/* set motion sensor pin as input */
	DDRD &= ~(1 << PD7);

	int spi_send_data[1];
	int spi_receive_data[1];
	int8_t motion_sensor_status = 0;

	/* send message to master and receive message from master */
	while (1)
	{
		/* read motion sensor */
		motion_sensor_status = (PIND & (1 << PD7));
		if (0 != motion_sensor_status)
		{
			//sensor high
			spi_send_data[0] = MOTION_DETECTED;
		}
		else
		{
			// sensor low
			spi_send_data[0] = NO_MOTION;
		}
		/* transmit data */
		for (int8_t spi_data_index = 0;
			 spi_data_index < sizeof(spi_send_data);
			 spi_data_index++)
		{
			// send byte using SPI data register
			SPDR = spi_send_data[spi_data_index];

			while (!(SPSR & (1 << SPIF)))
			{
				/* wait until the transmission is complete */
			}
			// receive byte from the SPI data register
			spi_receive_data[spi_data_index] = SPDR;
		}
	}
	return 0;
}