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