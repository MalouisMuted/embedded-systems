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
		status = "motion";
	} else {
		status = "still";
	}
	return status;
}

int main(void)
{
    // enable global interrupts
	char send[20];
	char recive[20];
	SPI_SlaveInit();
	MotionSensorInit();
	// Loop
    while (1)
    {
		/* Check sensor and then transmit to SPI master */
		send = MotionSensorCheck();
		recive = SPI_SlaveReceive(send);
    }
	return 0;
}
