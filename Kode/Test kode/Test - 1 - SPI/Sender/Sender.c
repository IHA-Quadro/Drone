// includes
#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>


// variables

void SPI_MasterInit(void)         //Initialize Atmega8 as master
{
	DDRB = (1<<DDB5)|(1<<DDB3)|(1<<DDB2);   // Set MOSI,SCK and SS\ output, all others input
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);   //  Enable SPI, Master, set clock rate fck/16
}

//Toggle CSn
void CSn(int i)
{
	if(i==1)
	PORTB|=0x04;
	else
	PORTB&=0xFB;
}


unsigned char SPI_transmit(unsigned char data)
{
	// Start transmission
	SPDR = data;

	// Wait for transmission complete
	while(!(SPSR & (1<<SPIF)));
	data = SPDR;

	return(data);
}

int main()
{
    DDRC = 0xFF;
	SPI_MasterInit();         //initialize SPI
	
	CSn(0);
	//This register will be forced to 0x88 or 0x81 when it wakes up from SLEEP mode
	SPI_transmit(0xAE); //Read test 2 register
	PORTC = ~SPI_transmit(0x3B); //strobe - flush TX
	CSn(1);

	while (1)
	{

	}
}

