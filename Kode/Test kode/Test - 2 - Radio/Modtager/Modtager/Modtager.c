#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

#define F_CPU 3686400
#define CE_PIN 4
char data[TWI_BUFFER_SIZE];
unsigned char messageBuf[TWI_BUFFER_SIZE];
char i;

void SPI_MasterInit(void)         //Initialize Atmega8 as master
{
	
	DDRB = (1<<DDB5)|(1<<DDB3)|(1<<DDB2);   // Set MOSI,SCK and SS\ output, all others input
	
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);   //  Enable SPI, Master, set clock rate fck/16
}

void CSn(int i)               //Toggle CSn
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

void Full_RegConfig(void)         //for full register configuration using BURST access
{
	CSn(0);
	while(bit_is_set(PINB,4)){}
SPI_transmit(0x3E);      //address for patable access
SPI_transmit(0x60);      // --ss1--//

//selve opsetningen

SPI_transmit(0x40);      //-- Starter fra 00 med burst--//

//GDO opsætning
SPI_transmit(0x02);      // register 0x00
SPI_transmit(0x2E);      // register 0x01
SPI_transmit(0x01);      // register 0x02
SPI_transmit(0x40);      // register 0x03

SPI_transmit(0xD3);      // register 0x04
SPI_transmit(0x91);      // register 0x05
SPI_transmit(0xFF);      // register 0x06
SPI_transmit(0x0C);      // register 0x07
SPI_transmit(0x0D);      // register 0x08
SPI_transmit(0x00);      // register 0x09
SPI_transmit(0x00);      // register 0x0A
SPI_transmit(0x06);      // register 0x0B
SPI_transmit(0x00);      // register 0x0C
SPI_transmit(0x10);      // register 0x0D
SPI_transmit(0xB1);      // register 0x0E
SPI_transmit(0x3B);      // register 0x0F
SPI_transmit(0xF6);      // register 0x10 
SPI_transmit(0x83);      // register 0x11 
SPI_transmit(0x13);      // register 0x12
SPI_transmit(0x22);      // register 0x13
SPI_transmit(0xF8);      // register 0x14
SPI_transmit(0x15);      // register 0x15
SPI_transmit(0x07);      // register 0x16
SPI_transmit(0x30);      // register 0x17
SPI_transmit(0x18);      // register 0x18

SPI_transmit(0x16);      // register 0x19
SPI_transmit(0x6C);      // register 0x1A
SPI_transmit(0x03);      // register 0x1B
SPI_transmit(0x40);      // register 0x1C
SPI_transmit(0x91);      // register 0x1D
SPI_transmit(0x87);      // register 0x1E
SPI_transmit(0x6B);      // register 0x1F
SPI_transmit(0xFB);      // register 0x20
SPI_transmit(0x56);      // register 0x21
SPI_transmit(0x10);      // register 0x22

SPI_transmit(0xE9);      // register 0x23
SPI_transmit(0x2A);      // register 0x24
SPI_transmit(0x00);      // register 0x25
SPI_transmit(0x1F);      // register 0x26

SPI_transmit(0x41);      // register 0x27
SPI_transmit(0x00);      // register 0x28
SPI_transmit(0x59);      // register 0x29
SPI_transmit(0x7F);      // register 0x2A
SPI_transmit(0x3F);      // register 0x2B
SPI_transmit(0x81);      // register 0x2C
SPI_transmit(0x35);      // register 0x2D
SPI_transmit(0x09);      // register 0x2E

CSn(1);

}

void initExtInts()
{
	// INT0:Rising edge
	MCUCR = 0b00000011;
	// Enable external interrupts INT0
	GICR |= 0b01000000;
}
// Interrupt service routine for INT0
ISR (INT0_vect)
{
	char info;	
	CSn(0);
	SPI_transmit(0xFB); //Number of bytes
	info = SPI_transmit(0x3D);//rand
	CSn(1);
	
	//read the data
	CSn(0);
	SPI_transmit(0xFF); //strobe - Exit all
	for (i=0;i<(info-1);i++)
	{
		data[i] = (SPI_transmit(0x00)>>1);
	}
	CSn(1);
	PORTC = ~data[0];
	
	CSn(0);
	SPI_transmit(0x36); //strobe - Exit all
	SPI_transmit(0x3A); //strobe - flush RX
	SPI_transmit(0x34); //enable Rx
	CSn(1);
}

int main()
{
	DDRC = 0b11111111;
	PORTC =0b00000000;

	// Initialize  INT0
	initExtInts();
	SPI_MasterInit();         //initialize SPI
				
	CSn(0);
	SPI_transmit(0x30);
	CSn(1);
	
	Full_RegConfig();
	
	CSn(0);
	SPI_transmit(0x36); //strobe - Exit all
	SPI_transmit(0x3A); //strobe - flush RX
	CSn(1);
		
	CSn(0);
	SPI_transmit(0x34); //enable Rx
	CSn(1);
	
	//Global interrupt enable
	sei();

	TWI_Start_Transceiver();

  while (1)
  {
  }
}