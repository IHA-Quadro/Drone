// includes
#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

// defines
#define F_CPU 8000000
#define CE_PIN 4
#define MAX_SWITCH_NR 7

// variables
int i=0;
char out =0;
char data[2];
char State = 1;
unsigned char dummy;


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

//Switch return true
unsigned char switchOn(unsigned char switch_nr)
{
	unsigned char mask;
	if (switch_nr <= MAX_SWITCH_NR)
	{
		mask = 0b00000001 << switch_nr;
		return (~PIND & mask);
	}
	else
	return 0;
}
//Switch return true
unsigned char switchOnC(unsigned char switch_nr)
{
	unsigned char mask;
	if (switch_nr <= MAX_SWITCH_NR)
	{
		mask = 0b00000001 << switch_nr;
		return (~PINC & mask);
	}
	else
	return 0;
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

void SendProgram(void)
{
	CSn(0);
	SPI_transmit(0x3B); //strobe - flush TX
	SPI_transmit(0x7F); //tx fifo burst
	//SPI_transmit(0x6); //Data
	SPI_transmit(0x3); //Pakke størrelse
	SPI_transmit(0x7E); //Data
	SPI_transmit(data[0]); //Data
	
	/*SPI_transmit(0x2); //Data
	SPI_transmit(0x3); //Data
	SPI_transmit(0x4); //Data*/
	CSn(1);
	
	CSn(0);
	SPI_transmit(0xFA); //tx bytes
	out = SPI_transmit(0xFF); //rand
	CSn(1);
	CSn(0);
	SPI_transmit(0x35); //enable tx
	CSn(1);
	while (out != 0)
	{
		CSn(0);
		SPI_transmit(0xFA); //tx bytes
		out = SPI_transmit(0xFF); //rand
		CSn(1);
	}

	CSn(0);
	SPI_transmit(0x36); //strobe - Exit all
	SPI_transmit(0x3B); //strobe - flush TX
	CSn(1);
}

void Full_RegConfig(void)         //for full register configuration using BURST access
{
	CSn(0);
	while(bit_is_set(PINB,4)){}
	SPI_transmit(0x3E);      //address for patable access
	SPI_transmit(0x60);      //patable for PA power control settings,10 dBm o/p power,29.1 ma current consumption // --ss1--//

	//selve opsetningen

	SPI_transmit(0x40);      //burst access and write status registers and command probes  //-- Starter fra 00 med burst--//

	//GDO opsætning
	SPI_transmit(0x02);      // register 0x00,GDO2 o/p pin set to default,s7 is CHIP_RDYn  // -- Gdo fis--//
	SPI_transmit(0x2E);      // register 0x01,GDO1 o/p pin 3-state default				// -- Gdo fis--//
	SPI_transmit(0x01);      // register 0x02,asserts when sync word has been sent / received, and de-asserts at the end of the packet.	// -- Gdo fis--//
	SPI_transmit(0x40);      // register 0x03,FIFOTHR:0dB rx attenuation,33 bytes in tx fifo,32 in rx fifo,?TEST1 = 0x35,TEST2 = 0x81 when waking up from SLEEP

	SPI_transmit(0xD3);      // register 0x04,SYNC1(high),8 MSB of 16-bit sync word
	SPI_transmit(0x91);      // register 0x05,SYNC0(low),8 LSB of 16-bit sync word
	SPI_transmit(0xFF);      // register 0x06,Packet length(variable(max)or fixed length):61
	SPI_transmit(0x0C);      // register 0x07 EARLIER 04,Packet Automation Control1:Preamble Quality estimator threshold=0,auto flush rxfifo when crc not ok,no address check of recvd packages
	SPI_transmit(0x0D);      // register 0x08 EARLIER 05,Packet Automation Control0:Whitening off,normal mode using fifos,crc disabled,infinite packet length mode
	SPI_transmit(0x00);      // register 0x09,address used for packet filtration ?
	SPI_transmit(0x00);      // register 0x0A,8-bit unsigned channel number
	SPI_transmit(0x06);      // register 0x0B,Frequency Synthesizer Control1:152 kHz IF frequency in RX assuming 26 MHz crystal
	SPI_transmit(0x00);      // register 0x0C,Frequency Synthesizer Control0:default
	SPI_transmit(0x10);      // register 0x0D,Frequency Control Word :417 MHz base frequency for frequency synthesizer
	SPI_transmit(0xB1);      // register 0x0E EARLIER A7 866,Frequency Control Word
	SPI_transmit(0x3B);      // 0x0F EARLIER 62 866,Frequency Control Word
	SPI_transmit(0xF6);      // register 0x10 CA,Modem configuration:channel bandwidth of 58kHz
	SPI_transmit(0x83);      // register 0x11 EARLIER 83 866,Modem Configuration
	SPI_transmit(0x13);      // register 0x12,Modem Configuration:Enable Digital DC blocking filter,2-FSK modulation,disable Manchester encoding/decoding,30/32 sync word bits qualifier
	SPI_transmit(0x22);      // register 0x13,Disable forward error correction,4 preamble bytes
	SPI_transmit(0xF8);      // register 0x14,Modem configuration,192 kHz channel spacing
	SPI_transmit(0x15);      // register 0x15 EARLIER 15 866,4.76 kHz frequency deviation
	SPI_transmit(0x07);      // register 0x16,Direct RX termination based on RSSI measurement (carrier sense),timeout for sync word search in rx:until end of packet?
	SPI_transmit(0x30);      // register 0x17,CCA_MODE,IDLE after packet transmission and after packet reception
	SPI_transmit(0x18);      // register 0x18,Automatically calibrate when going from IDLE to rx or tx,PO_TIMEOUT=2,timeout after xosc start=150 us,enable radio control,xosc on in sleep state

	SPI_transmit(0x16);      // register 0x19,frequency offset compensation configuration
	SPI_transmit(0x6C);      // register 0x1A,bit synchronization configuration
	SPI_transmit(0x03);      // register 0x1B,AGC control
	SPI_transmit(0x40);      // register 0x1C,AGC control
	SPI_transmit(0x91);      // register 0x1D,AGC control
	SPI_transmit(0x87);      // register 0x1E,High Byte Event0 Timeout
	SPI_transmit(0x6B);      // register 0x1F,Low Byte Event0 Timeout
	SPI_transmit(0xFB);      // register 0x20,Wake On Radio Control
	SPI_transmit(0x56);      // register 0x21,Front End RX Configuration
	SPI_transmit(0x10);      // register 0x22,Front End TX Configuration

	SPI_transmit(0xE9);      // register 0x23,Frequency Synthesizer Calibration3:FSCAL(Frequency synthesizer calibration configuration)=3 ?
	SPI_transmit(0x2A);      // register 0x24,Frequency Synthesizer Calibration2:high VCO
	SPI_transmit(0x00);      // register 0x25,Frequency Synthesizer Calibration1
	SPI_transmit(0x1F);      // register 0x26,Frequency Synthesizer Calibration0

	SPI_transmit(0x41);      // register 0x27,RC Oscillator Configuration
	SPI_transmit(0x00);      // register 0x28,RC Oscillator Configuration
	SPI_transmit(0x59);      // register 0x29,Frequency Synthesizer Calibration Control
	SPI_transmit(0x7F);      // register 0x2A,Production Test
	SPI_transmit(0x3F);      // register 0x2B,AGCTEST – AGC Test
	SPI_transmit(0x81);      // register 0x2C,Various Test Settings
	SPI_transmit(0x35);      // register 0x2D,Various Test Settings
	SPI_transmit(0x09);      // register 0x2E,Various Test Settings:Disable VCO selection calibration

	CSn(1);
}

int main()
{
	char i;

	DDRD = 0;
	DDRC = 0;
	SPI_MasterInit();         //initialize SPI
	Full_RegConfig();

	
	CSn(0);
	SPI_transmit(0x36); //strobe - Exit all
	SPI_transmit(0x3B); //strobe - flush TX
	CSn(1);

	while (1)
	{
		for (i=0; i<=7; i++)
		{
			if (switchOn(i)== 1)
			{
				data[0]=2;
				SendProgram();
			}
			else if (switchOnC(i)== 1)
			{
				data[0]=18;
				SendProgram();
			}
			
			else if (switchOn(i)== 2)
			{
				data[0]=4;
				SendProgram();
			}
			else if (switchOnC(i)== 2)
			{
				data[0]=20;
				SendProgram();
			}
			
			else if (switchOn(i)== 4)
			{
				data[0]=6;
				SendProgram();
			}
			else if (switchOn(i)== 8)
			{
				data[0]=8;
				SendProgram();
			}
			else if (switchOn(i)== 16)
			{
				data[0]=10;
				SendProgram();
			}
			else if (switchOn(i)== 32)
			{
				data[0]=12;
				SendProgram();
			}
			else if (switchOn(i)== 64)
			{
				data[0]=14;
				SendProgram();
			}
			else if (switchOn(i)== 128)
			{
				data[0]=16;
				SendProgram();
			}
		}
		
		
	}
}

