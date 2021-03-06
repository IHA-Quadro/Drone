#include "Receiver_MEGA.h"

byte receiverPin[8] = {1, 2, 3, 0, 4, 5, 6, 7};

volatile static uint8_t PCintLast[3];

volatile uint8_t *port_to_pcmask[] = {
	&PCMSK0,
	&PCMSK1,
	&PCMSK2
};

void initializeReceiver(int nbChannel) {

  initializeReceiverParam(nbChannel);
  
  DDRK = 0;
  PORTK = 0;
  PCMSK2 |=(1<<lastReceiverChannel)-1;
  PCICR |= 0x1 << 2;

  for (byte channel = XAXIS; channel < lastReceiverChannel; channel++)
    pinData[receiverPin[channel]].edge = FALLING_EDGE;
}

int getRawChannelValue(byte channel) {
	byte pin = receiverPin[channel];
	uint8_t oldSREG = SREG;
	cli();
	// Get receiver value read by pin change interrupt handler
	uint16_t receiverRawValue = pinData[pin].lastGoodWidth;
	SREG = oldSREG;

	return receiverRawValue;
}

void MegaPcIntISR() {
  uint8_t bit;
  uint8_t curr;
  uint8_t mask;
  uint8_t pin;
  uint32_t currentTime;
  uint32_t time;

  curr = *portInputRegister(11);
  mask = curr ^ PCintLast[0];
  PCintLast[0] = curr;

  // mask is pins that have changed. screen out non pcint pins.
  if ((mask &= PCMSK2) == 0) {
    return;
  }

  currentTime = micros();

  // mask is pcint pins that have changed.
  for (uint8_t i=0; i < 8; i++) 
	{
    bit = 0x01 << i;
    if (bit & mask) 
		{
      pin = i;
      // for each pin changed, record time of change
      if (bit & PCintLast[0]) 
			{
        time = currentTime - pinData[pin].fallTime;
        pinData[pin].riseTime = currentTime;
        if ((time >= MINOFFWIDTH) && (time <= MAXOFFWIDTH))
          pinData[pin].edge = RISING_EDGE;
        else
          pinData[pin].edge = FALLING_EDGE; // invalid rising edge detected
      }
      else 
			{
        time = currentTime - pinData[pin].riseTime;
        pinData[pin].fallTime = currentTime;
        if ((time >= MINONWIDTH) && (time <= MAXONWIDTH) && (pinData[pin].edge == RISING_EDGE)) 
				{
          pinData[pin].lastGoodWidth = time;
          pinData[pin].edge = FALLING_EDGE;
        }
      }
    }
  }
}