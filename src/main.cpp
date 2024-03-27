#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

#define bitSet(reg, n) (reg |= 1 << n)
#define bitRead(reg, n) (reg & (1 << n))
#define bitClear(reg, n) (reg &= ~(1 << n))

#define FOSC 16000000 // Frequency Oscillator 16Mhz for Uno R3
#define BAUD 9600 // 9600 Bits per second
#define MYUBRR FOSC / 16 / BAUD - 1 // My USART Baud Rate Register

#define PORT &PORTB
#define DDR &DDRB
#define TRIGGER_PIN PINB0
#define ECHO_PIN PINB1
#define BUTTON PINB2


#define MAXDISTANCE 4000 //mm
#define SPEED .343 //mm per microsecond
#define TIMEOUT (MAXDISTANCE / SPEED) * 2 ///2 for round trip


// Unit types
enum Unit
{
  CM,
  IN
};

Unit currentUnit = CM;

// Set the baud rate
void USART_Init(unsigned int ubrr)
{
  // unsigned char is 8-bit data type so effictively gets rid of remaining bits
  // load upper 8-bits of the baud rate value into the high byte of the UBRR register
  // eg. 0b0101101010101010 >> 8 = 0b0000000001011010 and the first 8 0's ignored
  UBRR0H = (unsigned char)(ubrr >> 8);
  // load lower 8-bits of the baud rate value into the low byte of the UBRR register
  UBRR0L = (unsigned char)ubrr;
  // enable receiver and transmitter
  UCSR0B = (1 << TXEN0) | (1 << RXEN0);
  // set frame format: USBS0 = 0b1 2stop bit, UCSZ00 = 0b11 8bit data,
  UCSR0C = (1 << USBS0) | (3 << UCSZ00);
}

void USART_Transmit(unsigned char data)
{
  // UDRE0 is the bit number of USART Data Register Empty
  // In Armega328p it is bit 5.
  // Bitwise & checks if both bits are 1 and returns 1,
  while (!(UCSR0A & (1 << UDRE0)))
    ;
  // puts data ubti buffer. 8bit data register
  UDR0 = data;
}

// in C++ strings are just arrays of characters
// this passes in the pointer to first character in the string.
void txString(char *pStr)
{
  //while it doesnt have a null character
  while (*pStr != '\0')
  {
    USART_Transmit(*pStr);
    pStr++;
  }
}

// trigger the ultrasonic sensor
void PulseTrigger()
{
  bitSet(*PORT, TRIGGER_PIN);
  _delay_us(10);
  bitClear(*PORT, TRIGGER_PIN);
}

void USART_Init(unsigned int ubrr);
void USART_Transmit(unsigned char data);
void txString(char *pStr);

// get time of ultrasonic sensor
int Listen()
{
  int counter = 0;
  while(!(bitRead(PINB,ECHO_PIN)));
  while(bitRead(PINB, ECHO_PIN)){
   _delay_us(1);
   counter++;
   if (counter > TIMEOUT){
     return TIMEOUT / 2;
   }
  }
  return counter / 2;
}

// distance of ultrasonic sensor
double CalculateDistance(int time)
{
  if (currentUnit == CM)
  {
    return (time * SPEED);
  }
  else
  {
    // converts to inches
    return (time * SPEED * 0.393701);
  }
}

// toggles unit between cm and inches
void ToggleUnit()
{
  if (currentUnit == CM)
  {
    currentUnit = IN;
  }
  else
  {
    currentUnit = CM;
  }
}


int main(void)
{
  // 1 output, 0 input
  bitSet(*DDR, TRIGGER_PIN);
  bitClear(*DDR, ECHO_PIN);
  // configure as input
  bitClear(*DDR, BUTTON);
    // use internal pull up resistor
  bitSet(*PORT, BUTTON);

  USART_Init(MYUBRR);
  char txBuffer[20];
  while(1)
  {
    if (!bitRead(PINB, BUTTON))
    {
      _delay_ms(50);
      if (!bitRead(PINB, BUTTON))
      {
        ToggleUnit();
      }
     
    }
    PulseTrigger();
    int time = Listen();
    double distance = CalculateDistance(time);
    dtostrf(distance,0,2,txBuffer);
    // compiler adds null character to end of string
    // togggles unit in teleplot
    if (currentUnit == CM)
    {
      txString(">cm:");
    }
    else
    {
      txString(">in:");
    }
    txString(txBuffer);
    USART_Transmit('\n');
    _delay_ms(20);
  }
}