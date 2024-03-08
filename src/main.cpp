#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

#define bitSet(reg, n) (reg |= 1 << n)
#define bitRead(reg, n) (reg & (1 << n))
#define bitClear(reg, n) (reg &= ~(1 << n))

#define FOSC 16000000 // 16Mhz for Uno R3
#define BAUD 9600
#define MYUBRR FOSC / 16 / BAUD - 1

void USART_Init(unsigned int ubrr)
{
  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)ubrr;
  UCSR0B = (1 << TXEN0) | (1 << RXEN0);
  UCSR0C = (1 << USBS0) | (3 << UCSZ00);
}

void USART_Transmit(unsigned char data)
{
  while (!(UCSR0A & (1 << UDRE0)))
    ;
  UDR0 = data;
}

void txString(char *pStr)
{
  while (*pStr != '\0')
  {
    USART_Transmit(*pStr);
    pStr++;
  }
}

void USART_Init(unsigned int ubrr);
void USART_Transmit(unsigned char data);
void txString(char *pStr);

int main(void)
{
  USART_Init(MYUBRR);
  char txBuffer[20];
  char flag_read = 1;

  while(1)
  {
    unsigned int tx_data = 0;
    char strBuffer[4];

    while (tx_data < 256)
    {
      txString(">a:");
      txString("1");
      USART_Transmit('\n');

      tx_data++;
    }

    tx_data = 0;
    while (tx_data < 1024)
    {
      txString(">a:");
      txString("1");
      USART_Transmit('\n');

      tx_data++;
    }
  }
}