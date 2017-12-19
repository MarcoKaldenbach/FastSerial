/*
  FastSerial.h - Fast serial library headerfile for Arduino
  Copyright (c) 2017 Marco Kaldenbach (marcokaldenbach@gmail.com).  All right reserved.

  Implemented the basics derived from the Arduino HardwareSerial driver:
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.
  
  See also the LICENSE file for licenses
*/

#ifndef FastSerial_h
#define FastSerial_h

#include <inttypes.h>

#include <avr/interrupt.h>

#include <stddef.h>

/* SERIAL_BUFFER_SIZE and ring_buffer, same definitions as HardwareSerial.cpp */
#if (RAMEND < 1000)
  #define SERIAL_BUFFER_SIZE 16
#else
  #define SERIAL_BUFFER_SIZE 64
#endif

struct ring_buffer
{
  unsigned char buffer[SERIAL_BUFFER_SIZE];
  volatile unsigned int head;
  volatile unsigned int tail;
};

/* unions for the UCSRA, UCSRB, UCSRC definitions as decribed by Mika Tuupola (https://www.appelsiini.net/2011/simple-usart-with-avr-libc) */
typedef union
{
    struct
    {
        // UCSRA bit values:
        unsigned MPCM   :  1;   // bit 0	MPCM	Multi-processor Communication Mode. When set incoming data is ignored if no addressing information is provided.
        unsigned U2X    :  1;   // bit 1	U2X     USART Double Transmission Speed. When set decreases the bit time by half doubling the speed.
        unsigned UPE    :  1;   // bit 2	UPE     USART Parity Error. Set when next frame in the UDR0 has a parity error.
        unsigned DOR    :  1;   // bit 3	DOR     Data OverRun. Set when the UDR0 was not read before the next frame arrived.
        unsigned FE     :  1;   // bit 4	FE      Frame Error. Set when next byte in the UDR0 register has a framing error.
        unsigned UDRE   :  1;   // bit 5	UDRE	USART Data Register Empty. Set when the UDR0 register is empty and new data can be transmitted.
        unsigned TXC    :  1;   // bit 6	TXC     USART Transmit Complete. Set when all data has transmitted.
        unsigned RXC    :  1;   // bit 7	RXC     USART Receive Complete. Set when data is available and the data register has not be read yet.
    } bits;
    uint8_t UCSRA;
} UCSRA_bits;


typedef union
{
    struct
    {
        // UCSRB bit values:
        unsigned TXB8   :  1;   // bit 0	TXB8	Transmit Data Bit 8. When using 8 bit transmission the 8th bit to be submitted.
        unsigned RXB8   :  1;   // bit 1	RXB8	Receive Data Bit 8. When using 8 bit transmission the 8th bit received.
        unsigned UCSZ2  :  1;   // bit 2	UCSZ2	USART Character Size 0. Used together with UCSZ01 and UCSZ00 to set data frame size. Available sizes are 5-bit (000), 6-bit (001), 7-bit (010), 8-bit (011) and 9-bit (111).
        unsigned TXEN   :  1;   // bit 3	TXEN	Transmitter enable. Set to enable transmitter.
        unsigned RXEN   :  1;   // bit 4	RXEN	Receiver Enable. Set to enable receiver.
        unsigned UDRIE  :  1;   // bit 5	UDRIE	USART Data Register Empty Interrupt Enable. Set to allow data register empty interrupts.
        unsigned TXCIE  :  1;   // bit 6	TXCIE	TX Complete Interrupt Enable. Set to allow transmission complete interrupts.
        unsigned RXCIE  :  1;   // bit 7	RXCIE	RX Complete Interrupt Enable. Set to allow receive complete interrupts.
    } bits;
    uint8_t UCSRB;
} UCSRB_bits;


typedef union
{
    struct
    {
        // UCSRC bit values:
        unsigned UCPOL  :  1;   // bit 0    UCPOL	USART Clock Polarity. Set to transmit on falling edge and sample on rising edge. Unset to transmit on rising edge and sample on falling edge.
        unsigned UCSZ0  :  1;   // bit 1    UCSZ0   see line below
        unsigned UCSZ1  :  1;   // bit 2    UCSZ1   see line below
                                                // USART Character Size 1 and 0. Used together with with UCSZ2 to set data frame size. Available sizes are 5-bit (000), 6-bit (001), 7-bit (010), 8-bit (011) and 9-bit (111).
        unsigned USBS   :  1;   // bit 3	USBS	USART Stop Bit Select. Set to select 1 stop bit. Unset to select 2 stop bits.
        unsigned UPM0   :  1;   // bit 4    UPM0    see line below
        unsigned UPM1   :  1;   // bit 5    UPM1    see line below
                                                // USART Parity Mode 1 and 0. UPM1 and UPM0 select the parity. Available modes are none (00), even (10) and odd (11).
        unsigned UMSEL0 :  1;   // bit 6    UMSEL0  see line below
        unsigned UMSEL1 :  1;   // bit 7    UMSEL1  see line below
                                                // USART Mode Select 1 and 0. UMSEL1 and UMSEL0 combined select the operating mode. Available modes are asynchronous (00), synchronous (01) and master SPI (11).
    } bits;
    uint8_t UCSRC;
} UCSRC_bits;



class FastSerial {
public:
    FastSerial( volatile uint8_t * ubrrh, volatile uint8_t *ubrrl,
                volatile uint8_t *ucsra, volatile uint8_t *ucsrb,
                volatile uint8_t *ucsrc, volatile uint8_t *udr);

    void begin(long baud);
    void interrupt_init(void);
    size_t write(uint8_t data);
    size_t write(uint8_t* data, uint8_t data_size);
    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n) { return write((uint8_t)n); }
    inline size_t write(unsigned int n) { return write((uint8_t)n); }
    inline size_t write(int n) { return write((uint8_t)n); }
    int read();
    int available();
    void received_interrupt();

private:


protected:
  // http://www.atmel.com/Images/Atmel-2549-8-bit-AVR-Microcontroller-ATmega640-1280-1281-2560-2561_datasheet.pdf
  volatile uint8_t * const _ubrrh; // USART Baud Rate Register High Byte (only 4 bits)
  volatile uint8_t * const _ubrrl; // USART Baud Rate Register Low Byte

  // https://www.appelsiini.net/2011/simple-usart-with-avr-libc
  volatile UCSRA_bits * const _ucsra;    // UCSRA  8 setting bits: | RXC    |  TXC    |  UDRE  |  FE   |  DOR  |  UPE   |  U2X   |  MPCM  |
  volatile UCSRB_bits * const _ucsrb;    // UCSRB  8 setting bits: | RXCIE  |  TXCIE  |  UDRIE |  RXEN |  TXEN |  UCSZ2 |  RXB8  |  TXB8  |
  volatile UCSRC_bits * const _ucsrc;    // UCSRC  8 setting bits: | UMSEL1 |  UMSEL0 |  UPM1  |  UPM0 |  USBS |  UCSZ1 |  UCSZ0 |  UCPOL |

  volatile uint8_t * const _udr;    // USART Data Register
 
  ring_buffer _rx_buffer;
};

/* Forward the FastSerial objects */
#if defined(UBRRH) || defined(UBRR0H)
  extern FastSerial FastSerial0;
#endif
#if defined(UBRR1H)
  extern FastSerial FastSerial1;
#endif
#if defined(UBRR2H)
  extern FastSerial FastSerial2;
#endif
#if defined(UBRR3H)
  extern FastSerial FastSerial3;
#endif


#endif /* FastSerial_h */
