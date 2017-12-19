/*
  FastSerial.cpp - Fast serial library for Arduino
  Copyright (c) 2017 Marco Kaldenbach (marcokaldenbach@gmail.com).  All right reserved.

  Implemented the basics derived from the Arduino HardwareSerial driver:
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.
  
  See also the LICENSE file for licenses
*/


#include "FastSerial.h"


/* Define the Receive Interrupt function when it has not been defined already */
#if !defined(USART0_RX_vect) && defined(USART1_RX_vect)
// do nothing - on the 32u4 the first USART is USART1
#else
#if !defined(USART_RX_vect) && !defined(USART0_RX_vect) && \
    !defined(USART_RXC_vect)
  #error "Don't know what the Data Received vector is called for the first UART"
#elif !defined(serialEvent_implemented)
  //only define interrupt when it has not already been defined
  #define serialEvent_implemented
#if defined(USART_RX_vect)
  ISR(USART_RX_vect)
#elif defined(USART0_RX_vect)
  ISR(USART0_RX_vect) 
#elif defined(USART_RXC_vect)
  ISR(USART_RXC_vect) // ATmega8
#endif
  {
    FastSerial0.received_interrupt();
  }
#endif
#endif

#if defined(USART1_RX_vect) && !defined(serialEvent1_implemented)
  #define serialEvent1_implemented
ISR(USART1_RX_vect)
{
  FastSerial1.received_interrupt();
}
#endif

#if defined(USART2_RX_vect) && defined(UDR2) && !defined(serialEvent2_implemented)
  #define serialEvent2_implemented
ISR(USART2_RX_vect)
{
  FastSerial2.received_interrupt();
}
#endif

#if defined(USART3_RX_vect) && defined(UDR3) && !defined(serialEvent3_implemented)
  #define serialEvent3_implemented
ISR(USART3_RX_vect)
{
  FastSerial3.received_interrupt();
}
#endif

/* store_char(unsigned char, ring_buffer *) function copied from HardwareSerial.cpp */
inline void store_char(unsigned char c, ring_buffer *buffer)
{
  int i = (unsigned int)(buffer->head + 1) % SERIAL_BUFFER_SIZE;

  // if we should be storing the received character into the location
  // just before the tail (meaning that the head would advance to the
  // current location of the tail), we're about to overflow the buffer
  // and so we don't write the character or advance the head.
  if (i != buffer->tail) {
    buffer->buffer[buffer->head] = c;
    buffer->head = i;
  }
}

FastSerial::FastSerial(volatile uint8_t * ubrrh, volatile uint8_t *ubrrl,
                       volatile uint8_t *ucsra, volatile uint8_t *ucsrb,
                       volatile uint8_t *ucsrc, volatile uint8_t *udr):
                       _ubrrh(ubrrh),
                       _ubrrl(ubrrl),
                       _ucsra((volatile UCSRA_bits*)ucsra),
                       _ucsrb((volatile UCSRB_bits*)ucsrb),
                       _ucsrc((volatile UCSRC_bits*)ucsrc),
                       _udr(udr)
{
  //Initialise the rx head and tail values
  _rx_buffer.head = 0;
  _rx_buffer.tail = 0;
}

void FastSerial::begin(long baud)
{
    cli();
    *_ubrrh = ((F_CPU / 16 + baud / 2) / baud - 1) >> 8;
    *_ubrrl = ((F_CPU / 16 + baud / 2) / baud - 1); 				// Set the baud rate prescale rate register
    sei();
    
    interrupt_init();
}

void FastSerial::interrupt_init(void)
{
    cli();
    // Macro to determine the baud prescale rate see table 22.1 in the Mega datasheet

    // Enable receiver and transmitter and Rx interrupt
    _ucsrb->bits.RXEN = 1;
    _ucsrb->bits.TXEN = 1;
    _ucsrb->bits.RXCIE = 1;

    // Set frame format: 8data, 1 stop bit. See Table 22-7 for details
    _ucsrc->bits.USBS = 1;
    _ucsrc->bits.UCSZ0= 1;
    _ucsrc->bits.UCSZ1= 1;
    
    sei();
}

size_t FastSerial::write(uint8_t data)
{
    //loop while the transmit buffer is not empty
    while(_ucsra && _ucsra &&_ucsra->bits.UDRE != 1);

    //when the buffer is empty write data to the transmitted
    *_udr = data;
    
    return 1;
}

size_t FastSerial::write(uint8_t* data, uint8_t data_size)
{
    size_t N = 0;
    for(int i = 0; i < data_size; i++)
        N += write(data[i]);
    return N;
}

int FastSerial::read()
{
  // if the head isn't ahead of the tail, we don't have any characters
  if (_rx_buffer.head == _rx_buffer.tail) {
    return -1;
  } else {
    unsigned char c = _rx_buffer.buffer[_rx_buffer.tail];
    _rx_buffer.tail = (unsigned int)(_rx_buffer.tail + 1) % SERIAL_BUFFER_SIZE;
    return c;
  }
}

int FastSerial::available()
{
  return (int)(SERIAL_BUFFER_SIZE + _rx_buffer.head - _rx_buffer.tail) % SERIAL_BUFFER_SIZE;
}

void FastSerial::received_interrupt() /* Handle the receive interrupt */
{
    if (bit_is_clear(*_ucsra, _ucsra->bits.UPE)) {  
      char c = *_udr;
      store_char(c, &_rx_buffer);
    }
}

/*--------------------- Preinstantiate Objects ---------------------*/
#if defined(UBRRH) && defined(UBRRL)
  FastSerial FastSerial0(&UBRRH, &UBRRL, &UCSRA, &UCSRB, &UCSRC, &UDR);
#elif defined(UBRR0H) && defined(UBRR0L)
  FastSerial FastSerial0(&UBRR0H, &UBRR0L, &UCSR0A, &UCSR0B, &UCSR0C, &UDR0);
#else
  #error no serial port defined  (port 0)
#endif

#if defined(UBRR1H)
  FastSerial FastSerial1(&UBRR1H, &UBRR1L, &UCSR1A, &UCSR1B, &UCSR1C, &UDR1);
#endif
#if defined(UBRR2H)
  FastSerial FastSerial2(&UBRR2H, &UBRR2L, &UCSR2A, &UCSR2B, &UCSR2C, &UDR2);
#endif
#if defined(UBRR3H)
  FastSerial FastSerial3(&UBRR3H, &UBRR3L, &UCSR3A, &UCSR3B, &UCSR3C, &UDR3);
#endif
