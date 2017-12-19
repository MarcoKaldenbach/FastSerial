# FastSerial
These files will make it possible for Arduino's to communicate faster over serial connections. 

Steps to make the code work:
1. Implement the FastSerial.cpp and FastSerial.h file to your Arduino project.

2. Comment the HardwareSerial Receive Interrupt function(s) in the Arduino core:
  - Open the HardwareSerial.cpp file (C:\Program Files (x86)\Arduino\hardware\arduino\cores\arduino\HardwareSerial.cpp)
  - Comment the Interrupt
  
      For Serial:
      ```c++
      /*
    #if !defined(USART0_RX_vect) && defined(USART1_RX_vect)
    // do nothing - on the 32u4 the first USART is USART1
    #else
    #if !defined(USART_RX_vect) && !defined(USART0_RX_vect) && \
        !defined(USART_RXC_vect)
      #error "Don't know what the Data Received vector is called for the first UART"
    #else
      void serialEvent() __attribute__((weak));
      void serialEvent() {}
      #define serialEvent_implemented
    #if defined(USART_RX_vect)
      ISR(USART_RX_vect)
    #elif defined(USART0_RX_vect)
      ISR(USART0_RX_vect) 
    #elif defined(USART_RXC_vect)
      ISR(USART_RXC_vect) // ATmega8
    #endif
      {
      #if defined(UDR0)
        if (bit_is_clear(UCSR0A, UPE0)) {
          unsigned char c = UDR0;
          store_char(c, &rx_buffer);
        } else {
          unsigned char c = UDR0;
        };
      #elif defined(UDR)
        if (bit_is_clear(UCSRA, PE)) {
          unsigned char c = UDR;
          store_char(c, &rx_buffer);
        } else {
          unsigned char c = UDR;
        };
      #else
        #error UDR not defined
      #endif
      }
    #endif
    #endif
    */ 
    ```
      For Serial1:
      ```c++
    /*   
    #if defined(USART1_RX_vect)
      void serialEvent1() __attribute__((weak));
      void serialEvent1() {}
      #define serialEvent1_implemented
      ISR(USART1_RX_vect)
      {
        if (bit_is_clear(UCSR1A, UPE1)) {
          unsigned char c = UDR1;
          store_char(c, &rx_buffer1);
        } else {
          unsigned char c = UDR1;
        };
      }
    #endif
    */
    ```
      For Serial2:
      ```c++
    /*   
    #if defined(USART2_RX_vect)
      void serialEvent2() __attribute__((weak));
      void serialEvent2() {}
      #define serialEvent2_implemented
      ISR(USART2_RX_vect)
      {
        if (bit_is_clear(UCSR2A, UPE2)) {
          unsigned char c = UDR2;
          store_char(c, &rx_buffer2);
        } else {
          unsigned char c = UDR2;
        };
      }
    #endif
    */
    ```
      For Serial3:
      ```c++
    /*   
    #if defined(USART3_RX_vect)
      void serialEvent3() __attribute__((weak));
      void serialEvent3() {}
      #define serialEvent3_implemented
      ISR(USART3_RX_vect)
      {
        if (bit_is_clear(UCSR3A, UPE3)) {
          unsigned char c = UDR3;
          store_char(c, &rx_buffer3);
        } else {
          unsigned char c = UDR3;
        };
      }
    #endif
    */
    ```
  - Save HardwareSerial.cpp
  
3. Use the FastSerial object:
  ```c++
  #include "FastSerial.h"
  void setup()
  {
    //Initialise the FastSerial object with a baudrate of 1Mbps (1000000bps)
    FastSerial0.begin(1000000); //Or use the other serial: FastSerial1, FastSerial2, FastSerial3
  }

  void loop()
  {
    //Wait untill data becomes available
    if(FastSerial0.available())
    {
        //Response the same data as been received
        FastSerial0.write(FastSerial0.read());
    }
  }
  ```
