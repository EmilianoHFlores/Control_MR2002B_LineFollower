#ifndef RadioReceiver_h
#define RadioReceiver_h

#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define MAX_STRING_SIZE 4 // Maximum string length you expect to receive

class RadioReceiver{
  private:
    //Reciever
    char received_buffer[MAX_STRING_SIZE];
    uint8_t received_buffer_index = 0;

    RF24 radio;
    const byte address[6] = "10001";
    bool receivedData = false; // Flag to track received data

    uint8_t rpms;
  public:

    // Methods.
    void initRadio();
    bool receiveData(uint8_t *received_buffer);
};

#endif