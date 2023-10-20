#include "RadioReceiver.h"


void RadioReceiver::initRadio(){
  //Reciever
  radio.begin();
  radio.openReadingPipe(1, address); // Use the same address as the transmitter
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening(); // Start listening for incoming data
}

bool RadioReceiver::receiveData(uint8_t *value){
    uint8_t received_buffer[MAX_STRING_SIZE];
  if (radio.available()) {
    radio.read(received_buffer, MAX_STRING_SIZE - 1);
    value = &received_buffer[0];
    Serial.print("Value: ");
    Serial.println(*value);
    return true;
  }
    return false;

}