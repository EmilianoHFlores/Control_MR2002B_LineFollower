//Reciever
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#define MAX_STRING_SIZE 4 // Maximum string length you expect to receive

char received_buffer[MAX_STRING_SIZE];
uint8_t received_buffer_index = 0;

RF24 radio(9, 8);
const byte address[6] = "10001";
bool receivedData = false; // Flag to track received data

uint8_t rpms;

void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.openReadingPipe(1, address); // Use the same address as the transmitter
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening(); // Start listening for incoming data
}

void loop() {
  if (radio.available()) {
    radio.read(received_buffer, MAX_STRING_SIZE - 1);
    uint8_t value = received_buffer[0];
    Serial.print("Value: ");
    Serial.println(value);

  }
}