#include "BluetoothSerial.h"
#define RELAY_PIN 2 
BluetoothSerial SerialBT;


//String MACadd = "00:19:10:1109:6B:F3";
String MACadd = "00:21:11:01:1D:65";
uint8_t address[6]  = {0x00, 0x21, 0x11, 0x01, 0x1D, 0x65};
//uint8_t address[6]  = {0x11, 0x1D, 0xA5, 0x02, 0xC3, 0x22};
String name = "HC-06";
char *pin = "1234"; //<- standard pin would be provided by default
bool connected;

void setup() {
  Serial.begin(9600);
  SerialBT.begin("ESPBT", true); 
  SerialBT.setPin(pin);
  Serial.println("The device started in master mode, make sure remote BT device is on!");
  
  connected = SerialBT.connect(address);
  
  if(connected) {
    Serial.println("Connected Succesfully!");
  } else {
    while(!SerialBT.connected(10000)) {
      Serial.println("Failed to connect. Make sure remote device is available and in range, then restart app."); 
    }
  }
}

void loop() {

  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }
  delay(5);
}