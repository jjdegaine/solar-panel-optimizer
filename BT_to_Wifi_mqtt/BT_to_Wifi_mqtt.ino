#include "BluetoothSerial.h"
#include <M5Stack.h>
#include <Esp.h>
#include <WiFi.h>
#include <esp_task_wdt.h>
#include <PubSubClient.h> //MQTT


BluetoothSerial SerialBT;



String MACadd = "00:21:11:01:1D:65";
uint8_t address[6]  = {0x00, 0x21, 0x11, 0x01, 0x1D, 0x65};
String name = "HC-06";
char *pin = "1234"; //<- standard pin would be provided by default
bool connected;
byte display =0 ; //
int position_y =0 ;
byte DataToRead [13]; 

void setup() {
  Serial.begin(9600);
  SerialBT.begin("ESPBT", true); 
  M5.begin();
  M5.Power.begin();
 // SerialBT.setPin(pin);

   // Initialize the M5Stack object
  
  M5.Lcd.clear(BLACK);
  M5.Lcd.setCursor(0,0);
  M5.Lcd.setTextSize(4);
  M5.Lcd.print("ready ...");
  delay(1000);

  M5.Lcd.print("The device started in master mode, make sure remote BT device is on!");
  
  connected = SerialBT.connect(address);
  
  if(connected) {
    M5.Lcd.setCursor(0,0);
    M5.Lcd.clear(BLACK);
    M5.Lcd.print("Connected Succesfully!");
  } else {
    while(!SerialBT.connected(10000)) {
      M5.Lcd.setCursor(0,0);
      M5.Lcd.clear(BLACK);
      M5.Lcd.print("Failed to connect. Make sure remote device is available and in range, then restart app."); 
    }
  }
}

/*
void loop() {

  if (SerialBT.available()) {
   
    
  int data = SerialBT.readBytesUntil(char(13), DataToRead, 13);
  M5.Lcd.print (data) ;
    //M5.Lcd.print(SerialBT.read());
  }
  
 
}
*/


void loop() {

  // if there's any serial available, read it:

  if (SerialBT.available()) {

    // look for the next valid integer in the incoming serial stream:

    int power = Serial.parseInt();

    // do it again:

    int relay1 = Serial.parseInt();

    // do it again:

    int relay2 = Serial.parseInt();

     // do it again:

    int dim = Serial.parseInt();

    // look for the newline. That's the end of your sentence:

    if (SerialBT.read() == '\n') {

      // constrain the values to 0 - 255 and invert

      // if you're using a common-cathode LED, just use "constrain(color, 0, 255);"

      

      // print the three numbers in one string as hexadecimal:

      M5.Lcd.print(power);

      M5.Lcd.print(relay1);

      M5.Lcd.print(relay2);

      M5.Lcd.print(dim);

    }

  }
}