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
uint16_t position_y =0 ;
uint16_t xo =0 ;

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

void loop() {

  if (SerialBT.available()) {
    //Serial.write(SerialBT.read());
    M5.Lcd.setCursor(0, position_Y);
    M5.Lcd.print(SerialBT.read());
  }
  position_y=position_y+30 ;
  display = display + 1 ;
  delay(5);

   if ( display == 4) {
      M5.Lcd.clear(BLACK);
      position_y = 0;
      display = 0 ;
   }
    
}