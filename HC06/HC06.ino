/*

Test to increased dim from 0 to 3 and send a command to a serial port HC06


*/




#include <Esp.h>

//oled
//#include <wire.h>
#include "SSD1306.h"
SSD1306Wire display(0x3c, SDA, SCL);   // ADDRESS, SDA, SCL


unsigned int reaction_coeff  = 90; 

// Input and ouput of the ESP32

const byte SCR_pin           = 5;
const byte pin_winter        = 14;
const byte unballast_relay2  = 15;    
const byte unballast_relay1  = 17;    
const byte SCRLED            = 16;     
const byte limiteLED         = 18;   
const byte pin_verbose       = 26;
const byte pin_calibration   = 27;
const byte voltageSensorPin  = 34;     
const byte currentSensorPin  = 35;      
const byte zeroCrossPin      = 19;      

#define TXD0 1 
#define RXD0 3



float dim = 0; // dim increased 0 to  3



      

unsigned long time_now;
unsigned long time_limit = 5000 ; // time 1 sec



    

//_____________________________________________________________________________________________
//
// SETUP
//_____________________________________________________________________________________________

void setup() {                  // Begin setup

 pinMode(SCR_pin, OUTPUT);            // Set the SCR pin as output
 pinMode(unballast_relay1, OUTPUT);    // Set the Delest pin as output
 pinMode(unballast_relay2, OUTPUT);    // Set the Delest pin as output
 pinMode(SCRLED,  OUTPUT);            // Set the LED pin as output
 pinMode(limiteLED, OUTPUT);            // Set the limite pin LED as output
 pinMode(zeroCrossPin, INPUT_PULLUP);   // set the zerocross pin as in with pullup for interrupt
 pinMode(pin_winter, INPUT); 
 pinMode(pin_verbose, INPUT);    
 pinMode(pin_calibration, INPUT); 

time_now= millis(); // set up timer 

//RS232 init
//Serial2.begin(9600, SERIAL_8N1, RXD0, TXD0);

 //Serial2.println("serial2test");
 
// USB init
 Serial.begin(9600);


 // init OLED
display.init();
display.flipScreenVertically();
display.setFont(ArialMT_Plain_24);
display.drawString(0, 0, "Ready");
display.display();


 Serial.println ();

 Serial.println(); 
 Serial.println("Ready ...");


 Serial.println (dim);
 delay(500); 


  display.drawString(0, 22, "GO");
  display.display();
  
 
  
 
}                

//____________________________________________________________________________________________
// End setup
//____________________________________________________________________________________________


                              



void loop()
 {
 

if (long (millis() - time_now > time_limit)) 
{

    if ( dim >= 4) { 
      dim =0;
      }
    else{
   
              display.setColor(BLACK);        // clear first line
              display.fillRect(0, 0, 128, 22);
              display.setColor(WHITE); 

              display.drawString(0, 0, String (dim));
              display.display();
              Serial.println (dim);
            
          dim++ ;
      }        
  time_now= millis() ;

}

  }
  
