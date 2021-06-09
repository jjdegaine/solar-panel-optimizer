/*

Test to increased dim from 1 to 192; 
connect a standard lamp (not a led); the brightness should increase with the dim value. if brightness is at the maximum with dim=0, increase the value dimthreshold.

*/


// init to use the two core of the ESP32; one core for power calculation and one core for wifi

// #if CONFIG_FREERTOS_UNICORE
// #define ARDUINO_RUNNING_CORE 0
// #else
// #define ARDUINO_RUNNING_CORE 1
// #endif

#include <Esp.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_task_wdt.h>

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

// zero-crossing interruption  :
 
byte dimthreshold=40 ;					// dimthreshold; value to added at dim to compensate phase shift
byte dimmax = 192;              // max value to start SCR command
byte dimled = dimmax-dimthreshold ; // 

byte dim = 0; // dim increased 0 to  192
byte dim_sinus [192] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192} ;
byte dim_sinus_display= 0 ;
byte dimphase = dim + dimthreshold; 
byte dimphasemax = dimmax + dimthreshold;
byte dimphaseit = dimphase ; // dimphaseit is used during it timer

byte wifi_wait = 0;       // 
        

 volatile bool send_UDP_wifi = false;

unsigned long time_now;
unsigned long time_limit = 2500 ; // time 2000 sec

signed long wait_it_limit = 3 ;  // delay 3msec
signed long it_elapsed; // counter for delay 3 msec

char periodStep = 51;                            // 51 * 192 = 10msec, calibration using oscilloscope
volatile int i = 0;                              // Variable to use as a counter
volatile bool zero_cross = false;                // zero cross flag for SCR
volatile bool zero_cross_flag = false;           // zero cross flag for power calculation
volatile bool first_it_zero_cross = false ;      // flag first IT on rising edge zero cross
volatile bool wait_2msec ;



#define WDT_TIMEOUT 3

byte zero_crossCount = 0;          // half period counter
    
// other value :

int dimstep;                    // DIM step value 

unsigned int memo_temps = 0;   



// init timer IT
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//init external PIN IT
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// define two tasks for UI & wifi_udp
void TaskUI( void *pvParameters );
void Taskwifi_udp( void *pvParameters );




//____________________________________________________________________________________________
//
// ZERO CROSS DETECT : interruption at each mains zero cross
// the interruption is not fully in line with the real sinus ==> dimthreshold will compensate the phase
// the interruption is define on rising edge BUT due to the slope on falling edge there is a "false"
// interruption on the falling edge ==> first_it_zero_cross with 3msec time delay
//____________________________________________________________________________________________

void IRAM_ATTR zero_cross_detect() {   // 
   
     portENTER_CRITICAL_ISR(&mux);

      detachInterrupt(digitalPinToInterrupt(zeroCrossPin));

        zero_cross_flag = true;   // Flag for power calculation
        zero_cross = true;        // Flag for SCR
        first_it_zero_cross = true ;  // flag to start a delay 2msec
        dimphaseit= dimphase;
      

     portEXIT_CRITICAL_ISR(&mux);
 
}  



/* _________________________________________________________________
 *
 * IT timer task
 * _________________________________________________________________
*/ 
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);


  if(zero_cross == true && dimphaseit <= dimphasemax )  // First check to make sure the zero-cross has 
                                                        // happened else do nothing
 {                                                    
      
     
     if(i>dimphaseit) {            // i is a counter which is used to SCR command delay 
                                // i minimum ==> start SCR just after zero crossing half period ==> max power
                                // i maximum ==> start SCR at the end of the zero crossing half period ==> minimum power
       digitalWrite(SCR_pin, HIGH);     // start SCR
       delayMicroseconds(5);             // Pause briefly to ensure the SCR turned on
       digitalWrite(SCR_pin, LOW);      // Turn off the SCR gate, 
       i = 0;                             // Reset the accumulator

       digitalWrite(SCRLED, HIGH);      // start led SCR
          zero_cross = false;
          
     } 
      else {  
          i++; 
          }           // If the dimming value has not been reached, incriment our counter
   
 }      // End zero_cross check


portEXIT_CRITICAL_ISR(&timerMux);

}


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


// USB init
 Serial.begin(115200);


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
  
 // init timer 
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, periodStep , true);
  timerAlarmEnable(timer); 

 // init interrupt on PIN  zero_crossing
 attachInterrupt(digitalPinToInterrupt(zeroCrossPin), zero_cross_detect, RISING);  
 

  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch

  dim =150 ;

 
}                

//____________________________________________________________________________________________
// End setup
//____________________________________________________________________________________________


                              



void loop()
 {
 
if (dim < dimled && digitalRead (zeroCrossPin) == true ){ digitalWrite(SCRLED, LOW);}// SCR LED}
if (dim >= dimled && digitalRead (zeroCrossPin) == false ){ digitalWrite(SCRLED, LOW);}// SCR LED}

// function delay 2msec

    if (first_it_zero_cross == true  )            // first IT on rising edge ==> start a delay during 3msec to avoid false zero cross detection
      {            
       //detachInterrupt(digitalPinToInterrupt(zeroCrossPin)); // invalid interrupt during 3msec to avoid false interrupt during falling edge
       first_it_zero_cross = false;      // flag for IT zero_cross
       it_elapsed = millis () + wait_it_limit;
                   
       wait_2msec = true ;

      //dimphaseit= dimphase;


      }
      
      if (wait_2msec == true && long (millis() - it_elapsed) >= 0 )        // check if delay > 3msec to validate interrupt zero cross, wait_it is incremeted by it timer ( 75usec)
      {
        wait_2msec=false;

        attachInterrupt(digitalPinToInterrupt(zeroCrossPin), zero_cross_detect, RISING);
        
        
      }

if (long (millis() - time_now > time_limit)) 
{

    if ( dim >= 160) { 
      dim =150;
      }
    else{
    dimphase = dim + dimthreshold; // Value to used by the timer interrupt due to real phase between interruption and mains

    dim_sinus_display = dim_sinus [ dim ] ;

    dim++ ;

              display.setColor(BLACK);        // clear first line
              display.fillRect(0, 0, 128, 22);
              display.setColor(WHITE); 

              display.drawString(0, 0, String (dim));
              display.display();
              Serial.print (dim);
              Serial.print (" ");
              Serial.print (dimphaseit);
              Serial.print (" ");
              Serial.println (dimphase);
      }        
  time_now= millis() ;

  }
esp_task_wdt_reset();

}
  
