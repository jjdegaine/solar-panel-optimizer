/*

Test to increased dim from 1 to 128

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

//oled
//#include <wire.h>
#include "SSD1306.h"
SSD1306Wire display(0x3c, SDA, SCL);   // ADDRESS, SDA, SCL


// float Vcalibration     = 0.90;   // to obtain the mains exact value 
// float Icalibration     = 93;     // current in milliampères
// float phasecalibration = 1.7;    // value to compensate  the phase shift linked to the sensors. 
// byte totalCount        = 20;     // number of half perid used for measurement
// float ADC_V_0V = 467 ;
// float ADC_I_0A = 467 ;

// Threshold value for power adjustment: 

// int tresholdP     = 50000;           // Threshold to start power adjustment 1 = 1mW ; 

// unsigned long unballasting_timeout = 10000; // timeout to avoid relay command to often 10 secondes
// unsigned long unballasting_time;            // timer for unballasting 
// byte unballasting_counter = 0;             // counter mains half period
// byte unballasting_dim_min = 5;             // value of dim to start relay

// reaction rate coefficient
// reaction_coeff define the DIM value to be added or substract
// If too small the control loop is too slow
// if too large the control loop is unstable
// reaction_coeff ~ (control loop resistance power )/4  Watt

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
 
byte dimthreshold=30 ;					// dimthreshold; value to added at dim to compensate phase shift
byte dimmax = 128;              // max value to start SCR command
//byte dim = dimmax;              // Dimming level (0-128)  0 = on, 128 = 0ff 
byte dim = 0; // dim increased 0 to  128

byte dimphase = dim + dimthreshold; 
byte dimphasemax = dimmax + dimthreshold;

// byte reset_wifi = 0;			// counter for wifi reset due to time to leave
byte wifi_wait = 0;       // 
        

// wifi UDP

// byte ack = 0; // byte received ack from client
// byte send_UDP = 0 ; //
// byte send_UDP_max = 200; // send UDP data each 200*10 msec
 volatile bool send_UDP_wifi = false;

// unsigned long time_udp_now;
// unsigned long time_udp_limit = 5000 ; // time to leave UDP 5 sec

signed long wait_it_limit = 3 ;  // delay 3msec
signed long it_elapsed; // counter for delay 3 msec

char periodStep = 68;                            // 68 * 127 = 10msec, calibration using oscilloscope
volatile int i = 0;                              // Variable to use as a counter
volatile bool zero_cross = false;                // zero cross flag for SCR
volatile bool zero_cross_flag = false;           // zero cross flag for power calculation
volatile bool first_it_zero_cross = false ;      // flag first IT on rising edge zero cross
volatile bool wait_2msec ;



// Voltage and current measurement  :

// int readV, memo_readV, readI;   // voltage and current withn ADC (0 à 1023 bits)
// float rPower, V, I, sqV, sumV = 0, sqI, sumI = 0, instP, sumP = 0;  
// float Power_wifi;
//long Power_wifi ;                   // power to be sent by wifi
// char mystring_power_wifi [50] ;       // string to be transmitted by wifi
byte zero_crossCount = 0;          // half period counter
    
// other value :

int dimstep;                    // DIM step value 

unsigned int memo_temps = 0;   


// bool relay_1 ; // Flag relay 1
// bool relay_2 ; // Flag relay 2

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
     portEXIT_CRITICAL_ISR(&mux);
     zero_cross_flag = true;   // Flag for power calculation
     zero_cross = true;        // Flag for SCR
     first_it_zero_cross = true ;  // flag to start a delay 2msec
     digitalWrite(SCRLED, LOW); //reset SCR LED
     
    //   send_UDP ++ ;
    //  if (send_UDP > send_UDP_max)
    //  {
    //    send_UDP=0; // reset counter send_UDP
    //    send_UDP_wifi = true ; // ready to send UDP 
    //  }
   
}  



/* _________________________________________________________________
 *
 * IT timer task
 * _________________________________________________________________
*/ 
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  
  portEXIT_CRITICAL_ISR(&timerMux);
  
   if(zero_cross == true && dimphase < dimphasemax )  // First check to make sure the zero-cross has 
 {                                                    // happened else do nothing

      
     
     if(i>dimphase) {            // i is a counter which is used to SCR command delay 
                                // i minimum ==> start SCR just after zero crossing half period ==> max power
                                // i maximum ==> start SCR at the end of the zero crossing half period ==> minimum power
       digitalWrite(SCR_pin, HIGH);     // start SCR
       delayMicroseconds(50);             // Pause briefly to ensure the SCR turned on
       digitalWrite(SCR_pin, LOW);      // Turn off the SCR gate, 
       i = 0;                             // Reset the accumulator
       digitalWrite(SCRLED, HIGH);      // start led SCR 
       zero_cross = false;
     } 
    else {  
      i++; 
      }           // If the dimming value has not been reached, incriment our counter
     
 }      // End zero_cross check

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

//unballasting_time= millis(); // set up timer unballasting


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

   
  
 
}                

//____________________________________________________________________________________________
// End setup
//____________________________________________________________________________________________


                              



void loop()
 {
 


// function delay 2msec

    if (first_it_zero_cross == true  )            // first IT on rising edge ==> start a delay during 3msec to avoid false zero cross detection
      {            
       
       it_elapsed = millis () + wait_it_limit;
      
       detachInterrupt(digitalPinToInterrupt(zeroCrossPin)); // invalid interrupt during 3msec to avoid false interrupt during falling edge
       first_it_zero_cross = false;      // flag for IT zero_cross
       wait_2msec = true ;
      }
      
      if (wait_2msec == true && long (millis() - it_elapsed) >= 0 )        // check if delay > 3msec to validate interrupt zero cross, wait_it is incremeted by it timer ( 75usec)
      {
      
        attachInterrupt(digitalPinToInterrupt(zeroCrossPin), zero_cross_detect, RISING);
        wait_2msec=false ; 
      }

  
if ( dim >= 128) { 
  dim =0;
  }
else{
dimphase = dim + dimthreshold; // Value to used by the timer interrupt due to real phase between interruption and mains
 dim++ ;
          display.setColor(BLACK);        // clear first line
          display.fillRect(0, 0, 128, 22);
          display.setColor(WHITE); 

          display.drawString(0, 0, String (dim));
          display.display();
          Serial.println (dim);
          
 delay (10000) ; // 10 secondes
}

  }
  

