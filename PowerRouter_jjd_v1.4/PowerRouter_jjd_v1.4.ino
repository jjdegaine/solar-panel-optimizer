/*

Power_Router est un système qui permet d'utiliser l'excédent d'énergie autoproduit par 
l'allumage d'un appareil résistif (facteur de puissance proche de 1) ce qui évite l'injection 
au réseau public de distribution d'électricité.

Le principe de fonctionnement est le suivant :
- détection de phase entre courant et tension permet de savoir si on consomme ou bien on injecte
- en cas d'injection il se produit la mise en route progressive d'un dispositif d'absorption 
d'excédent de puissance 
- la mesure du courant permet d'ajuster au mieux le niveau d'absorption de cet excédent.
- Par ailleurs il est prévu une sortie temporisée de 30 secondes (paramétrable) lorsque le treshold 
d'injection est proche de 3W (paramétrable) permettant par exemple de couper l'injection d'une 
éolienne au profit de la charge de batteries. 

le programme prévoit :
- une sonde de tension : simple transfo 230V/5V Crête à Crête sur mi-tension (2.5V)
- une sonde de courant : 20A/25mA sur mi-tension (2.5V)
- un module de commande par SCR
- un dispositif de détection de passage à zéro de la sinusoïde de tension secteur (par exemple 
l'optocoupleur H11AA1)
- la bibliothèque TimeOne.h à installer et disponible là : 
http://www.arduino.cc/playground/Code/Timer
- en option un afficheur LCD 1602 avec extension I2C

La gamme de puissances testée est va de 300 à 1000W

merci à Ryan McLaughlin <ryanjmclaughlin@gmail.com> pour avoir étudié et mis au point la partie 
commande du SCR il y a quelques années et que j'ai repris dans ce programme :)
source : https://web.archive.org/web/20091212193047/http://www.arduino.cc:80/cgi-bin/yabb2/YaBB.pl?num=1230333861/15

_________________________________________________________________
|                                                               |
|       auteur : Philippe de Craene <dcphilippe@yahoo.fr        |
|           pour l' Association P'TITWATT                       |
_________________________________________________________________

Toute contribution en vue de l’amélioration de l’appareil est la bienvenue ! Il vous est juste
demandé de conserver mon nom et mon email dans l’entête du programme, et bien sûr de partager 
avec moi cette amélioration. Merci.

hronologie des versions :
version 0.5 - 3 mai 2018     - boucle de décrémentation dim --
version 0.8 - 5 juil. 2018   - 1ère version fonctionnelle, pb du pic de courant du SCR 
version 1   - 6 juil. 2018   - ajout de la bibliothèque EmonLib.h pour mesure du secteur
version 1.4 - 7 juil. 2018   - simplification des tests sur sPower et dim.
version 1.6 - 8 juil. 2018   - ajout LED d'overflow + optimisation des paramètres + tresholdPoff
version 1.8 - 24 sept 2018   - ajout du pas variable sur dim avec dimstep
version 1.9 - 12 oct. 2018   - ajout d'une sortie temporisée de 5min à tresholdPoff (25W) du treshold d'injection  
version 2.0 - 4 nov. 2018    - ajout d'un watchdog avec comptage de reset en EEPROM
version 2.2 - 7 nov. 2018    - tresholdPtempo variable à part entière pour le délestage + correction coquille
version 2.3 - 16 dec 2018    - réaménagemet des messages console pour gagner du temps
version 2.4 - 12 jan 2019    - ajout d'un afficheur LCD 1602 avec extension I2C
version 3.2 - 17 jan 2019    - gain en performances en contournant EmonLib.h
version 3.3 - 22 fev 2019    - abandon de tresholdPoff : arrêt en cas de chutte brusque d'injection 
version 3.4 - 27 avr 2019    - changement du délestage par les tresholds delestON et delestOFF
version 3.5 - 9 july 2019    - test if no energy detected which started the WatchDog
version XXX - 2020           - Modif JJ + test Github
____________________________________________________________________________________________


_____________________________________________________________________
|																                                  	|
|              modification by J.J.Delorme 2020					          	|
|																                                  	|
_____________________________________________________________________

an ESP32 is used instead of an classic arduino Atmel AVR. The goal is to use the wifi link to transmit to an another ESP32 module the energy to be used to optimize the solar panel energy

PIN description

 IN description

 - PIN 5 static relay command
 
 - PIN 14 input WINTER 

 - PIN 15 relay command 2

 - PIN 16 LED power static relay

 - PIN 17 relay command 1

 - PIN 18 overflow LED

 - PIN 19 zero cross voltage

 - PIN 21 SDA for OLED SSD1306

 - PIN 22 SCL for OLED SSD1306

 - PIN26 input VERBOSE

 - PIN27 input CALIBRATION

 - PIN34 analog for voltage measuement

 - PIN35 analog for intensity measurement

version 1.0 january 2020
version 1.1 february 2020 data sent by wifi using IT_sec
version 1.2 february 2020 relay added. calibration V I OK 
version 1.3 march 2020 

*/


// init to use the two core of the ESP32; one core for power calculation and one core for wifi

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#include <Esp.h>
#include <WiFi.h>
#include <WiFiUdp.h>

//oled
//#include <wire.h>
#include "SSD1306.h"
SSD1306Wire display(0x3c, SDA, SCL);   // ADDRESS, SDA, SCL

// initialization wifi

const int channel = 4;  // define channel 4 seems to be the best for wifi....

WiFiUDP Udp; // Creation of wifi Udp instance, UDP is used to maximized the timing transfert

unsigned int localPort = 9999;

const char *ssid = "BB9ESERVER";   // for example to be changed 
const char *password = "BB9ESERVER";  // for examplet  to be changed


IPAddress ipServidor(192, 168, 4, 1);   // default IP for server
IPAddress ipCliente(192, 168, 4, 10);   // Different IP than server


// Information to be displayed

bool CALIBRATION = false;   // to calibrate Vcalibration and Icalibration
bool VERBOSE = true ;       // to verify dim and dimstep 
bool WINTER = true	;		 	  // winter -> no wifi summer wifi


float Vcalibration     = 0.90;   // to obtain the mains exact value 
float Icalibration     = 93;     // current in milliampères
float phasecalibration = 1.7;    // value to compensate  the phase shift linked to the sensors. 
byte totalCount        = 20;     // number of half perid used for measurement
float ADC_V_0V = 467 ;
float ADC_I_0A = 467 ;

// Threshold value for power adjustment: 

int tresholdP     = 50000;           // Threshold to start power adjustment 1 = 1mW ; 

unsigned long unballasting_timeout = 10000; // timeout to avoid relay command to often 10 secondes
unsigned long unballasting_time;            // timer for unballasting 
byte unballasting_counter = 0;             // counter mains half period
byte unballasting_dim_min = 5;             // value of dim to start relay

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
byte dim = dimmax;              // Dimming level (0-128)  0 = on, 128 = 0ff 

byte dimphase = dim + dimthreshold; 
byte dimphasemax = dimmax + dimthreshold;

byte reset_wifi = 0;			// counter for wifi reset due to time to leave
byte wifi_wait = 0;       // 
        

// wifi UDP

byte ack = 0; // byte received ack from client
byte send_UDP = 0 ; //
byte send_UDP_max = 200; // send UDP data each 200*10 msec
volatile bool send_UDP_wifi = false;

unsigned long time_udp_now;
unsigned long time_udp_limit = 5000 ; // time to leave UDP 5 sec

signed long wait_it_limit = 3 ;  // delay 3msec
signed long it_elapsed; // counter for delay 3 msec

char periodStep = 68;                            // 68 * 127 = 10msec, calibration using oscilloscope
volatile int i = 0;                              // Variable to use as a counter
volatile bool zero_cross = false;                // zero cross flag for SCR
volatile bool zero_cross_flag = false;           // zero cross flag for power calculation
volatile bool first_it_zero_cross = false ;      // flag first IT on rising edge zero cross
volatile bool wait_2msec ;



// Voltage and current measurement  :

int readV, memo_readV, readI;   // voltage and current withn ADC (0 à 1023 bits)
float rPower, V, I, sqV, sumV = 0, sqI, sumI = 0, instP, sumP = 0;  
float Power_wifi;
//long Power_wifi ;                   // power to be sent by wifi
char mystring_power_wifi [50] ;       // string to be transmitted by wifi
byte zero_crossCount = 0;          // half period counter
    
// other value :

int dimstep;                    // DIM step value 

unsigned int memo_temps = 0;   


bool relay_1 ; // Flag relay 1
bool relay_2 ; // Flag relay 2

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
     
      send_UDP ++ ;
     if (send_UDP > send_UDP_max)
     {
       send_UDP=0; // reset counter send_UDP
       send_UDP_wifi = true ; // ready to send UDP 
     }
   
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

unballasting_time= millis(); // set up timer unballasting


// USB init
 Serial.begin(115200);


 //init OLED
display.init();
display.flipScreenVertically();
display.setFont(ArialMT_Plain_24);
display.drawString(0, 0, "Ready");
display.display();


 Serial.println ();

 Serial.println(); 
 Serial.println("Ready ...");
 display.drawString(0, 0, "Ready");
 display.display();

 Serial.println ();
 delay(500); 
 if( VERBOSE == true ) Serial.print("  Pu (W) || dimstep |  dim || ");
 else Serial.println("GO"); 
 Serial.println();

  display.setFont(ArialMT_Plain_24);
  display.clear();


 digitalWrite(unballast_relay1, LOW);    // unballast relay 1 init
 digitalWrite(unballast_relay2, LOW);    // unballast relay 2 init

 
  //init wifi_udp


  WiFi.softAP(ssid, password,channel);  // ESP-32 as access point
  delay(500); // Delay to wait Wifi init 
  Udp.begin(localPort);
  Serial.println("init access point UDP OK");

  display.drawString(0, 22, "UDP OK");
  display.display();
  
 // init timer 
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, periodStep , true);
  timerAlarmEnable(timer); 

 // init interrupt on PIN  zero_crossing
 attachInterrupt(digitalPinToInterrupt(zeroCrossPin), zero_cross_detect, RISING);  
  
 
 // Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(
    TaskUI
    ,  "TaskUI"   // A name just for humans
    ,  20000  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    Taskwifi_udp
    ,  "wifi_udp"
    ,  20000  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
   
  
 
}                

//____________________________________________________________________________________________
// End setup
//____________________________________________________________________________________________


                              



void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks UI ------------------*/
/*--------------------------------------------------*/
//


// 
// Power calculation using ADC value ==> rPower
//____________________________________________________________________________________________
//
 
void TaskUI(void *pvParameters)  // This is the task UI.
{
  (void) pvParameters;

  for (;;) // A Task shall never return or exit.
  {
    
  
  unsigned int numberOfSamples = 0;
  sumV = 0;
  sumI = 0;
  sumP = 0;
  unsigned int time_now_second = millis()/1000;      // timer in second





  
// à chaque passage à zéro de la tension du secteur réinitialisation périodique du compteur de 
// passage à zéro zero_crossCount lorque le nombre de cycles de mesures totalCount est atteint 


 if( zero_crossCount >= totalCount ) { zero_crossCount = 0; }


// Value are measure during totalcount half period  
  while( zero_crossCount < totalCount ) {
    if( zero_cross_flag == true ) {        // Half period counter
      zero_cross_flag = false;
      zero_crossCount++; 
    } 
    
    numberOfSamples++;                         // counter of samples U and I
    
    memo_readV = readV;                  // 
    readV = analogRead(voltageSensorPin) / 4;   // Voltage Value  0V = bit ADC_V_0V. 12bits ADC ==> /4 ==> max 1024
    
	if( memo_readV == 0 && readV == 0 ) { break; } // exit the while if no powersupply
    readI = analogRead(currentSensorPin) /4 ;   // Current value - 0A = bit ADC 12bits ADC ==> /4 ==> max 1024

  
// RMS Current and Voltage 

    if( CALIBRATION == true ) {                         // 
      sqV= (readV -ADC_V_0V) * (readV -ADC_V_0V);             // ADC_V_0V ==> 0volt 
      sumV += sqV;               
      sqI = (readI -ADC_I_0A) * (readI -ADC_I_0A);
	    sumI += sqI;
    } 
 
// instantaneous power calculation 
    instP = ((memo_readV -ADC_V_0V) + phasecalibration * ((readV -ADC_V_0V) - (memo_readV -ADC_V_0V))) * (readI -ADC_I_0A); 
    sumP +=instP;  


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

  
 }      // end while sur zero_crossCount


// Power calculation

  if( numberOfSamples > 0 ) {
    if( CALIBRATION == true ) { 
      V = Vcalibration * sqrt(sumV / numberOfSamples);
      I = Icalibration * sqrt(sumI / numberOfSamples);
    }
    rPower = Vcalibration * Icalibration * sumP / numberOfSamples;
 
    Power_wifi =  rPower/1000 ; // Power wifi using float 
  }
	
//____________________________________________________________________________________________
//
// Power to be unbalanced to avoid injection of electricity to the grid
//
//____________________________________________________________________________________________
//
// dimstep calculation.  
//
  if( rPower > 0 ) { dimstep = (rPower/1000)/reaction_coeff + 1; } 
  else { dimstep = 1 - (rPower/1000)/reaction_coeff; }
  
  // when rPower is less than tresholdP ==> unlalanced power must increased ==> DIM must be reduced

  if( rPower < tresholdP ) {      
    if( dim > dimstep )  dim -= dimstep; else  dim = 0;
  } 

// when rPower is higher than tresholdP ==> unlalanced power must decreased ==> DIM must be increasad

  else if( rPower > tresholdP ) {                   
    if( dim + dimstep < dimmax ) dim += dimstep;  else  dim = dimmax; 
  }

  if(dim < 1) { digitalWrite(limiteLED, HIGH); }  // if dim is at the minimum, control regulation is at the maximum 
  else { digitalWrite(limiteLED, LOW); }
  

dimphase = dim+ dimthreshold; // Value to used by the timer interrupt due to real phase between interruption and mains

// Relay command. to avoid control regulation with a large power (which imply large harmonic) two relay are used to command fixed power charge. 
// to avoid instability the DIM value is confirm 10 times and the relay remains stable during unballasting_timeout time
//  
  if (long (millis() - unballasting_time > unballasting_timeout))
   {
    
     if (dim < unballasting_dim_min)  // DIM is minimum => power in SCR is maximum
      {
      
        if (unballasting_counter > 10) // dim is < unballasting_dim_min during 10 half period
        {
          
          if (relay_1 == true) 
          {
            if(relay_2 == true)
              {
                unballasting_counter = 10;      // overflow
                digitalWrite(limiteLED, HIGH) ;
                
              }
            else 
              {
                digitalWrite( unballast_relay2, HIGH) ; // set relay 2 
                relay_2 =true;
                unballasting_counter= 10 ;
                unballasting_time = millis() ;
              }
          }     
          else
              {
              digitalWrite (unballast_relay1, HIGH)  ; //set relay 1
              relay_1 = true;
              unballasting_counter= 0 ;
              }     
        }  
        else
          {
            unballasting_counter ++ ; 
          }  
       unballasting_counter ++ ; 
      }
      // dim is more than unballasting_dim_min
      if (unballasting_counter > 0 ) // 
      {
        unballasting_counter -- ;
      }
      else  // unballasting_counter = 0
      {
        if (relay_2 == true)
        {
          digitalWrite (unballast_relay2, LOW) ; 
          relay_2 = false;
          unballasting_counter = 10 ;
        }
        else
        {
          digitalWrite (unballast_relay1, LOW) ;
          relay_1 = false;
          unballasting_time= millis();
        }
      }
  }

 

  // Display each 2 seconds


  if( time_now_second >= memo_temps +2 ) {

    if ( (CALIBRATION ==false) || (VERBOSE == false)) {
          memo_temps = time_now_second;


          Serial.print("P= ");
          Serial.print(rPower/1000);   
          Serial.print("w");
          Serial.print("dim: ");
          Serial.println(dim);

          display.setColor(BLACK);        // clear first line
          display.fillRect(0, 0, 128, 22);
          display.setColor(WHITE); 

          display.drawString(0, 0, String(int(Power_wifi)) + "||" + String (dim));
          display.display();
        }
      }

          // 
      
        if( CALIBRATION == true ) {
      	  Serial.print(V);
      	  Serial.print("  |  ");
          Serial.print(I/1000);
          Serial.print("  |  ");
          Serial.print(rPower/1000);
          Serial.println();

          display.clear();
          display.drawString(0, 0, String(int(V)) + "||" + String(int(I/1000))) ;
          display.drawString(0, 22, String(int(Power_wifi)));
          display.display();

        }
        if( VERBOSE == true ) {
          Serial.print(rPower/1000);
          Serial.print("  ||     ");
          Serial.print(dimstep);
          Serial.print("  ||  ");
          Serial.print(dim);
          Serial.print(" ||  ");
          Serial.print(dimphase);
          Serial.print(" ||  ");
          Serial.print (relay_1);
          Serial.print(" ||  ");
          Serial.print (relay_2);
          Serial.print(" ||  ");
          Serial.print (unballasting_counter);
          Serial.print(" ||  ");
          Serial.print (millis() - unballasting_time);
       
          Serial.println();

        }
  
        else { delay(1); }           // needed for stability

// update switches winter, verbose, calibration

       // WINTER = digitalRead (pin_winter);
        WINTER= true;
       // VERBOSE = digitalRead (pin_verbose);
        VERBOSE = true;
        //CALIBRATION = digitalRead (pin_calibration);
        CALIBRATION = false;

  } 
  
}                              // end task UI


/*--------------------------------------------------*/
/*---------------------- Tasks Wifi ----------------*/
/*--------------------------------------------------*/
//

void Taskwifi_udp(void *pvParameters)  // This is a task.
{
    (void) pvParameters;
    //String msg;
    time_udp_now= millis(); 
    Serial.println ("start task UDP");
    

    for (;;) // A Task shall never return or exit.
    {
  	   while(send_UDP_wifi == false )
  	   {
  		wifi_wait=0; // loop to wait update DIM
  	    		
  	   }

       // logic: we want wifi if not (calibration or verbose or winter)
      if (((CALIBRATION == false) && (VERBOSE == false) && (WINTER == true)))
      
      {
 
        // verification time to leave UDP
                   
              if (long (millis() - time_udp_now > time_udp_limit))             // comparing durations
              {                    
              Serial.println ("time to leave UDP");

              display.setColor(BLACK);        // clear second  line
              display.fillRect(0, 22, 128, 22);
              display.setColor(WHITE); 
              display.drawString(0, 22, "TIME UDP");
              display.display();


              WiFi.disconnect();
              WiFi.softAP(ssid, password,channel);  // ESP-32 as access point
              delay(500); //  
              Udp.begin(localPort);
              delay(5000);
              Serial.println("init access point UDP OK");

              display.setColor(BLACK);        // clear second  line
              display.fillRect(0, 22, 128, 22);
              display.setColor(WHITE); 
              display.drawString(0, 22, "UDP OK");
              display.display();
              
              time_udp_now= millis(); // reset time to leave
             
              
              }
         
      		send_UDP_wifi = false ; 
      		Udp.beginPacket(ipCliente,9999);   //Initiate transmission of data

          sprintf(mystring_power_wifi, "%g", Power_wifi); 

          Udp.print (mystring_power_wifi) ; 
       
      		Udp.endPacket();  // Close communication
        
      		// read acknowledge from client
              int packetSize = Udp.parsePacket();
              if (packetSize) 
            		{
                    int len = Udp.read(&ack, 1);
                   
                    time_udp_now= millis(); // reset time to leave
            		} 
       }

     } // end for loop wifi
    
  }