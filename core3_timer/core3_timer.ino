#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#include <Esp.h>

#include <WiFi.h>

const char *ssid = "freebox_ZPRLHQ_2GEXT";  // SSID WiFi
const char *password = "Cairojude58";       // mot de passe WiFi

#include <esp_task_wdt.h>
#define WDT_TIMEOUT 5  // secondes

TaskHandle_t taskUIcHandle = NULL;
TaskHandle_t taskwifi_udpHandle = NULL;


// time for reset at 00:00
#include <time.h>

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;      // Décalage horaire (ex: France hiver = 3600)
const int   daylightOffset_sec = 3600; // Heure d'été (mettre 0 si non utilisé)

int lastDay = -1;

//OTA
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>

AsyncWebServer server(80);

const char* hostname = "ESP32 test timer core 3";

// oled

#include "SSD1306.h"
SSD1306Wire display(0x3c, SDA, SCL);  // ADDRESS, SDA, SCL
const byte SDA_PIN = 21;
const byte CLK_PIN = 22;
const byte limiteLED = 18;
const byte SCR_pin = 5;
const byte SCRLED  = 16;
#define PIN_INTERRUPT 19


// zero-crossing interruption  :
 
byte dimthreshold=30 ;					// dimthreshold; value to added at dim to compensate phase shift
byte dimmax = 128;              // max value to start SCR command

byte dim = 0; // dim increased 0 to  128
byte dim_sinus [129] = {0, 15, 27, 30, 34, 38, 40, 43, 45, 47, 48, 50, 52, 54, 55, 57, 59, 60, 62, 63, 64, 65, 67, 68, 70, 71, 73, 74, 75, 76, 77, 78, 79, 80, 81, 83, 83, 84, 85, 86, 87, 87, 88, 89, 90, 91, 92, 93, 94, 95, 95, 96, 96, 96, 97, 98, 98, 98, 99, 100, 101, 102, 102, 103, 103, 104, 104, 105, 106, 106, 106, 106, 106, 106, 107, 107, 107, 107, 107, 107, 107, 108, 108, 108, 109, 109, 109, 109, 110, 111, 112, 113, 114, 114, 115, 115, 116, 116, 117, 117, 118, 118, 119, 120, 121, 121, 122, 122, 123, 123, 124, 124, 125, 125, 126, 127, 127, 127, 127, 127, 127, 127, 128, 128, 128, 128, 128, 128, 128} ;
byte dim_sinus_display= 0 ;
byte dimphase = dim + dimthreshold; 
byte dimphasemax = dimmax + dimthreshold;
byte dimphaseit = dimphase ; // dimphaseit is used during it timer

signed long wait_it_limit = 3 ;  // delay 3msec
signed long it_elapsed; // counter for delay 3 msec

volatile int i = 0;                              // Variable to use as a counter
volatile int i_counter = 0;                      // Variable to use as a counter for SSR
volatile int I_led = 0;                          // Variable to use as a counter for LED
volatile bool zero_cross = false;                // zero cross flag for SCR
volatile bool zero_cross_flag = false;           // zero cross flag for power calculation
volatile bool first_it_zero_cross = false ;      // flag first IT on rising edge zero cross
volatile bool wait_2msec ;
volatile bool led_zero = false;

volatile uint32_t lastZeroTime = 0;

byte zero_crossCount = 0;          // half period counter

unsigned long time_now;
unsigned long time_limit = 2500 ; // time 2000 sec



// init timer IT
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool flag_timer = false;


// init external PIN IT
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

//____________________________________________________________________________________________
//
// ZERO CROSS DETECT : interruption at each mains zero cross
// the interruption is not fully in line with the real sinus ==> dimthreshold will compensate the phase
// the interruption is define on rising edge BUT due to the slope on falling edge there is a "false"
// interruption on the falling edge ==> first_it_zero_cross with 3msec time delay
//____________________________________________________________________________________________

void IRAM_ATTR isrPin19() {   // 
     portENTER_CRITICAL_ISR(&mux);
     
    // portENTER_CRITICAL_ISR(&timerMux);// critical sequence timer

  uint32_t now = micros();

  if (now - lastZeroTime > 5000)   // ignore <5 ms
  {
    digitalWrite(SCRLED, HIGH);      // for test only
    zero_cross_flag = true;   // Flag for power calculation
    zero_cross = true;        // Flag for SCR
    //first_it_zero_cross = true ;  // flag to start a delay 2msec
    led_zero = true;
    lastZeroTime = now;
    digitalWrite(SCRLED, LOW); //for test only
  }

     //portEXIT_CRITICAL_ISR(&timerMux);// critical sequence timer
     digitalWrite(SCRLED, LOW); //for test only
     portEXIT_CRITICAL_ISR(&mux);
   
}  


// Routine d'interruption timer
void IRAM_ATTR onTimer()
{
  portENTER_CRITICAL_ISR(&timerMux);

  digitalWrite(limiteLED, HIGH) ; //for scope measurement
   

  digitalWrite(limiteLED, LOW) ; //for scope measurement
  
  if (led_zero == true)
  {
    I_led = 0;
    led_zero = false;
  }
  if (led_zero == false && I_led == dimthreshold)
  {
    //digitalWrite(SCRLED, LOW); // reset SSR LED
  }
  else
  {
    I_led++;
  }

  if (zero_cross == true && dimphase < dimphasemax) // First check to make sure the zero-cross has
  {                                                 // happened else do nothing 
    if (i_counter > dimphase)
    {                              // i is a counter which is used to SSR command delay
                                   // i minimum ==> start SSR just after zero crossing half period ==> max power
                                   // i maximum ==> start SSR at the end of the zero crossing half period ==> minimum power
      digitalWrite(SCR_pin, HIGH); // start SSR
      delayMicroseconds(5);        // Pause briefly to ensure the SSR turned on
      digitalWrite(SCR_pin, LOW);  // Turn off the SSR gate,
      i_counter = 0;               // Reset the accumulator
      // digitalWrite(SCRLED, HIGH);  // start led SSR
      zero_cross = false;
    }
    else
    {
      i_counter++;
    } // If the dimming value has not been reached, incriment the counter

  } // End zero_cross check

  portEXIT_CRITICAL_ISR(&timerMux);
}



void setup()
{
  Serial.begin(115200);

  delay(2000);
   pinMode(limiteLED, OUTPUT);           // Set the limite pin LED as output
   pinMode(SCR_pin, OUTPUT);            // Set the SSR pin as output
   pinMode(SCRLED, OUTPUT);           // Set the SCR LED as output
   //pinMode(PIN_INTERRUPT, INPUT_PULLUP);  // set the zerocross pin with pullup for interrupt
    pinMode(PIN_INTERRUPT, INPUT);  // set the zerocross pin without pullup for interrupt

    // work around I²C bug at start up   https://github.com/esp8266/Arduino/issues/1025
  // INIT OLED try i2c bus recovery at 100kHz = 5uS high, 5uS low
  pinMode(SDA_PIN, OUTPUT);  // keeping SDA high during recovery
  digitalWrite(SDA_PIN, HIGH);
  pinMode(CLK_PIN, OUTPUT);
  for (int i = 0; i < 10; i++) {  // 9nth cycle acts as NACK
    digitalWrite(CLK_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(CLK_PIN, LOW);
    delayMicroseconds(5);
  }

  // a STOP signal (SDA from low to high while CLK is high)
  digitalWrite(SDA_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(CLK_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(SDA_PIN, HIGH);
  delayMicroseconds(2);
  // bus status is now : FREE

  Serial.println("bus recovery done, starting scan in 2 secs");
  // return to power up mode
  pinMode(SDA_PIN, INPUT);
  pinMode(CLK_PIN, INPUT);
  delay(2000);


  
  Wire.begin(SDA_PIN, CLK_PIN);

  // init OLED
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_24);
  display.drawString(0, 0, "Ready");
  display.display();

  
  //Wifi
  WiFi.setHostname(hostname);  //define hostname
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
    display.drawString(0, 0, "connecting to WiFi...");
  }
  Serial.println("Connected to the WiFi network");
  display.drawString(0, 0, "connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  //OTA server
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) { 
    request->send(200, "text/plain", "ESP routeur server OTA esp test timer core 3");
  });

  server.begin();
  Serial.println("HTTP server started OTA version 2026_03_03");
 
  ElegantOTA.begin(&server);  // Start ElegantOTA

   //Timer 0 avec prescaler 80 → 1 tick = 1 µs (80 MHz / 80)
  timer = timerBegin(1000000);  // fréquence 1 MHz

   //Attache l'interruption
  timerAttachInterrupt(timer, &onTimer);
  
   //init interrupt on PIN  zero_crossing
  attachInterrupt(digitalPinToInterrupt(PIN_INTERRUPT), isrPin19, RISING);

   // Déclenche toutes les 73 µs = 73*128 => 10ms
  timerAlarm(timer, 73 , true, 0);

  timerStart(timer);

  // init watchdog

  esp_task_wdt_config_t config = {
    .timeout_ms = WDT_TIMEOUT * 1000,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, // Core 0 + Core 1
    .trigger_panic = true
  };

  esp_task_wdt_init(&config);

  Serial.println("Watchdog actif sur Core 0 et Core 1");


// Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(
    TaskUI, "TaskUI"  // A name just for humans
    ,
    20000  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,
    NULL, 2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,
    NULL, ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    Taskwifi_udp, "wifi_udp", 20000  // Stack size
    ,
    NULL, 1  // Priority
    ,
    NULL, ARDUINO_RUNNING_CORE);

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

 


//____________________________________________________________________________________________
// End setup
//____________________________________________________________________________________________

void loop() {
  // Empty. Things are done in Tasks.
  
}

void TaskUI(void *pvParameters)  // This is the task UI.
{
  (void)pvParameters;

  
   // init watch dog esp core 3
    esp_task_wdt_add(NULL);                // add current thread to WDT watch

  for (;;)  // A Task shall never return or exit.
  {

  if (long (millis() - time_now > time_limit)) 
    {

    if ( dim >= 128) { 
      dim =0;
      }
    else{
    //dimphase = dim + dimthreshold; // Value to used by the timer interrupt due to real phase between interruption and mains
    dimphase = dim_sinus [ dim ] + dimthreshold; // linear sinus
    
      //portENTER_CRITICAL_ISR(&timerMux); // critical phase it timer
     // if (zero_cross == false ) {dimphaseit= dimphase;}
      //portEXIT_CRITICAL_ISR(&timerMux); // critical phase it timer

        dim_sinus_display = dim_sinus [ dim ] ;
        dim++ ;
              display.setColor(BLACK);        // clear first line
              display.fillRect(0, 0, 128, 22);
              display.setColor(WHITE); 

              display.drawString(0, 0, String (dim));
              display.display();
              Serial.print (dim);
              Serial.print (" ");
              Serial.print (dim_sinus_display);
              Serial.print (" ");
              Serial.println (dimphase);
      }        
      time_now= millis() ;

      }

       esp_task_wdt_reset();  // Reset WDT
       
   }

  }


void Taskwifi_udp(void *pvParameters)  // This is a task.
{
  (void)pvParameters;
  
  // init watch dog esp core 3
  esp_task_wdt_add(NULL);                // add current thread to WDT watch 
  
  for (;;)  // A Task shall never return or exit.
  {
    //OTA
    
    ElegantOTA.loop();
    esp_task_wdt_reset();  // Reset WDT
    
  }
}