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




// init timer IT
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool flag_timer = false;

// init external PIN IT
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;


// Routine d'interruption
void IRAM_ATTR onTimer()
{
  portENTER_CRITICAL_ISR(&timerMux);

  digitalWrite(limiteLED, HIGH) ; //for scope measurement
  flag_timer = true;   // faire le minimum dans l'ISR !
  digitalWrite(limiteLED, LOW) ; //for scope measurement
  
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup()
{
  Serial.begin(115200);
  // init wifi
  delay(2000);
   pinMode(limiteLED, OUTPUT);           // Set the limite pin LED as output

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

  // Timer 0 avec prescaler 80 → 1 tick = 1 µs (80 MHz / 80)
  timer = timerBegin(1000000);  // fréquence 1 MHz

  // Attache l'interruption
  timerAttachInterrupt(timer, &onTimer);

  // Déclenche toutes les 73 µs = 73*128 => 10ms
  timerAlarm(timer, 73 , true, 0);

  timerStart(timer);

  

// work around I²C bug at start up   https://github.com/esp8266/Arduino/issues/1025
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


  if (flag_timer)
  {
    flag_timer = false;
    esp_task_wdt_reset();  // Reset WDT   
    
  
  }

  

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