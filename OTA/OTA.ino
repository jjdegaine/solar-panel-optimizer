/*


_____________________________________________________________________
|																                                  	|
|             OTA J.J 2025					          	|
|																                                  	|
_____________________________________________________________________


 2026_03 OTA only
*/
// init to use the two core of the ESP32; one core for power calculation and one core for wifi
/*
#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif
*/
#include <Esp.h>

#include <WiFi.h>

#include "PubSubClient.h" //wifi mqtt

//OTA
//#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
//include <WebServer.h>
#include <ElegantOTA.h>

AsyncWebServer server(80); 
//WebServer server(80);

// oled

#include "SSD1306.h"
SSD1306Wire display(0x3c, SDA, SCL); // ADDRESS, SDA, SCL

// initialization wifi

const int channel = 4; // define channel 4 seems to be the best for wifi....

const char *ssid = "freebox_ZPRLHQ_2GEXT"; // SSID WiFi
const char *password = "Cairojude58";      // mot de passe WiFi
String hostname= "ESP32 OTA "; 


WiFiClient espClient;

// Input and ouput of the ESP32


const byte SDA_PIN = 21;
const byte CLK_PIN = 22;

bool stop = false ;





//
// other value :


// define two tasks for UI & wifi
void TaskUI(void *pvParameters);
void Taskwifi_udp(void *pvParameters);


//_____________________________________________________________________________________________
//
// SETUP
//_____________________________________________________________________________________________

void setup()
{ // Begin setup

  
  // USB init
  Serial.begin(115200);
  server.begin();
  ElegantOTA.begin(&server);  // Start ElegantOTA


  // work around I²C bug at start up   https://github.com/esp8266/Arduino/issues/1025

  delay(2000);
  // try i2c bus recovery at 100kHz = 5uS high, 5uS low
  pinMode(SDA_PIN, OUTPUT); // keeping SDA high during recovery
  digitalWrite(SDA_PIN, HIGH);
  pinMode(CLK_PIN, OUTPUT);
  for (int i = 0; i < 10; i++)
  { // 9nth cycle acts as NACK
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

  Serial.println();

  Serial.println();
  Serial.println("Ready ...");

  Serial.println();
  delay(500);
  
    // init wifi

  WiFi.setHostname(hostname.c_str()); //define hostname
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.println("Connecting to WiFi..");
    display.drawString(0, 0, "connecting to WiFi...");
  }
  Serial.println("Connected to the WiFi network");
  display.drawString(0, 0, "connected to WiFi");

  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! I am ESP32 OTA.");
  });

  //server.begin();
  Serial.println("HTTP server started");
  ElegantOTA.begin(&server);    // Start ElegantOTA

}

/*
  // Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(
      TaskUI, "TaskUI" // A name just for humans
      ,
      20000 // This stack size can be checked & adjusted by reading the Stack Highwater
      ,
      NULL, 2 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,
      NULL, ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
      Taskwifi_udp, "wifi_udp", 20000 // Stack size
      ,
      NULL, 1 // Priority
      ,
      NULL, ARDUINO_RUNNING_CORE);

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}
*/
//____________________________________________________________________________________________
// End setup
//____________________________________________________________________________________________

void loop()
{
 
  
   ElegantOTA.loop();

}
/*--------------------------------------------------*/
/*---------------------- Tasks UI ------------------*/
/*--------------------------------------------------*/
//

//
// Power calculation using ADC value ==> rPower
//____________________________________________________________________________________________
//
/*
void TaskUI(void *pvParameters) // This is the task UI.
{
  (void)pvParameters;

  for (;;) // A Task shall never return or exit.
  {

  Serial.println("TaskUI started");
  while (stop == false)
  { 
    stop = false ; // do nothing
  }

  } // end task UI
}
*/
/*--------------------------------------------------*/
/*---------------------- Tasks Wifi ----------------*/
/*--------------------------------------------------*/
//
/*
void Taskwifi_udp(void *pvParameters) // This is a task.
{
  (void)pvParameters;

  Serial.println("Taskwifi started");

  for (;;) // A Task shall never return or exit.
    {
    
   
      
    ElegantOTA.loop();
      
    
    }

} // end for loop wifi

*/