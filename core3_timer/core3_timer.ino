#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#include <Esp.h>

#include <WiFi.h>

const char *ssid = "freebox_ZPRLHQ_2GEXT";  // SSID WiFi
const char *password = "Cairojude58";       // mot de passe WiFi


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

hw_timer_t *timer = NULL;
volatile bool flag_timer = false;

// Routine d'interruption
void IRAM_ATTR onTimer()
{
  flag_timer = true;   // faire le minimum dans l'ISR !
}

void setup()
{
  Serial.begin(115200);
  // init wifi
  delay(2000);
  // try i2c bus recovery at 100kHz = 5uS high, 5uS low
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

  
}

void loop()
{
  if (flag_timer)
  {
    flag_timer = false;

    // Code exécuté toutes les 73 us
    //Serial.println("Tick 73us");
    
    //OTA
  
  }
  ElegantOTA.loop();
}