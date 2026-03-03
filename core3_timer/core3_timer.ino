#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#include <Esp.h>

/OTA
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>

const char* hostname = "ESP32 test timer core 3";

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

  // Timer 0 avec prescaler 80 → 1 tick = 1 µs (80 MHz / 80)
  timer = timerBegin(1000000);  // fréquence 1 MHz

  // Attache l'interruption
  timerAttachInterrupt(timer, &onTimer);

  // Déclenche toutes les 73 µs = 73*128 => 10ms
  timerAlarm(timer, 73 , true, 0);

  timerStart(timer);

  // init wifi
  
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
  Serial.println("HTTP server started OTA version 2026_02_28_h13_mn40");
 
  ElegantOTA.begin(&server);  // Start ElegantOTA

}

void loop()
{
  if (flag_timer)
  {
    flag_timer = false;

    // Code exécuté toutes les 73 us
    //Serial.println("Tick 73us");
    
    //OTA

  ElegantOTA.loop();
  
  }
}