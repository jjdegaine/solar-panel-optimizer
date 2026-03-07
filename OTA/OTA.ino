

#include <Esp.h>
#include <WiFi.h>
//#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>

const char *ssid = "freebox_ZPRLHQ_2GEXT"; // SSID WiFi
const char *password = "Cairojude58";      // mot de passe WiFi
String hostname= "ESP32 OTA ";

AsyncWebServer server(80);

void setup()
{
  Serial.begin(115200);
  WiFi.setHostname(hostname.c_str()); //define hostname
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "ESP32 OTA");
  });

  server.begin();
  Serial.println("HTTP server started");

  ElegantOTA.begin(&server);    // Start ElegantOTA
}

void loop() 

{
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "ESP32 OTA");
  });
  
  ElegantOTA.loop();
}