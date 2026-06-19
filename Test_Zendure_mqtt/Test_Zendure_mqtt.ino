/*


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




Basé sur version 3.6.1 with mqtt_test pour envoie info sur MQTT depuis jardin
*/
// 
#define CORE_REALTIME  0   // Core 0 : mesure V/I/P, contrôle SSR
#define CORE_WIFI      1   // Core 1 : WiFi, MQTT, OTA

#include "secrets.h"

#include <Esp.h>

#include <WiFi.h>

#include "PubSubClient.h" //wifi mqtt


//OTA
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>

AsyncWebServer server(80); 


// oled

#include "SSD1306.h"
SSD1306Wire display(0x3c, SDA, SCL); // ADDRESS, SDA, SCL

// initialization wifi

const int channel = 4; // define channel 4 seems to be the best for wifi....


String hostname= "ESP32 test Zendure"; 

// MQTT Broker
const char *mqtt_broker = "192.168.0.154";

const char *topic_test = "routeur/Wmqtt_10mn"; // for test only

const int mqtt_port = 1883;
WiFiClient espClient;
PubSubClient client(espClient);
bool send_MQTT = false;
String client_id = "routeur";


unsigned long MQTT_timeout = 5000;       // timeout MQTT: 5 secondes
unsigned long MQTT_time;

float mean_power_MQTT_10mn = 0;
char mystring_power_wifi_10mn[50];             // string to be transmitted by wifi MQTT

// Input and ouput of the ESP32

const byte SCR_pin = 5;
const byte pin_winter = 14;
const byte unballast_relay2 = 15;
const byte unballast_relay1 = 17;
const byte SCRLED = 16;
const byte limiteLED = 18;
const byte SDA_PIN = 21;
const byte CLK_PIN = 22;
const byte pin_verbose = 26;
const byte pin_calibration = 27;
const byte voltageSensorPin = 34;
const byte currentSensorPin = 35;
const byte zeroCrossPin = 19;

// define two tasks for UI & wifi (mqtt)
void TaskUI(void *pvParameters);
void Taskwifi(void *pvParameters);





//_____________________________________________________________________________________________
//
// SETUP
//_____________________________________________________________________________________________

void setup()
{ // Begin setup

  pinMode(SCR_pin, OUTPUT);            // Set the SSR pin as output
  pinMode(unballast_relay1, OUTPUT);   // Set the Delest pin as output
  pinMode(unballast_relay2, OUTPUT);   // Set the Delest pin as output
  pinMode(SCRLED, OUTPUT);             // Set the LED pin as output
  pinMode(limiteLED, OUTPUT);          // Set the limite pin LED as output
  pinMode(zeroCrossPin, INPUT_PULLUP); // set the zerocross pin with pullup for interrupt
  pinMode(pin_winter, INPUT);
  pinMode(pin_verbose, INPUT);
  pinMode(pin_calibration, INPUT);

   

  // USB init
  Serial.begin(115200);

 

  

  // work around I²C bug at start up   https://github.com/esp8266/Arduino/issues/1025

  vTaskDelay(pdMS_TO_TICKS(2000));
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
  vTaskDelay(pdMS_TO_TICKS(2000));

  Wire.begin(SDA_PIN, CLK_PIN);

  // init OLED
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_24);
  display.drawString(0, 0, "Ready");
  display.display();

  Serial.println();

  Serial.println("Ready ...");

  
  // display.setFont(ArialMT_Plain_24);
 

    // init wifi

  WiFi.setHostname(hostname.c_str()); //define hostname
  WiFi.begin(ssid, password);
  display.clear();
  while (WiFi.status() != WL_CONNECTED)
  {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.println("Connecting to WiFi..");
    display.drawString(0, 0, "connecting");
     display.display();
  }
   display.clear();
  Serial.println("Connected to the WiFi network");
  display.drawString(0, 0, "connected");
   display.display();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
  request->send(200, "ESP32 OTA");
    });
  server.begin();
  Serial.println("HTTP server started"); 
  ElegantOTA.begin(&server);  // Start ElegantOTA

  

  // client.setCallback(callback);
  Connect_MQTT();
  
MQTT_time = millis();

xTaskCreatePinnedToCore(
    TaskUI, "TaskUI",
    20000,
    NULL, 2,          // priorité haute : temps réel
    NULL, CORE_REALTIME);

xTaskCreatePinnedToCore(
    Taskwifi, "wifi", 20000,
    NULL, 1,          // priorité basse : réseau
    NULL, CORE_WIFI);

}

//____________________________________________________________________________________________
// End setup
//____________________________________________________________________________________________

void loop ()

{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks UI ------------------*/
/*--------------------------------------------------*/
//
void TaskUI(void *pvParameters) // This is the task UI.
{
  (void)pvParameters;

  for (;;) // A Task shall never return or exit.
  {
  if (long(millis() - MQTT_time) > long(MQTT_timeout))  
        {
        
        MQTT_time = millis();
        mean_power_MQTT_10mn ++ ;
        display.clear();
        display.drawString(0, 22, String(int(mean_power_MQTT_10mn)));
        display.display();
       
        send_MQTT = true ;
        }

  }
}    
void Taskwifi(void *pvParameters) // This is a task.
{
  (void)pvParameters;

  for (;;) // A Task shall never return or exit.
  {
      if (send_MQTT)
      {
          send_MQTT= false ;

           if (Connect_MQTT()) 
          {
              sprintf(mystring_power_wifi_10mn, "%g", mean_power_MQTT_10mn);
              client.publish(topic_test, mystring_power_wifi_10mn, true);  
              Serial.print("test " );
              Serial.println(mean_power_MQTT_10mn); 
              display.drawString(0, 44, "send_MQTT");
              display.display();
          }
        
      }
      else
      {
        
        ElegantOTA.loop();
      }
  }
} // end for loop wifi

bool Connect_MQTT()
{
  if (client.connected())
  {
    return true;
  }

  while (!client.connected())
  {
       display.clear();
       client.disconnect();
    vTaskDelay(pdMS_TO_TICKS(2000)); // wait 2 secondes before connecting again.

    WiFi.disconnect();
      vTaskDelay(pdMS_TO_TICKS(2000));
    Serial.println("disConnecting to WiFi..");
        // init wifi
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(hostname.c_str()); //define hostname
    WiFi.begin(ssid, password);
      while (WiFi.status() != WL_CONNECTED)
      {
       vTaskDelay(pdMS_TO_TICKS(500));
      Serial.println("Connecting to WiFi..");
      display.drawString(0, 0, "connecting");
       display.display();
      }
      Serial.println("Connected to the WiFi network");
      display.clear();
      display.drawString(0, 0, "connected");
       display.display();

    String l_client_id = client_id;
    l_client_id += String(WiFi.macAddress());

    // init wifi mqtt
    client.setServer(mqtt_broker, mqtt_port);

    Serial.printf("The client %s connects to the public MQTT broker ", l_client_id.c_str());
    Serial.println();

    if (client.connect(l_client_id.c_str(), mqtt_username, mqtt_password))
    {
      Serial.println("Public EMQX MQTT broker connected");
      display.clear();
      display.drawString(0, 0, "MQTT OK");
       display.display();
      
      return true;
     }
    else
    {
      display.clear();
      Serial.print("failed with state ");
      Serial.println(client.state());
      display.drawString(0, 0, "MQTT KO " + client.state());
       display.display();
      vTaskDelay(pdMS_TO_TICKS(2000));
      return false;
    }
   
  }
}
