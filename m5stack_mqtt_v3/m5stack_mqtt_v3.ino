/*

_____________________________________________________________________
|																                                  	|
|              M5STACK MQTT display by J.J.Delorme 2025             |
|                      				                                     	|
|																                                  	|
_____________________________________________________________________

an ESP32 is used instead of an classic arduino Atmel AVR. The goal is to use the wifi link to transmit to an another ESP32 module the energy to be used to optimize the solar panel energy

PIN description

 IN description

 - 
version 1.0 November 2025 client connected on mqtt HA
version 2.0 November 2025 client connected on mqtt HA adding watt 5mn and watt 10mn
version 3.0 November 2025 client connected on mqtt HA adding watt PV
*/


// init to use the two core of the ESP32; one core for power calculation and one core for wifi

#include <Esp.h>
#include <WiFi.h>
#include "PubSubClient.h" //wifi mqtt
//#include <M5Stack.h>
#include <M5Unified.h>
// initialization wifi

const int channel = 4;  // define channel 4 seems to be the best for wifi....

const char *ssid = "freebox_ZPRLHQ_2GEXT"; // SSID WiFi
const char *password = "Cairojude58";      // mot de passe WiFi

// MQTT Broker
const char *mqtt_broker = "192.168.0.154";
const char *topic = "routeur/Wmqtt";
const char *topic_5mn = "routeur/Wmqtt_5mn";
const char *topic_10mn = "routeur/conso";
const char *topic_PV = "sensor.production_watt_pv";
const char *mqtt_username = "mqtt_adm";
const char *mqtt_password = "surel";
const int mqtt_port = 1883;
int MQTT_wait = 0;
bool received_MQTT = false ;
bool check_mqtt = true ;
WiFiClient espClient;
PubSubClient client(espClient);

String client_id = "routeur";

float Power_wifi =0;  // power to be received by wifi

//_____________________________________________________________________________________________
//
// SETUP
//_____________________________________________________________________________________________

void setup() {                  // Begin setup

 // Initialize the M5Stack object
  M5.begin();
  M5.Lcd.clear(BLACK);
  M5.Lcd.setCursor(0,0);
  M5.Lcd.setTextSize(4);
  M5.Lcd.print("ready ...");
  delay(1000);

// USB init

 Serial.begin(115200);
 


 
     // init wifi

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the Wi-Fi network");

  // client.setCallback(callback);
  Connect_MQTT();



   
}           

//____________________________________________________________________________________________
// End setup
//____________________________________________________________________________________________


                              



void loop()
{

 
for (;;) // A Task shall never return or exit.
  {
    
       
    if (Connect_MQTT()) 
    {
      MQTT_wait = 0; // loop to wait 
      client.loop();
    }

  }  

    
}    

               
    

     // end for loop wifi
    

bool Connect_MQTT()
{
  if (client.connected())
  {
    return true;
  }

  while (!client.connected())
  {
 
    client.disconnect();
    client.unsubscribe (topic);
    client.unsubscribe (topic_5mn);
    client.unsubscribe (topic_10mn);
     client.unsubscribe (topic_PV);

    String l_client_id = client_id;
    l_client_id += String(WiFi.macAddress());

    // init wifi mqtt
    client.setServer(mqtt_broker, mqtt_port);

    client.setCallback(OnMqttReceived);
 
    Serial.printf("The client %s connects to the public MQTT broker ", l_client_id.c_str());
    Serial.println();

    if (client.connect(l_client_id.c_str(), mqtt_username, mqtt_password))
    {
      Serial.println("Public EMQX MQTT broker connected");
      M5.Lcd.clear(BLACK);
      M5.Lcd.setCursor(0,0);
      M5.Lcd.print ("MQTT OK");
      if (client.subscribe (topic)) Serial.println("topic suscribed");;
      
      if (client.subscribe (topic_5mn)) Serial.println("topic_5mn suscribed");;
      
      if (client.subscribe (topic_10mn)) Serial.println("topic_10mn suscribed");;

      if (client.subscribe (topic_PV)) Serial.println("topic_PV suscribed");;
      return true;
    }
    else
    {
      Serial.print("failed with state ");
      Serial.println(client.state());
      M5.Lcd.clear(BLACK);
      M5.Lcd.setCursor(0,0);
      M5.Lcd.print ("MQTT KO");

    }
    delay(2000);
    M5.Lcd.clear(BLACK);
  }
}

void OnMqttReceived(char *r_topic, byte *payload, unsigned int length)
{
    
    Serial.print("Received on ");
    Serial.print(r_topic);
    Serial.print(": ");
if (strcmp(r_topic,"routeur/Wmqtt")==0){
    String content = "";
    for (size_t i = 0; i < length; i++)
    {
        content.concat((char)payload[i]);
    }
    Serial.print(content);
    Serial.println();
    Power_wifi = strtof(content.c_str(), NULL);

    M5.Lcd.setCursor(0,0);
    M5.Lcd.print ("W    ");
    M5.Lcd.print (content);
    }
    
  if (strcmp(r_topic,"routeur/Wmqtt_5mn")==0){
    String content = "";
    for (size_t i = 0; i < length; i++)
    {
        content.concat((char)payload[i]);
    }
    Serial.print(content);
    Serial.println();
    Power_wifi = strtof(content.c_str(), NULL);

    M5.Lcd.setCursor(0,40);
    M5.Lcd.print ("W_5  ");
    M5.Lcd.print (content);
    } 

    if (strcmp(r_topic,"routeur/conso")==0){
    String content = "";
    for (size_t i = 0; i < length; i++)
    {
        content.concat((char)payload[i]);
    }
    Serial.print(content);
    Serial.println();
    Power_wifi = strtof(content.c_str(), NULL);

    M5.Lcd.setCursor(0,80);
    M5.Lcd.print ("W_10 ");
    M5.Lcd.print (content);
    } 

    if (strcmp(r_topic,"sensor.production_watt_pv")==0){
    String content = "";
    for (size_t i = 0; i < length; i++)
    {
       content.concat((char)payload[i]);

    }
   
    Serial.print(content);
    Serial.println();
    Power_wifi = strtof(content.c_str(), NULL);

    M5.Lcd.setCursor(0,120);
    M5.Lcd.print ("W_PV ");
    M5.Lcd.print (content);
    } 
    
}
