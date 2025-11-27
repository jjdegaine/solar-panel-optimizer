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
version 2.0 November 2025 adding watt 5mn and watt 10mn
version 3.0 November 2025 adding watt PV
version 4.0 November 2025 adding time using NTP
version 5.0 November 2025 adding temperature
*/


// init to use the two core of the ESP32; one core for power calculation and one core for wifi

#include <Esp.h>
#include <WiFi.h>
#include "PubSubClient.h" //wifi mqtt
//#include <M5Stack.h>
#include <M5Unified.h>

#include <time.h>
// initialization wifi

const int channel = 4;  // define channel 4 seems to be the best for wifi....

const char *ssid = "freebox_ZPRLHQ_2GEXT"; // SSID WiFi
const char *password = "Cairojude58";      // mot de passe WiFi
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;






// MQTT Broker
const char *mqtt_broker = "192.168.0.154";
const char *topic = "routeur/Wmqtt";
const char *topic_5mn = "routeur/Wmqtt_5mn";
const char *topic_10mn = "routeur/conso";
const char *topic_PV = "sensor.production_watt_pv";
const char *topic_temp_in = "sensor.0xa4c1380333e7ffff_temperature";
const char *topic_temp_out = "sensor.0xa4c1380376a0ffff_temperature";
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

 // Init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();


   
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
    client.unsubscribe (topic_temp_in);
    client.unsubscribe (topic_temp_out);

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

      if (client.subscribe (topic_temp_in)) Serial.println("topic_temp_in suscribed");;

      if (client.subscribe (topic_temp_out)) Serial.println("topic_temp_out suscribed");;
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

if (strcmp(r_topic,"sensor.0xa4c1380333e7ffff_temperature")==0){
    String content = "";
    for (size_t i = 0; i < length; i++)
    {
       content.concat((char)payload[i]);

    }
   
    Serial.print(content);
    Serial.println();
    Power_wifi = strtof(content.c_str(), NULL);

    M5.Lcd.setCursor(0,200);
    M5.Lcd.print ("W_PV ");
    M5.Lcd.print (content);

    } 

if (strcmp(r_topic,"sensor.0xa4c1380376a0ffff_temperature")==0){
    String content = "";
    for (size_t i = 0; i < length; i++)
    {
       content.concat((char)payload[i]);

    }
   
    Serial.print(content);
    Serial.println();
    Power_wifi = strtof(content.c_str(), NULL);

    M5.Lcd.setCursor(80,200);
    M5.Lcd.print ("W_PV ");
    M5.Lcd.print (content);
    
    } 

    printLocalTime();

}


void printLocalTime(){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  Serial.print("Day of week: ");
  Serial.println(&timeinfo, "%A");
  Serial.print("Month: ");
  Serial.println(&timeinfo, "%B");
  Serial.print("Day of Month: ");
  Serial.println(&timeinfo, "%d");
  Serial.print("Year: ");
  Serial.println(&timeinfo, "%Y");
  Serial.print("Hour: ");
  Serial.println(&timeinfo, "%H");
  Serial.print("Hour (12 hour format): ");
  Serial.println(&timeinfo, "%I");
  Serial.print("Minute: ");
  Serial.println(&timeinfo, "%M");
  Serial.print("Second: ");
  Serial.println(&timeinfo, "%S");

  Serial.println("Time variables");
  char timeHour[3];
  strftime(timeHour,3, "%H", &timeinfo);
  Serial.println(timeHour);
  char timeWeekDay[10];
  strftime(timeWeekDay,10, "%A", &timeinfo);
  Serial.println(timeWeekDay);
  Serial.println();

      M5.Lcd.setCursor(80,160);
      M5.Lcd.print (&timeinfo, "%H:%M");

}  

