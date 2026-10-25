/*

_____________________________________________________________________
|                                                                   |
|              M5STACK MQTT display by J.J.Delorme 2025             |
|                                                                   |
_____________________________________________________________________


M5core used for ESP card

version 1.0 November 2025 client connected on mqtt HA
version 2.0 November 2025 adding watt 5mn and watt 10mn
version 3.0 November 2025 adding watt PV
version 4.0 November 2025 adding time using NTP
version 5.0 November 2025 adding temperature
version 6.0 October 2026 changing to apssytems solar power ESP32c6 ECU
version 7.0 october 2026 changing MQTT adress
version 8.0 october 2026 automatic summer/winter time change (POSIX TZ),
                         (time display refreshed on each MQTT message, as in v7)
version 8.1 october 2026 WiFi/MQTT credentials moved to secret.h

*/


#include <Esp.h>
#include <WiFi.h>
#include "PubSubClient.h" //wifi mqtt
//#include <M5Stack.h>
#include <M5Unified.h>

#include <time.h>
#include "secret.h" // identifiants WiFi / MQTT (ne pas partager)
// initialization wifi

const int channel = 4;  // define channel 4 seems to be the best for wifi....

const char *ssid = WIFI_SSID;         // SSID WiFi (secret.h)
const char *password = WIFI_PASSWORD; // mot de passe WiFi (secret.h)

// NTP + fuseau Europe/Paris avec passage automatique heure d'ete / hiver
// CET-1CEST,M3.5.0,M10.5.0/3 :
//   heure d'ete   : dernier dimanche de mars    a 02h00 -> 03h00
//   heure d'hiver : dernier dimanche d'octobre  a 03h00 -> 02h00
const char* ntpServer  = "pool.ntp.org";
const char* ntpServer2 = "time.google.com";
const char* TZ_PARIS   = "CET-1CEST,M3.5.0,M10.5.0/3";



String hostname= "M5 stack v8"; 


// MQTT Broker
const char *mqtt_broker = MQTT_BROKER;
const char *topic = "routeur/Wmqtt";
const char *topic_5mn = "routeur/Wmqtt_5mn";
const char *topic_10mn = "routeur/Wmqtt_10mn";
//const char *topic_PV = "sensor.production_watt_pv";
const char *topic_PV = "sensor.apsystems_fleet_solar_power";
const char *topic_temp_in = "zigbee2mqtt/0xa4c1380333e7ffff";
const char *topic_temp_out = "zigbee2mqtt/0xa4c1380376a0ffff";
const char *mqtt_username = MQTT_USER;
const char *mqtt_password = MQTT_PASSWORD;
const int mqtt_port = 1883;
int MQTT_wait = 0;
bool received_MQTT = false ;
bool check_mqtt = true ;
WiFiClient espClient;
PubSubClient client(espClient);

String client_id = "routeur";


String battery; //data String
String humidity;
String linkquality;
String temperature;
String voltage;
String temp_in;
String temp_out;
String temp_text;


int ind1; // , locations
int ind2;
int ind3;
int ind4;
int ind5;
int ind6;
int ind7;

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
  WiFi.setHostname(hostname.c_str()); //define hostname
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the Wi-Fi network");

  // NTP + fuseau horaire avec regles ete/hiver automatiques
  configTzTime(TZ_PARIS, ntpServer, ntpServer2);

  // Attente de la synchronisation NTP (10 s max)
  struct tm t;
  if (!getLocalTime(&t, 10000)) {
    Serial.println("NTP non synchronise");
  }

  Connect_MQTT();

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

    delay(10); // evite de saturer le CPU

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
  return false;
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

    M5.Lcd.setCursor(0,40);
    M5.Lcd.print ("W_5  ");
    M5.Lcd.print (content);
    } 

    if (strcmp(r_topic,"routeur/Wmqtt_10mn")==0){
    String content = "";
    for (size_t i = 0; i < length; i++)
    {
        content.concat((char)payload[i]);
    }
    Serial.print(content);
    Serial.println();

    M5.Lcd.setCursor(0,80);
    M5.Lcd.print ("W_10 ");
    M5.Lcd.print (content);
    } 

    if (strcmp(r_topic,"sensor.apsystems_fleet_solar_power")==0){
    String content = "";
    for (size_t i = 0; i < length; i++)
    {
       content.concat((char)payload[i]);

    }
   
    Serial.print(content);
    Serial.println();

    M5.Lcd.setCursor(0,120);
    M5.Lcd.print ("W_PV ");
    M5.Lcd.print (content);
    M5.Lcd.print ("       ");
    
    } 

if (strcmp(r_topic,"zigbee2mqtt/0xa4c1380333e7ffff")==0){
    String content = "";
    for (size_t i = 0; i < length; i++)
    {
       content.concat((char)payload[i]);

    }
      ind1 = content.indexOf(',');  //finds location of first ,
      battery = content.substring(0, ind1);   //captures first data String
      ind2 = content.indexOf(',', ind1+1 );   //finds location of second ,
      humidity = content.substring(ind1+1, ind2+1);   //captures second data String
      ind3 = content.indexOf(',', ind2+1 );
      linkquality = content.substring(ind2+1, ind3+1);
      ind4 = content.indexOf(',', ind3+1 );
      temperature = content.substring(ind3+1, ind4+1); 
      ind5 = content.indexOf(',', ind4+1 );
      voltage = content.substring(ind4+1); 
      Serial.print("temperature content: ");
      Serial.print(temperature);
      Serial.println();
      Serial.print("ind1: ");
      Serial.print (ind1);

      
      ind6 = temperature.indexOf(':');  //finds location of first 
      temp_text = temperature.substring(0, ind6);   //captures first data String
      ind7 = temperature.indexOf(',', ind6+1 );   //finds location of second ,
      temp_in = temperature.substring(ind6+1, ind7);   //captures second data String
      Serial.print("temp_in content: ");
      Serial.print(temp_in);
      Serial.println();



    M5.Lcd.setCursor(0,200);
    M5.Lcd.print("      ");
    M5.Lcd.setCursor(0,200);
    M5.Lcd.print ("I ");
    M5.Lcd.print (temp_in);

    } 

if (strcmp(r_topic,"zigbee2mqtt/0xa4c1380376a0ffff")==0){
    String content = "";
    for (size_t i = 0; i < length; i++)
    {
       content.concat((char)payload[i]);

    }
      ind1 = content.indexOf(',');  //finds location of first ,
      battery = content.substring(0, ind1);   //captures first data String
      ind2 = content.indexOf(',', ind1+1 );   //finds location of second ,
      humidity = content.substring(ind1+1, ind2+1);   //captures second data String
      ind3 = content.indexOf(',', ind2+1 );
      linkquality = content.substring(ind2+1, ind3+1);
      ind4 = content.indexOf(',', ind3+1 );
      temperature = content.substring(ind3+1, ind4+1); 
      ind5 = content.indexOf(',', ind4+1 );
      voltage = content.substring(ind4+1); 

      ind6 = temperature.indexOf(':');  //finds location of first 
      temp_text = temperature.substring(0, ind6);   //captures first data String
      ind7 = temperature.indexOf(',', ind6+1 );   //finds location of second ,
      temp_out = temperature.substring(ind6+1, ind7);   //captures second data String
      Serial.print("temp_out content: ");
      Serial.print(temp_out);
      Serial.println();

    M5.Lcd.setCursor(160,200);
    M5.Lcd.print("      ");
    M5.Lcd.setCursor(160,200);
    M5.Lcd.print ("E ");
    M5.Lcd.print (temp_out);


    } 

    /*
    heure via NTP (rafraichie a chaque message MQTT recu)
    */
    printLocalTime();

}


void printLocalTime(){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  Serial.println(timeinfo.tm_isdst > 0 ? "Heure d'ete" : "Heure d'hiver");

  // Efface la zone de l'heure avant d'ecrire (evite les chevauchements)
  M5.Lcd.fillRect(80, 160, 130, 32, BLACK);
  M5.Lcd.setCursor(80,160);
  M5.Lcd.print (&timeinfo, "%H:%M");

}  
