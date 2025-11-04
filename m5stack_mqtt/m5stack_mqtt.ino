/*

_______________________________
|																                                  	|
|              M5STACK MQTT display by J.J.Delorme 2025             |
|                      				                       	|
|																                                  	|
_____________________________________________________________________

an ESP32 is used instead of an classic arduino Atmel AVR. The goal is to use the wifi link to transmit to an another ESP32 module the energy to be used to optimize the solar panel energy

PIN description

 IN description

 - 
version 1.0 November 2025 client connected on mqtt HA

*/


// init to use the two core of the ESP32; one core for power calculation and one core for wifi

#include <Esp.h>
#include <WiFi.h>
#include "PubSubClient.h" //wifi mqtt
#include <M5Stack.h>

// initialization wifi

const int channel = 4;  // define channel 4 seems to be the best for wifi....

const char *ssid = "freebox_ZPRLHQ_2GEXT"; // SSID WiFi
const char *password = "Cairojude58";      // mot de passe WiFi

// MQTT Broker
const char *mqtt_broker = "192.168.0.154";
const char *topic = "routeur/Wmqtt";
const char *topic_5mn = "routeur/Wmqtt_5mn";
const char *topic_10mn = "routeur/conso";
const char *mqtt_username = "mqtt_adm";
const char *mqtt_password = "surel";
const int mqtt_port = 1883;
int MQTT_wait = 0;
bool received_MQTT = false ;
bool check_mqtt = true ;
WiFiClient espClient;
PubSubClient client(espClient);

String client_id = "routeur";


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
  


          Serial.print("P= ");
          Serial.print(rPower/1000);   
          Serial.print("w ");

          
          M5.Lcd.clear(BLACK);
          M5.Lcd.setCursor(0,0);
          M5.Lcd.print ("power");
          M5.Lcd.print ( Power_wifi);

         //Serial.print("mean_power ");
          //Serial.print(mean_power); 

          check_mqtt = true ;
         }  // 
      
        


        
        








 
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
    /*Serial.print(" state MQTT ");
    Serial.println(client.state());
    Serial.print(" state wifi ");
    Serial.println(WiFi.status());
    */
    client.disconnect();
    client.unsubscribe (topic);

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
      display.drawString(0, 22, "MQTT OK");
      if (client.subscribe (topic)) Serial.println("topic suscribed");;
      return true;
    }
    else
    {
      Serial.print("failed with state ");
      Serial.println(client.state());
      display.drawString(0, 22, "MQTT KO " + client.state());
      delay(2000);
    }
    /*
    Serial.print(" state MQTT ");
    Serial.println(client.state());
    Serial.print(" state wifi ");
    Serial.println(WiFi.status());
    */
  }
}

void OnMqttReceived(char *r_topic, byte *payload, unsigned int length)
{
    
    Serial.print("Received on ");
    Serial.print(r_topic);
    Serial.print(": ");

    String content = "";
    for (size_t i = 0; i < length; i++)
    {
        content.concat((char)payload[i]);
    }
    Serial.print(content);
    Serial.println();
    Power_wifi = strtof(content.c_str(), NULL);

}
