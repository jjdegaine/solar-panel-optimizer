/*
________________________________________________________________
|                                                               |
|       auteur : Philippe de Craene <dcphilippe@yahoo.fr        |
|           pour l' Association P'TITWATT                       |
_________________________________________________________________

Toute contribution en vue de l'amélioration de l'appareil est la bienvenue ! Il vous est juste
demandé de conserver mon nom et mon email dans l'entête du programme, et bien sûr de partager
avec moi cette amélioration. Merci.

merci à Ryan McLaughlin <ryanjmclaughlin@gmail.com> pour avoir étudié et mis au point la partie
commande du SCR il y a quelques années et que j'ai repris dans ce programme :)
source : https://web.archive.org/web/20091212193047/http://www.arduino.cc:80/cgi-bin/yabb2/YaBB.pl?num=1230333861/15

_____________________________________________________________________
|                                                                    |
|              modification by J.J 2025/2026                        |
|                                                                    |
_____________________________________________________________________

an ESP32 is used instead of a classic arduino Atmel AVR. The goal is to use the wifi link
to transmit to another ESP32 module the energy to be used to optimize the solar panel energy.

ESP32 DEV MODULE

PIN description
 - PIN 5  static relay command (SCR/SSR gate)
 - PIN 14 input WINTER
 - PIN 15 relay command 2
 - PIN 16 LED power static relay
 - PIN 17 relay command 1
 - PIN 18 overflow LED
 - PIN 19 zero cross voltage (interrupt)
 - PIN 21 SDA for OLED SSD1306
 - PIN 22 SCL for OLED SSD1306
 - PIN 26 input VERBOSE
 - PIN 27 input CALIBRATION
 - PIN 34 analog for voltage measurement
 - PIN 35 analog for intensity measurement

version 2.0  first release version
version 2.1  phasecalibration = -10 to compensate U/I phase shift + adding watchdog on task UI
version 2.3  SSR LED improvement
version 2.4  unsignedlong for all timer with millis
version 3.0  2025_07 data is sent to mqtt instead of UDP
version 3.1  2025_07 relay2 is used for overload (P > 6000W)
version 3.2  2026-02 test improvement timeout mqtt
version 3.3  2026_03_05 adding OTA + daily reset at 00:00
version 4.0  2026_03_05 migration to ESP32 Arduino Core 3.x
version 4.1  2026_03_09 corrections : //                            not to used not working
              - mutex FreeRTOS sur variables partagées inter-tâches
              - protection division par zéro sur mean_power_counter
              - timeout WiFi dans Connect_MQTT (évite boucle infinie)
              - correction parenthésage timers long()
              - sélection carte par #define BOARD
              - nettoyage code commenté inutile
              - timer 78µs pour 50Hz exact (128 ticks * 78µs = 9.984ms)

// https://randomnerdtutorials.com/esp32-ota-elegantota-arduino/
*/

// ============================================================
//  SÉLECTION DE LA CARTE — modifier uniquement cette ligne
// ============================================================
#define BOARD 4   // 1, 3, 4, 5 ou 6

// ============================================================
//  Paramètres de calibration par carte
// ============================================================
#if BOARD == 1
  float Vcalibration    = 0.90;
  float Icalibration    = 93;
  float phasecalibration= -10;
  float ADC_V_0V        = 467;
  float ADC_I_0A        = 467;
  int   tresholdP       = 50000;

#elif BOARD == 3
  float Vcalibration    = 0.955;
  float Icalibration    = 85;
  float phasecalibration= -6;
  float ADC_V_0V        = 462;
  float ADC_I_0A        = 462;
  int   tresholdP       = 50000;

#elif BOARD == 4   // carte TGBT
  float Vcalibration    = 0.975;
  float Icalibration    = 100;
  float phasecalibration= -6;
  float ADC_V_0V        = 446;
  float ADC_I_0A        = 454;
  int   tresholdP       = -100000;

#elif BOARD == 5   // wrover module
  float Vcalibration    = 0.95;
  float Icalibration    = 95;
  float phasecalibration= -6;
  float ADC_V_0V        = 470;
  float ADC_I_0A        = 471;
  int   tresholdP       = 50000;

#elif BOARD == 6
  float Vcalibration    = 0.91;
  float Icalibration    = 90;
  float phasecalibration= -6;
  float ADC_V_0V        = 480;
  float ADC_I_0A        = 481;
  int   tresholdP       = 50000;

#else
  #error "BOARD non défini ou invalide. Choisir 1, 3, 4, 5 ou 6."
#endif

// ============================================================
//  Includes
// ============================================================
#if CONFIG_FREERTOS_UNICORE
  #define ARDUINO_RUNNING_CORE 0
#else
  #define ARDUINO_RUNNING_CORE 1
#endif

#include <Esp.h>
#include <WiFi.h>
#include "PubSubClient.h"
#include <time.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include "SSD1306.h"

// ============================================================
//  Identifiants WiFi et MQTT  (à déplacer dans secrets.h)
// ============================================================
const char *ssid          = "freebox_ZPRLHQ_2GEXT";
const char *password      = "Cairojude58";
const char *hostname      = "ESP32 routeur core3 V4.2";

const char *mqtt_broker   = "192.168.0.154";
const char *mqtt_username = "mqtt_adm";
const char *mqtt_password = "surel";
const int   mqtt_port     = 1883;

// Topics MQTT
const char *topic         = "routeur/Wmqtt";
const char *topic_5mn     = "routeur/Wmqtt_5mn";
const char *topic_10mn    = "routeur/conso";
const char *topic_dim     = "routeur/dim";

// ============================================================
//  NTP / reset quotidien
// ============================================================
const char* ntpServer          = "pool.ntp.org";
const long  gmtOffset_sec      = 3600;   // France hiver
const int   daylightOffset_sec = 3600;   // heure d'été

int           lastDay      = -1;
unsigned long timeout_24H  = 86400000UL; // 24h en ms (fallback si NTP HS)
unsigned long time_24H     = 0;

// ============================================================
//  OTA
// ============================================================
AsyncWebServer server(80);

// ============================================================
//  OLED SSD1306
// ============================================================
SSD1306Wire display(0x3c, SDA, SCL);

// ============================================================
//  Handles de tâches FreeRTOS
// ============================================================
TaskHandle_t taskUIcHandle     = NULL;
TaskHandle_t taskwifi_udpHandle= NULL;

// ============================================================
//  MUTEX FreeRTOS — protège les variables partagées inter-tâches
// ============================================================
SemaphoreHandle_t xMutex = NULL;

// ============================================================
//  Objets WiFi / MQTT
// ============================================================
WiFiClient   espClient;
PubSubClient client(espClient);
String       client_id = "routeur";

// Flags d'envoi MQTT (écrits par TaskUI, lus par Taskwifi)
volatile bool send_MQTT      = false;
volatile bool send_MQTT_5mn  = false;
volatile bool send_MQTT_10mn = false;

// ============================================================
//  Modes de fonctionnement
// ============================================================
bool CALIBRATION = false;  // affiche V/I bruts pour calibrer
bool VERBOSE     = false;  // affiche dim/dimstep
bool WINTER      = false;  // hiver → pas de wifi

// ============================================================
//  Paramètres de mesure et régulation
// ============================================================
byte totalCount = 20;  // nombre de demi-périodes pour 1 mesure

unsigned long unballasting_timeout = 10000;  // 10 s entre deux commandes relais
unsigned long unballasting_time    = 0;
unsigned long MQTT_timeout         = 1800000UL; // 30 min sans MQTT → restart
unsigned long MQTT_time            = 0;

byte unballasting_counter  = 0;
byte unballasting_dim_min  = 5;   // dim min → relais ON
byte unballasting_dim_max  = 64;  // dim max → relais OFF

unsigned int reaction_coeff = 90; // coefficient de réaction de la boucle

// ============================================================
//  Pins ESP32
// ============================================================
const byte SCR_pin          = 5;
const byte pin_winter       = 14;
const byte unballast_relay2 = 15;
const byte unballast_relay1 = 17;
const byte SCRLED           = 16;
const byte limiteLED        = 18;
const byte SDA_PIN          = 21;
const byte CLK_PIN          = 22;
const byte pin_verbose      = 26;
const byte pin_calibration  = 27;
const byte voltageSensorPin = 34;
const byte currentSensorPin = 35;
#define PIN_INTERRUPT 19

// ============================================================
//  Table de correction sinus pour le SCR (dim → phase)
// ============================================================
byte dimthreshold = 30;
byte dimmax       = 128;
byte dim          = 128;  // 0 = pleine puissance, 128 = arrêt

byte dim_sinus[129] = {
  0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,2,2,2,3,3,4,4,5,5,6,7,7,8,9,10,
  11,12,13,14,15,16,18,19,20,21,23,24,25,27,28,31,32,34,35,37,39,41,
  43,44,47,49,50,53,54,57,58,60,63,64,65,68,70,71,74,77,78,79,82,84,
  86,87,89,91,93,94,96,99,100,101,103,104,106,107,108,110,111,112,113,
  114,115,116,117,118,119,120,121,122,122,123,124,124,124,125,125,126,
  126,127,127,127,127,127,127,128,128,128,128,128,128,128,128,128,128,128
};

byte dimphase    = 128 + 30;  // = dim + dimthreshold à l'init
byte dimphasemax = 128 + 30;  // = dimmax + dimthreshold

byte wifi_wait = 0;

// ============================================================
//  Variables ISR (accès depuis interruptions)
// ============================================================
volatile int  i_counter            = 0;
volatile int  i                    = 0;
volatile int  I_led                = 0;
volatile bool zero_cross           = false;
volatile bool zero_cross_flag      = false;
volatile bool led_zero             = false;
volatile uint32_t lastZeroTime     = 0;

// ============================================================
//  Mesure V / I / P  (Core 0 — TaskUI uniquement)
// ============================================================
int   readV = 0, memo_readV = 0, readI = 0;
float rPower = 0, V = 0, I = 0;
float sqV, sumV = 0, sqI, sumI = 0, instP, sumP = 0;
float Power_wifi     = 0;
float Power_wifi_5mn = 0;
byte  zero_crossCount = 0;

// Puissance moyenne 20 s → MQTT
float         mean_power         = 0;
float         mean_power_MQTT    = 0;
int           mean_power_counter = 0;
unsigned long mean_power_time    = 0;
unsigned long mean_power_timing  = 20000;  // 20 s
char          mystring_power_wifi[20];

// Puissance moyenne 5 min → MQTT
float         mean_power_5mn         = 0;
float         mean_power_MQTT_5mn    = 0;
int           mean_power_counter_5mn = 0;
unsigned long mean_power_time_5mn    = 0;
unsigned long mean_power_timing_5mn  = 300000;  // 5 min
char          mystring_power_wifi_5mn[20];

// Puissance moyenne 10 min → MQTT
float         mean_power_10mn         = 0;
float         mean_power_MQTT_10mn    = 0;
int           mean_power_counter_10mn = 0;
unsigned long mean_power_time_10mn    = 0;
unsigned long mean_power_timing_10mn  = 600000;  // 10 min
char          mystring_power_wifi_10mn[20];

// DIM test
float dim_test = 0;
char  mystring_dim[20];

// Puissance max réseau et chauffe-eau
float PowerMax          = 6000.0;
float Power_water_heater= 3000.0;

int          dimstep    = 0;
unsigned int memo_temps = 0;
bool relay_1 = false;
bool relay_2 = false;

// ============================================================
//  Timer hardware (Core 3 API)
// ============================================================
hw_timer_t   *timer    = NULL;
portMUX_TYPE  timerMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE  mux      = portMUX_INITIALIZER_UNLOCKED;

// ============================================================
//  Prototypes de tâches
// ============================================================
void TaskUI(void *pvParameters);
void Taskwifi_udp(void *pvParameters);
bool Connect_MQTT();

//____________________________________________________________________________________________
//
// ISR — ZERO CROSS (PIN 19, front montant)
// Filtre les faux déclenchements < 5 ms (front descendant parasite)
//____________________________________________________________________________________________
void IRAM_ATTR isrPin19() {
  portENTER_CRITICAL_ISR(&mux);
  uint32_t now = micros();
  if (now - lastZeroTime > 5000) {  // ignore si < 5 ms depuis dernier passage
    zero_cross_flag = true;
    zero_cross      = true;
    led_zero        = true;
    lastZeroTime    = now;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

//____________________________________________________________________________________________
//
// ISR — TIMER  (78 µs * 128 ticks ≈ 9.984 ms = demi-période 50 Hz)
//____________________________________________________________________________________________
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);

  digitalWrite(limiteLED, HIGH);

  // Gestion LED zéro
  if (led_zero) {
    I_led   = 0;
    led_zero= false;
  } else {
    I_led++;
  }

  // Commande SCR
  if (zero_cross && dimphase < dimphasemax) {
    if (i_counter > dimphase) {
      digitalWrite(SCR_pin, HIGH);
      delayMicroseconds(5);
      digitalWrite(SCR_pin, LOW);
      i_counter  = 0;
      zero_cross = false;
    } else {
      i_counter++;
    }
  }

  digitalWrite(limiteLED, LOW);
  portEXIT_CRITICAL_ISR(&timerMux);
}

//____________________________________________________________________________________________
//
// SETUP
//____________________________________________________________________________________________
void setup() {

  // --- Sorties / Entrées ---
  pinMode(SCR_pin,         OUTPUT);
  pinMode(unballast_relay1,OUTPUT);
  pinMode(unballast_relay2,OUTPUT);
  pinMode(SCRLED,          OUTPUT);
  pinMode(limiteLED,       OUTPUT);
  pinMode(PIN_INTERRUPT,   INPUT);
  pinMode(pin_winter,      INPUT);
  pinMode(pin_verbose,     INPUT);
  pinMode(pin_calibration, INPUT);

  // --- Init états relais ---
  digitalWrite(unballast_relay1, LOW);
  digitalWrite(unballast_relay2, LOW);

  // --- Init timers logiciels ---
  unsigned long now    = millis();
  unballasting_time    = now;
  mean_power_time      = now;
  mean_power_time_5mn  = now;
  mean_power_time_10mn = now;
  MQTT_time            = now;
  time_24H             = now;

  // --- Série ---
  Serial.begin(115200);

  // --- Récupération bus I²C (workaround bug démarrage) ---
  // https://github.com/esp8266/Arduino/issues/1025
  delay(2000);
  pinMode(SDA_PIN, OUTPUT);
  digitalWrite(SDA_PIN, HIGH);
  pinMode(CLK_PIN, OUTPUT);
  for (int k = 0; k < 10; k++) {
    digitalWrite(CLK_PIN, HIGH); delayMicroseconds(5);
    digitalWrite(CLK_PIN, LOW);  delayMicroseconds(5);
  }
  // Signal STOP
  digitalWrite(SDA_PIN, LOW);  delayMicroseconds(5);
  digitalWrite(CLK_PIN, HIGH); delayMicroseconds(2);
  digitalWrite(SDA_PIN, HIGH); delayMicroseconds(2);
  pinMode(SDA_PIN, INPUT);
  pinMode(CLK_PIN, INPUT);
  delay(2000);

  // --- I²C + OLED ---
  Wire.begin(SDA_PIN, CLK_PIN);
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_24);
  display.drawString(0, 0, "Ready");
  display.display();

  Serial.println("Ready ...");
  delay(500);

  if (VERBOSE) {
    Serial.println("  Pu (W) || dimstep |  dim ||");
  } else {
    Serial.println("GO");
  }
  display.clear();

  // --- Mutex FreeRTOS ---
  xMutex = xSemaphoreCreateMutex();
  if (xMutex == NULL) {
    Serial.println("ERREUR : création mutex échouée !");
    ESP.restart();
  }

  // --- WiFi ---
  WiFi.setHostname(hostname);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - wifiStart > 30000) {
      Serial.println("WiFi timeout au démarrage, restart...");
      ESP.restart();
    }
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to WiFi");
  Serial.print("IP: "); Serial.println(WiFi.localIP());

  // --- OTA ---
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "ESP routeur OTA v4.1");
  });
  server.begin();
  ElegantOTA.begin(&server);
  Serial.println("HTTP OTA server started");

  // --- NTP ---
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // --- MQTT initial ---
  Connect_MQTT();

  // --- Timer hardware Core 3 ---
  // 78 µs × 128 ticks = 9984 µs ≈ demi-période 50 Hz
  timer = timerBegin(1000000);           // 1 MHz → 1 tick = 1 µs
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 78, true, 0);        // 78 µs, auto-reload
  timerStart(timer);

  // --- Interruption zero-cross ---
  attachInterrupt(digitalPinToInterrupt(PIN_INTERRUPT), isrPin19, RISING);

  // --- Tâches FreeRTOS ---
  xTaskCreatePinnedToCore(TaskUI,        "TaskUI",    20000, NULL, 2, &taskUIcHandle,      0); // Core 0
  xTaskCreatePinnedToCore(Taskwifi_udp,  "TaskWifi",  20000, NULL, 1, &taskwifi_udpHandle, 1); // Core 1
}

void loop() {
  // Vide : tout est géré par les tâches FreeRTOS
}

/*--------------------------------------------------*/
/*------------------ Tâche UI (Core 0) -------------*/
/*--------------------------------------------------*/
void TaskUI(void *pvParameters) {
  (void)pvParameters;

  for (;;) {

    unsigned int numberOfSamples = 0;
    sumV = 0; sumI = 0; sumP = 0;
    unsigned int time_now_second = millis() / 1000;

    // Remise à zéro du compteur demi-périodes
    if (zero_crossCount >= totalCount) {
      zero_crossCount = 0;
    }

    // Acquisition sur totalCount demi-périodes
    while (zero_crossCount < totalCount) {

      if (zero_cross_flag) {
        zero_cross_flag = false;
        zero_crossCount++;
      }

      numberOfSamples++;

      memo_readV = readV;
      readV = analogRead(voltageSensorPin) / 4;  // 12 bits → 10 bits (max 1023)

      // Détection absence de réseau
      if (memo_readV == 0 && readV == 0) {
        break;
      }

      readI = analogRead(currentSensorPin) / 4;

      // Calcul RMS (mode CALIBRATION uniquement)
      if (CALIBRATION) {
        sqV   = (readV - ADC_V_0V) * (readV - ADC_V_0V);
        sumV += sqV;
        sqI   = (readI - ADC_I_0A) * (readI - ADC_I_0A);
        sumI += sqI;
      }

      // Puissance instantanée avec correction de phase
      instP = ((memo_readV - ADC_V_0V) + phasecalibration * ((readV - ADC_V_0V) - (memo_readV - ADC_V_0V)))
              * (readI - ADC_I_0A);
      sumP += instP;

      // Calcul de la puissance
      if (numberOfSamples > 0) {
        if (CALIBRATION) {
          V = Vcalibration * sqrt(sumV / numberOfSamples);
          I = Icalibration * sqrt(sumI / numberOfSamples);
        }
        rPower     = Vcalibration * Icalibration * sumP / numberOfSamples;
        Power_wifi = rPower / 1000.0f;
      }

      //____________________________
      // Régulation DIM
      //____________________________
      if (rPower > 0) {
        dimstep = (int)(rPower / 1000.0f / reaction_coeff) + 1;
      } else {
        dimstep = (int)(1 - rPower / 1000.0f / reaction_coeff);
      }

      if (rPower < tresholdP) {
        dim = (dim > dimstep) ? dim - dimstep : 0;
      } else if (rPower > tresholdP) {
        dim = (dim + dimstep < dimmax) ? dim + dimstep : dimmax;
      }

      // Mise à jour dimphase pour l'ISR timer
      dimphase = dim_sinus[dim] + dimthreshold;

      //____________________________
      // Commande relais (avec timeout anti-rebond)
      //____________________________
      if ((millis() - unballasting_time) > unballasting_timeout) {

        if (dim < unballasting_dim_min) {
          if (unballasting_counter > 10) {
            digitalWrite(unballast_relay1, HIGH);
            relay_1 = true;
            unballasting_counter = 0;
            unballasting_time    = millis();
          }
          unballasting_counter++;
        }

        if (dim > unballasting_dim_max) {
          if (unballasting_counter > 10) {
            digitalWrite(unballast_relay1, LOW);
            relay_1 = false;
            unballasting_time    = millis();
            unballasting_counter = 0;
          }
          unballasting_counter++;
        }

        // Surcharge réseau → relais 2
        if (Power_wifi > PowerMax) {
          digitalWrite(unballast_relay2, HIGH);
          relay_2 = true;
        }
        if (relay_2 && Power_wifi < (PowerMax - Power_water_heater)) {
          digitalWrite(unballast_relay2, LOW);
          relay_2 = false;
        }
      }

      //____________________________
      // Moyennes pour MQTT (sous mutex)
      //____________________________

      // 20 s
      if ((millis() - mean_power_time) > mean_power_timing) {
        if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          mean_power_MQTT    = (mean_power_counter > 0) ? mean_power / mean_power_counter : 0;
          mean_power         = 0;
          mean_power_counter = 0;
          send_MQTT          = true;
          xSemaphoreGive(xMutex);
        }
        mean_power_time = millis();
      } else {
        mean_power += Power_wifi;
        mean_power_counter++;
      }

      // 5 min
      if ((millis() - mean_power_time_5mn) > mean_power_timing_5mn) {
        if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          mean_power_MQTT_5mn    = (mean_power_counter_5mn > 0) ? mean_power_5mn / mean_power_counter_5mn : 0;
          mean_power_5mn         = 0;
          mean_power_counter_5mn = 0;
          send_MQTT_5mn          = true;
          xSemaphoreGive(xMutex);
        }
        mean_power_time_5mn = millis();
      } else {
        mean_power_5mn += Power_wifi;
        mean_power_counter_5mn++;
      }

      // 10 min
      if ((millis() - mean_power_time_10mn) > mean_power_timing_10mn) {
        if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          mean_power_MQTT_10mn    = (mean_power_counter_10mn > 0) ? mean_power_10mn / mean_power_counter_10mn : 0;
          mean_power_10mn         = 0;
          mean_power_counter_10mn = 0;
          send_MQTT_10mn          = true;
          xSemaphoreGive(xMutex);
        }
        mean_power_time_10mn = millis();
        Serial.print("mean_power_10mn (W): ");
        Serial.println(mean_power_MQTT_10mn);
      } else {
        mean_power_10mn += Power_wifi;
        mean_power_counter_10mn++;
      }

      //____________________________
      // Affichage toutes les 2 s
      //____________________________
      if (time_now_second >= memo_temps + 2) {

        if (!CALIBRATION && !VERBOSE) {
          memo_temps = time_now_second;
          display.setColor(BLACK);
          display.fillRect(0, 0, 128, 22);
          display.setColor(WHITE);
          display.drawString(0, 0, String(int(Power_wifi)) + " || " + String(dim));
          display.display();
        }

        if (CALIBRATION) {
          Serial.print(V);   Serial.print("  |  ");
          Serial.print(I / 1000); Serial.print("  |  ");
          Serial.println(rPower / 1000);
          display.clear();
          display.drawString(0, 0,  String(int(V)) + "||" + String(int(I / 1000)));
          display.drawString(0, 22, String(int(Power_wifi)));
          display.display();
        }

        if (VERBOSE) {
          Serial.print(rPower / 1000); Serial.print("  ||  ");
          Serial.print(dimstep);       Serial.print("  ||  ");
          Serial.print(dim);           Serial.print("  ||  ");
          Serial.print(dimphase);      Serial.print("  ||  ");
          Serial.print(relay_1);       Serial.print("  ||  ");
          Serial.print(relay_2);       Serial.print("  ||  ");
          Serial.print(unballasting_counter); Serial.print("  ||  ");
          Serial.println(millis() - unballasting_time);
        } else {
          delay(1);  // stabilité boucle
        }
      }

    }  // end while zero_crossCount

  }  // end for(;;)
}  // end TaskUI

/*--------------------------------------------------*/
/*--------------- Tâche WiFi/MQTT (Core 1) ----------*/
/*--------------------------------------------------*/
void Taskwifi_udp(void *pvParameters) {
  (void)pvParameters;

  for (;;) {

    // Attente du signal d'envoi MQTT (avec timeout watchdog)
    while (!send_MQTT) {
      wifi_wait = 0;
      if ((millis() - MQTT_time) > MQTT_timeout) {
        Serial.println("Timeout MQTT dépassé → restart");
        // Envoi dim=1 pour signaler la déconnexion
        dim_test = 1;
        sprintf(mystring_dim, "%g", dim_test);
        if (Connect_MQTT()) {
          client.publish(topic_dim, mystring_dim, true);
        }
        delay(5000);
        ESP.restart();
      }
      delay(10);  // cède le CPU
    }

    MQTT_time = millis();

    if (Connect_MQTT()) {
      // Lecture des moyennes sous mutex
      float local_20s = 0, local_5mn = 0, local_10mn = 0;
      bool  do_20s = false, do_5mn = false, do_10mn = false;

      if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (send_MQTT) {
          local_20s = mean_power_MQTT;
          do_20s    = true;
          send_MQTT = false;
        }
        if (send_MQTT_5mn) {
          local_5mn    = mean_power_MQTT_5mn;
          do_5mn       = true;
          send_MQTT_5mn= false;
        }
        if (send_MQTT_10mn) {
          local_10mn    = mean_power_MQTT_10mn;
          do_10mn       = true;
          send_MQTT_10mn= false;
        }
        xSemaphoreGive(xMutex);
      }

      // Publication MQTT hors mutex
      if (do_20s) {
        sprintf(mystring_power_wifi, "%g", local_20s);
        client.publish(topic, mystring_power_wifi, true);
      }
      if (do_5mn) {
        sprintf(mystring_power_wifi_5mn, "%g", local_5mn);
        client.publish(topic_5mn, mystring_power_wifi_5mn, true);
      }
      if (do_10mn) {
        sprintf(mystring_power_wifi_10mn, "%g", local_10mn);
        client.publish(topic_10mn, mystring_power_wifi_10mn, true);
      }
    }

    // Reset quotidien à minuit (NTP)
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
      Serial.println("NTP : heure non disponible");
      if ((millis() - time_24H) > timeout_24H) {
        Serial.println("Timeout 24H sans NTP → restart");
        delay(5000);
        ESP.restart();
      }
    } else {
      time_24H = millis();  // NTP ok → réarme le timeout
      if (timeinfo.tm_hour == 0 && timeinfo.tm_min == 0 && timeinfo.tm_mday != lastDay) {
        Serial.println("Reset quotidien à minuit...");
        lastDay = timeinfo.tm_mday;
        delay(60000);  // 1 min pour éviter double reset
        ESP.restart();
      }
    }

    // OTA
    ElegantOTA.loop();

  }  // end for(;;)
}  // end Taskwifi_udp

/*--------------------------------------------------*/
/*------------- Connexion WiFi + MQTT --------------*/
/*--------------------------------------------------*/
bool Connect_MQTT() {

  if (client.connected()) return true;

  // Déconnexion propre
  client.disconnect();
  delay(1000);
  WiFi.disconnect();
  delay(1000);

  Serial.println("Reconnexion WiFi...");
  WiFi.setHostname(hostname);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Attente WiFi avec timeout 30 s
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - wifiStart > 30000) {
      Serial.println("WiFi timeout dans Connect_MQTT → restart");
      ESP.restart();
    }
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.print("WiFi OK — IP: "); Serial.println(WiFi.localIP());

  // Reconnexion MQTT
  client.setServer(mqtt_broker, mqtt_port);
  String l_client_id = client_id + String(WiFi.macAddress());
  Serial.printf("MQTT connect as %s ...\n", l_client_id.c_str());

  if (client.connect(l_client_id.c_str(), mqtt_username, mqtt_password)) {
    Serial.println("MQTT connecté");
    display.drawString(0, 22, "MQTT OK");
    return true;
  } else {
    Serial.print("MQTT échec, état: "); Serial.println(client.state());
    display.drawString(0, 22, "MQTT KO");
    return false;
  }
}
