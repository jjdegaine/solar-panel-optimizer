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
version 4.1  2026_03_09 mutex FreeRTOS, div/0 protection, timeout WiFi, timer 78us
version 4.2  2026_03_09 watchdog materiel TWDT sur Core 0 et Core 1     //                            not to used not working
              API Core 3 : esp_task_wdt_config_t + esp_task_wdt_init()
              - WDT_TIMEOUT_MS = 60 s
              - idle_core_mask surveille Core 0 + Core 1
              - trigger_panic = true -> panic + restart automatique
              - esp_task_wdt_add(NULL) dans chaque tache au demarrage
              - esp_task_wdt_reset() appele regulierement dans chaque tache
              - esp_task_wdt_reset() appele dans Connect_MQTT() pendant reconnexion

// https://randomnerdtutorials.com/esp32-ota-elegantota-arduino/
*/

// ============================================================
//  SELECTION DE LA CARTE - modifier uniquement cette ligne
// ============================================================
#define BOARD 4   // 1, 3, 4, 5 ou 6

// ============================================================
//  Parametres de calibration par carte
// ============================================================
#if BOARD == 1
  float Vcalibration     = 0.90;
  float Icalibration     = 93;
  float phasecalibration = -10;
  float ADC_V_0V         = 467;
  float ADC_I_0A         = 467;
  int   tresholdP        = 50000;
#elif BOARD == 3
  float Vcalibration     = 0.955;
  float Icalibration     = 85;
  float phasecalibration = -6;
  float ADC_V_0V         = 462;
  float ADC_I_0A         = 462;
  int   tresholdP        = 50000;
#elif BOARD == 4   // carte TGBT
  float Vcalibration     = 0.975;
  float Icalibration     = 100;
  float phasecalibration = -6;
  float ADC_V_0V         = 446;
  float ADC_I_0A         = 454;
  int   tresholdP        = -100000;
#elif BOARD == 5   // wrover module
  float Vcalibration     = 0.95;
  float Icalibration     = 95;
  float phasecalibration = -6;
  float ADC_V_0V         = 470;
  float ADC_I_0A         = 471;
  int   tresholdP        = 50000;
#elif BOARD == 6
  float Vcalibration     = 0.91;
  float Icalibration     = 90;
  float phasecalibration = -6;
  float ADC_V_0V         = 480;
  float ADC_I_0A         = 481;
  int   tresholdP        = 50000;
#else
  #error "BOARD non defini ou invalide. Choisir 1, 3, 4, 5 ou 6."
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
//  WATCHDOG TWDT - API Core 3 / ESP-IDF 5.x
//
//  DIFFERENCE AVEC CORE 2 :
//    Core 2 : esp_task_wdt_init(timeout_seconds, panic_bool)
//    Core 3 : esp_task_wdt_init(const esp_task_wdt_config_t*)
//
//  La structure esp_task_wdt_config_t contient :
//    .timeout_ms     : timeout en millisecondes (et non en secondes)
//    .idle_core_mask : bitmask des cores a surveiller (taches IDLE)
//    .trigger_panic  : true = panic + restart si timeout
//
//  Utilisation dans les taches :
//    esp_task_wdt_add(NULL)   : abonne la tache courante au WDT
//    esp_task_wdt_reset()     : nourrit le WDT (a appeler regulierement)
//    esp_task_wdt_delete(NULL): desabonne la tache courante
// ============================================================
#include <esp_task_wdt.h>

// Timeout 60 secondes :
// - TaskUI  : cycle de 20 demi-periodes ~200 ms -> tres en dessous de 60 s
// - TaskWifi: attente MQTT + reconnexion WiFi max 30 s -> en dessous de 60 s
// #define WDT_TIMEOUT_MS 60000

// ============================================================
//  Identifiants WiFi et MQTT
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
const long  gmtOffset_sec      = 3600;
const int   daylightOffset_sec = 3600;

int           lastDay     = -1;
unsigned long timeout_24H = 86400000UL;
unsigned long time_24H    = 0;

// ============================================================
//  OTA / OLED
// ============================================================
AsyncWebServer server(80);
SSD1306Wire    display(0x3c, SDA, SCL);

// ============================================================
//  Handles FreeRTOS
// ============================================================
TaskHandle_t taskUIcHandle      = NULL;
TaskHandle_t taskwifi_udpHandle = NULL;

// ============================================================
//  Mutex FreeRTOS - protege les variables partagees inter-taches
// ============================================================
SemaphoreHandle_t xMutex = NULL;

// ============================================================
//  WiFi / MQTT
// ============================================================
WiFiClient   espClient;
PubSubClient client(espClient);
String       client_id = "routeur";

volatile bool send_MQTT      = false;
volatile bool send_MQTT_5mn  = false;
volatile bool send_MQTT_10mn = false;

// ============================================================
//  Modes de fonctionnement
// ============================================================
bool CALIBRATION = false;
bool VERBOSE     = false;
bool WINTER      = false;

// ============================================================
//  Parametres regulation / mesure
// ============================================================
byte totalCount = 20;

unsigned long unballasting_timeout = 10000;
unsigned long unballasting_time    = 0;
unsigned long MQTT_timeout         = 1800000UL;
unsigned long MQTT_time            = 0;

byte         unballasting_counter = 0;
byte         unballasting_dim_min = 5;
byte         unballasting_dim_max = 64;
unsigned int reaction_coeff       = 90;

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
//  Table sinus SCR (dim -> phase)
// ============================================================
byte dimthreshold = 30;
byte dimmax       = 128;
byte dim          = 128;

byte dim_sinus[129] = {
  0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,2,2,2,3,3,4,4,5,5,6,7,7,8,9,10,
  11,12,13,14,15,16,18,19,20,21,23,24,25,27,28,31,32,34,35,37,39,41,
  43,44,47,49,50,53,54,57,58,60,63,64,65,68,70,71,74,77,78,79,82,84,
  86,87,89,91,93,94,96,99,100,101,103,104,106,107,108,110,111,112,113,
  114,115,116,117,118,119,120,121,122,122,123,124,124,124,125,125,126,
  126,127,127,127,127,127,127,128,128,128,128,128,128,128,128,128,128,128
};

byte dimphase    = 158;  // 128 + 30
byte dimphasemax = 158;

// ============================================================
//  Variables ISR
// ============================================================
volatile int      i_counter       = 0;
volatile int      I_led           = 0;
volatile bool     zero_cross      = false;
volatile bool     zero_cross_flag = false;
volatile bool     led_zero        = false;
volatile uint32_t lastZeroTime    = 0;

// ============================================================
//  Variables de mesure (Core 0 uniquement)
// ============================================================
int   readV = 0, memo_readV = 0, readI = 0;
float rPower = 0, V = 0, I = 0;
float sqV, sumV = 0, sqI, sumI = 0, instP, sumP = 0;
float Power_wifi      = 0;
byte  zero_crossCount = 0;

// Moyenne 20 s
float         mean_power         = 0;
float         mean_power_MQTT    = 0;
int           mean_power_counter = 0;
unsigned long mean_power_time    = 0;
unsigned long mean_power_timing  = 20000;
char          mystring_power_wifi[20];

// Moyenne 5 min
float         mean_power_5mn         = 0;
float         mean_power_MQTT_5mn    = 0;
int           mean_power_counter_5mn = 0;
unsigned long mean_power_time_5mn    = 0;
unsigned long mean_power_timing_5mn  = 300000;
char          mystring_power_wifi_5mn[20];

// Moyenne 10 min
float         mean_power_10mn         = 0;
float         mean_power_MQTT_10mn    = 0;
int           mean_power_counter_10mn = 0;
unsigned long mean_power_time_10mn    = 0;
unsigned long mean_power_timing_10mn  = 600000;
char          mystring_power_wifi_10mn[20];

// DIM de secours
float dim_test = 0;
char  mystring_dim[20];

// Limites puissance
float PowerMax           = 6000.0;
float Power_water_heater = 3000.0;

int          dimstep    = 0;
unsigned int memo_temps = 0;
bool relay_1 = false;
bool relay_2 = false;

// ============================================================
//  Timer hardware (API Core 3)
// ============================================================
hw_timer_t   *timer    = NULL;
portMUX_TYPE  timerMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE  mux      = portMUX_INITIALIZER_UNLOCKED;

// ============================================================
//  Prototypes
// ============================================================
void TaskUI(void *pvParameters);
void Taskwifi_udp(void *pvParameters);
bool Connect_MQTT();

//____________________________________________________________________________________________
// ISR - ZERO CROSS (PIN 19, front montant)
// Filtre < 5 ms pour ignorer le front descendant parasite
//____________________________________________________________________________________________
void IRAM_ATTR isrPin19() {
  portENTER_CRITICAL_ISR(&mux);
  uint32_t now = micros();
  if (now - lastZeroTime > 5000) {
    zero_cross_flag = true;
    zero_cross      = true;
    led_zero        = true;
    lastZeroTime    = now;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

//____________________________________________________________________________________________
// ISR - TIMER (78 us x 128 ticks = 9984 us ≈ demi-periode 50 Hz)
//____________________________________________________________________________________________
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);

  digitalWrite(limiteLED, HIGH);

  if (led_zero) {
    I_led    = 0;
    led_zero = false;
  } else {
    I_led++;
  }

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
// SETUP
//____________________________________________________________________________________________
void setup() {

  pinMode(SCR_pin,         OUTPUT);
  pinMode(unballast_relay1,OUTPUT);
  pinMode(unballast_relay2,OUTPUT);
  pinMode(SCRLED,          OUTPUT);
  pinMode(limiteLED,       OUTPUT);
  pinMode(PIN_INTERRUPT,   INPUT);
  pinMode(pin_winter,      INPUT);
  pinMode(pin_verbose,     INPUT);
  pinMode(pin_calibration, INPUT);

  digitalWrite(unballast_relay1, LOW);
  digitalWrite(unballast_relay2, LOW);

  unsigned long now    = millis();
  unballasting_time    = now;
  mean_power_time      = now;
  mean_power_time_5mn  = now;
  mean_power_time_10mn = now;
  MQTT_time            = now;
  time_24H             = now;

  Serial.begin(115200);

  // Workaround bus I2C au demarrage
  delay(2000);
  pinMode(SDA_PIN, OUTPUT);
  digitalWrite(SDA_PIN, HIGH);
  pinMode(CLK_PIN, OUTPUT);
  for (int k = 0; k < 10; k++) {
    digitalWrite(CLK_PIN, HIGH); delayMicroseconds(5);
    digitalWrite(CLK_PIN, LOW);  delayMicroseconds(5);
  }
  digitalWrite(SDA_PIN, LOW);  delayMicroseconds(5);
  digitalWrite(CLK_PIN, HIGH); delayMicroseconds(2);
  digitalWrite(SDA_PIN, HIGH); delayMicroseconds(2);
  pinMode(SDA_PIN, INPUT);
  pinMode(CLK_PIN, INPUT);
  delay(2000);

  Wire.begin(SDA_PIN, CLK_PIN);
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_24);
  display.drawString(0, 0, "Ready");
  display.display();
  Serial.println("Ready ...");
  delay(500);
  if (VERBOSE) Serial.println("  Pu (W) || dimstep |  dim ||");
  else         Serial.println("GO");
  display.clear();

  // Mutex
  xMutex = xSemaphoreCreateMutex();
  if (xMutex == NULL) { Serial.println("ERREUR mutex!"); ESP.restart(); }

  // ==========================================================
  //  INIT WATCHDOG TWDT - API Core 3
  //
  //  IMPORTANT : esp_task_wdt_deinit() desactive d'abord le WDT
  //  par defaut du systeme, avant de le reconfigurer avec nos
  //  parametres. Sans ce deinit, esp_task_wdt_init() retourne
  //  ESP_ERR_INVALID_STATE.
  //
  //  idle_core_mask = (1 << portNUM_PROCESSORS) - 1
  //    portNUM_PROCESSORS = 2 sur ESP32 dual-core
  //    -> 0b11 = surveille Core 0 ET Core 1 (taches IDLE)
  //
  //  Les taches metier (TaskUI, TaskWifi) sont ajoutees
  //  individuellement via esp_task_wdt_add(NULL) dans chaque tache.
  // ==========================================================
  /* disable wdt
  Serial.println("Init WDT TWDT...");
  esp_task_wdt_deinit();

  esp_task_wdt_config_t wdt_config = {
    .timeout_ms    = WDT_TIMEOUT_MS,
    .idle_core_mask= (1 << portNUM_PROCESSORS) - 1,  // Core 0 + Core 1
    .trigger_panic = true                              // panic -> restart auto
  };

  esp_err_t wdt_err = esp_task_wdt_init(&wdt_config);
  if (wdt_err != ESP_OK) {
    Serial.printf("ERREUR WDT init: %s\n", esp_err_to_name(wdt_err));
  } else {
    Serial.printf("WDT OK: timeout=%ds, Core0+Core1, panic=ON\n", WDT_TIMEOUT_MS / 1000);
  }
*/
  // WiFi
  WiFi.setHostname(hostname);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - wifiStart > 30000) { Serial.println("WiFi timeout, restart"); ESP.restart(); }
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to WiFi");
  Serial.print("IP: "); Serial.println(WiFi.localIP());

  // OTA
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "ESP routeur OTA v4.2");
  });
  server.begin();
  ElegantOTA.begin(&server);
  Serial.println("HTTP OTA server started");

  // NTP
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // MQTT initial
  Connect_MQTT();

  // Timer hardware - 78 us x 128 = 9984 us ≈ demi-periode 50 Hz
  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 78, true, 0);
  timerStart(timer);

  // Interruption zero-cross
  attachInterrupt(digitalPinToInterrupt(PIN_INTERRUPT), isrPin19, RISING);

  // Taches FreeRTOS
  // TaskUI sur Core 0, priorite 2 (haute) - calcul puissance temps reel
  // TaskWifi sur Core 1, priorite 1 (normale) - WiFi/MQTT
  xTaskCreatePinnedToCore(TaskUI,       "TaskUI",   20000, NULL, 2, &taskUIcHandle,      0);
  xTaskCreatePinnedToCore(Taskwifi_udp, "TaskWifi", 20000, NULL, 1, &taskwifi_udpHandle, 1);
}

void loop() {
  // Vide - gestion par taches FreeRTOS
}

/*--------------------------------------------------*/
/*----------------- Tache UI (Core 0) --------------*/
/*--------------------------------------------------*/
void TaskUI(void *pvParameters) {
  (void)pvParameters;

  // ---------------------------------------------------------
  //  Abonnement au TWDT pour cette tache (Core 0)
  //  esp_task_wdt_add(NULL) = abonne la tache COURANTE
  //  Elle doit appeler esp_task_wdt_reset() au moins toutes
  //  les WDT_TIMEOUT_MS millisecondes
  // ---------------------------------------------------------
  /* disable WDT
  esp_err_t err = esp_task_wdt_add(NULL);
  if (err != ESP_OK)
    Serial.printf("[TaskUI] WDT add error: %s\n", esp_err_to_name(err));
  else
    Serial.println("[TaskUI] WDT souscrit Core 0");
*/
  for (;;) {

    unsigned int numberOfSamples = 0;
    sumV = 0; sumI = 0; sumP = 0;
    unsigned int time_now_second = millis() / 1000;

    if (zero_crossCount >= totalCount) zero_crossCount = 0;

    // Acquisition 20 demi-periodes ≈ 200 ms << WDT 60 s
    while (zero_crossCount < totalCount) {

      if (zero_cross_flag) {
        zero_cross_flag = false;
        zero_crossCount++;
      }

      numberOfSamples++;
      memo_readV = readV;
      readV = analogRead(voltageSensorPin) / 4;

      if (memo_readV == 0 && readV == 0) break;  // absence reseau

      readI = analogRead(currentSensorPin) / 4;

      // RMS (mode calibration uniquement)
      if (CALIBRATION) {
        sqV   = (readV - ADC_V_0V) * (readV - ADC_V_0V);
        sumV += sqV;
        sqI   = (readI - ADC_I_0A) * (readI - ADC_I_0A);
        sumI += sqI;
      }

      // Puissance instantanee avec correction de phase
      instP = ((memo_readV - ADC_V_0V) + phasecalibration * ((readV - ADC_V_0V) - (memo_readV - ADC_V_0V)))
              * (readI - ADC_I_0A);
      sumP += instP;

      if (numberOfSamples > 0) {
        if (CALIBRATION) {
          V = Vcalibration * sqrt(sumV / numberOfSamples);
          I = Icalibration * sqrt(sumI / numberOfSamples);
        }
        rPower     = Vcalibration * Icalibration * sumP / numberOfSamples;
        Power_wifi = rPower / 1000.0f;
      }

      // Regulation DIM
      if (rPower > 0) dimstep = (int)(rPower / 1000.0f / reaction_coeff) + 1;
      else            dimstep = (int)(1 - rPower / 1000.0f / reaction_coeff);

      if      (rPower < tresholdP) dim = (dim > dimstep)             ? dim - dimstep : 0;
      else if (rPower > tresholdP) dim = (dim + dimstep < dimmax)    ? dim + dimstep : dimmax;

      dimphase = dim_sinus[dim] + dimthreshold;

      // Commande relais
      if ((millis() - unballasting_time) > unballasting_timeout) {

        if (dim < unballasting_dim_min) {
          if (unballasting_counter > 10) {
            digitalWrite(unballast_relay1, HIGH); relay_1 = true;
            unballasting_counter = 0; unballasting_time = millis();
          }
          unballasting_counter++;
        }

        if (dim > unballasting_dim_max) {
          if (unballasting_counter > 10) {
            digitalWrite(unballast_relay1, LOW); relay_1 = false;
            unballasting_time = millis(); unballasting_counter = 0;
          }
          unballasting_counter++;
        }

        if (Power_wifi > PowerMax)                                   { digitalWrite(unballast_relay2, HIGH); relay_2 = true; }
        if (relay_2 && Power_wifi < (PowerMax - Power_water_heater)) { digitalWrite(unballast_relay2, LOW);  relay_2 = false; }
      }

      // Moyennes MQTT (sous mutex)
      if ((millis() - mean_power_time) > mean_power_timing) {
        if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          mean_power_MQTT    = (mean_power_counter > 0) ? mean_power / mean_power_counter : 0;
          mean_power = 0; mean_power_counter = 0; send_MQTT = true;
          xSemaphoreGive(xMutex);
        }
        mean_power_time = millis();
      } else { mean_power += Power_wifi; mean_power_counter++; }

      if ((millis() - mean_power_time_5mn) > mean_power_timing_5mn) {
        if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          mean_power_MQTT_5mn    = (mean_power_counter_5mn > 0) ? mean_power_5mn / mean_power_counter_5mn : 0;
          mean_power_5mn = 0; mean_power_counter_5mn = 0; send_MQTT_5mn = true;
          xSemaphoreGive(xMutex);
        }
        mean_power_time_5mn = millis();
      } else { mean_power_5mn += Power_wifi; mean_power_counter_5mn++; }

      if ((millis() - mean_power_time_10mn) > mean_power_timing_10mn) {
        if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          mean_power_MQTT_10mn    = (mean_power_counter_10mn > 0) ? mean_power_10mn / mean_power_counter_10mn : 0;
          mean_power_10mn = 0; mean_power_counter_10mn = 0; send_MQTT_10mn = true;
          xSemaphoreGive(xMutex);
        }
        mean_power_time_10mn = millis();
        Serial.print("mean_power_10mn (W): "); Serial.println(mean_power_MQTT_10mn);
      } else { mean_power_10mn += Power_wifi; mean_power_counter_10mn++; }

      // Affichage toutes les 2 s
      if (time_now_second >= memo_temps + 2) {
        if (!CALIBRATION && !VERBOSE) {
          memo_temps = time_now_second;
          display.setColor(BLACK); display.fillRect(0, 0, 128, 22); display.setColor(WHITE);
          display.drawString(0, 0, String(int(Power_wifi)) + " || " + String(dim));
          display.display();
        }
        if (CALIBRATION) {
          Serial.print(V); Serial.print("  |  "); Serial.print(I / 1000); Serial.print("  |  "); Serial.println(rPower / 1000);
          display.clear();
          display.drawString(0, 0,  String(int(V)) + "||" + String(int(I / 1000)));
          display.drawString(0, 22, String(int(Power_wifi)));
          display.display();
        }
        if (VERBOSE) {
          Serial.print(rPower/1000); Serial.print("  ||  "); Serial.print(dimstep);   Serial.print("  ||  ");
          Serial.print(dim);         Serial.print("  ||  "); Serial.print(dimphase);  Serial.print("  ||  ");
          Serial.print(relay_1);     Serial.print("  ||  "); Serial.print(relay_2);   Serial.print("  ||  ");
          Serial.print(unballasting_counter); Serial.print("  ||  ");
          Serial.println(millis() - unballasting_time);
        } else {
          delay(1);
        }
      }

    }  // end while zero_crossCount

    // ---------------------------------------------------------
    //  RESET WATCHDOG Core 0
    //  Appele a chaque fin de cycle de mesure (~200 ms)
    //  Largement en dessous du timeout de 60 s
    // ---------------------------------------------------------
    //esp_task_wdt_reset();

  }  // end for(;;)
}

/*--------------------------------------------------*/
/*-------------- Tache WiFi/MQTT (Core 1) ----------*/
/*--------------------------------------------------*/
void Taskwifi_udp(void *pvParameters) {
  (void)pvParameters;

  // ---------------------------------------------------------
  //  Abonnement au TWDT pour cette tache (Core 1)
  // ---------------------------------------------------------
  /* disable WDT
  esp_err_t err = esp_task_wdt_add(NULL);
  if (err != ESP_OK)
    Serial.printf("[TaskWifi] WDT add error: %s\n", esp_err_to_name(err));
  else
    Serial.println("[TaskWifi] WDT souscrit Core 1");
*/
  for (;;) {

    // Attente du flag d'envoi MQTT
    // Le WDT est resete dans la boucle d'attente pour eviter
    // le declenchement pendant la periode creuse entre deux envois
    while (!send_MQTT) {
      if ((millis() - MQTT_time) > MQTT_timeout) {
        Serial.println("Timeout MQTT depasse -> restart");
        dim_test = 1;
        sprintf(mystring_dim, "%g", dim_test);
        if (Connect_MQTT()) client.publish(topic_dim, mystring_dim, true);
        delay(5000);
        ESP.restart();
      }
      //esp_task_wdt_reset();  // nourrit le WDT pendant l'attente (chaque 10 ms)
      delay(10);
    }

    MQTT_time = millis();

    if (Connect_MQTT()) {
      float local_20s = 0, local_5mn = 0, local_10mn = 0;
      bool  do_20s = false, do_5mn = false, do_10mn = false;

      if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (send_MQTT)      { local_20s  = mean_power_MQTT;     do_20s  = true; send_MQTT      = false; }
        if (send_MQTT_5mn)  { local_5mn  = mean_power_MQTT_5mn; do_5mn  = true; send_MQTT_5mn  = false; }
        if (send_MQTT_10mn) { local_10mn = mean_power_MQTT_10mn;do_10mn = true; send_MQTT_10mn = false; }
        xSemaphoreGive(xMutex);
      }
      // for test 
      //if (do_20s)  { sprintf(mystring_power_wifi,      "%g", local_20s);  client.publish(topic,      mystring_power_wifi,      true); }
      //if (do_5mn)  { sprintf(mystring_power_wifi_5mn,  "%g", local_5mn);  client.publish(topic_5mn,  mystring_power_wifi_5mn,  true); }
      //if (do_10mn) { sprintf(mystring_power_wifi_10mn, "%g", local_10mn); client.publish(topic_10mn, mystring_power_wifi_10mn, true); }
      if (do_10mn) { Serial.println ("topic 10mn boucle MQTT" ); }  // for test only
    }

    // Reset WDT apres publication
    //esp_task_wdt_reset();

    // Reset quotidien a minuit (NTP)
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
      Serial.println("NTP : heure non disponible");
      if ((millis() - time_24H) > timeout_24H) {
        Serial.println("Timeout 24H sans NTP -> restart");
        delay(5000); ESP.restart();
      }
    } else {
      time_24H = millis();
      if (timeinfo.tm_hour == 0 && timeinfo.tm_min == 0 && timeinfo.tm_mday != lastDay) {
        Serial.println("Reset quotidien minuit...");
        lastDay = timeinfo.tm_mday;
        delay(60000);
        ESP.restart();
      }
    }

    ElegantOTA.loop();

    // Reset WDT en fin de cycle complet
   // esp_task_wdt_reset();

  }  // end for(;;)
}

/*--------------------------------------------------*/
/*------------- Connexion WiFi + MQTT --------------*/
/*--------------------------------------------------*/
bool Connect_MQTT() {

  if (client.connected()) return true;

  client.disconnect();  delay(1000);
  WiFi.disconnect();    delay(1000);

  Serial.println("Reconnexion WiFi...");
  WiFi.setHostname(hostname);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Attente WiFi avec timeout 30 s
  // Le WDT est resete a chaque iteration (500 ms) pour ne pas
  // declencher pendant la reconnexion (jusqu'a 30 s possible)
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - wifiStart > 30000) {
      Serial.println("WiFi timeout Connect_MQTT -> restart");
      ESP.restart();
    }
    //esp_task_wdt_reset();  // nourrit le WDT pendant la reconnexion WiFi
    //delay(500);
  }
  Serial.print("WiFi OK - IP: "); Serial.println(WiFi.localIP());

  client.setServer(mqtt_broker, mqtt_port);
  String l_client_id = client_id + String(WiFi.macAddress());
  Serial.printf("MQTT connect as %s\n", l_client_id.c_str());

  //esp_task_wdt_reset();  // nourrit le WDT avant tentative MQTT

  if (client.connect(l_client_id.c_str(), mqtt_username, mqtt_password)) {
    Serial.println("MQTT connecte");
    display.drawString(0, 22, "MQTT OK");
    return true;
  } else {
    Serial.print("MQTT echec, etat: "); Serial.println(client.state());
    display.drawString(0, 22, "MQTT KO");
    return false;
  }
}
