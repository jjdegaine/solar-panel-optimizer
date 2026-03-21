/**
 * ESP32 - Calcul des millisecondes restantes avant minuit
 * Utilise la bibliothèque time.h et la synchronisation NTP
 */

#include <Arduino.h>
#include <time.h>
#include <WiFi.h>

// ── Configuration WiFi ────────────────────────────────────────────────────────
const char* WIFI_SSID     = "VOTRE_SSID";
const char* WIFI_PASSWORD = "VOTRE_MOT_DE_PASSE";

// ── Configuration NTP ─────────────────────────────────────────────────────────
const char* NTP_SERVER    = "pool.ntp.org";
const long  GMT_OFFSET_SEC   = 3600;      // UTC+1 (France heure normale)
const int   DAYLIGHT_OFFSET_SEC = 3600;   // +1h en été

// ─────────────────────────────────────────────────────────────────────────────

void connecterWiFi() {
  Serial.printf("Connexion au WiFi : %s ", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connecté !");
  Serial.printf("Adresse IP : %s\n", WiFi.localIP().toString().c_str());
}

/**
 * Retourne le nombre de millisecondes restantes avant minuit (00:00:00).
 * Repose uniquement sur time.h (struct tm, mktime, time).
 */
long long msecAvantMinuit() {
  // Heure actuelle sous forme de timestamp UNIX
  time_t maintenant = time(nullptr);

  // Décomposer en struct tm
  struct tm tmMaintenant;
  localtime_r(&maintenant, &tmMaintenant);

  // Construire un struct tm représentant le prochain minuit
  struct tm tmMinuit = tmMaintenant;   // copie (même date)
  tmMinuit.tm_hour = 0;
  tmMinuit.tm_min  = 0;
  tmMinuit.tm_sec  = 0;
  tmMinuit.tm_mday += 1;              // jour suivant → minuit
  // mktime normalise automatiquement le débordement de jour/mois/année
  time_t prochainMinuit = mktime(&tmMinuit);

  // Différence en secondes, convertie en millisecondes
  double diffSec = difftime(prochainMinuit, maintenant);
  long long diffMsec = (long long)(diffSec * 1000.0);

  return diffMsec;
}

void afficherTempsRestant(long long msec) {
  long long secondesTotal = msec / 1000;
  int heures   = secondesTotal / 3600;
  int minutes  = (secondesTotal % 3600) / 60;
  int secondes = secondesTotal % 60;
  long long ms = msec % 1000;

  Serial.printf("⏳ Avant minuit : %02dh %02dm %02ds %03llums  (%lld ms au total)\n",
                heures, minutes, secondes, ms, msec);
}

void afficherHeureActuelle() {
  time_t maintenant = time(nullptr);
  struct tm tmInfo;
  localtime_r(&maintenant, &tmInfo);
  char buf[32];
  strftime(buf, sizeof(buf), "%d/%m/%Y  %H:%M:%S", &tmInfo);
  Serial.printf("🕐 Heure locale  : %s\n", buf);
}

// ─────────────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== ESP32 – Millisecondes avant minuit ===");

  connecterWiFi();

  // Synchronisation de l'horloge via NTP
  Serial.println("Synchronisation NTP...");
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);

  // Attendre que l'heure soit valide (> année 2020)
  struct tm tmInfo;
  while (!getLocalTime(&tmInfo)) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nHeure synchronisée !\n");
}

void loop() {
  afficherHeureActuelle();

  long long msec = msecAvantMinuit();
  afficherTempsRestant(msec);

  Serial.println("─────────────────────────────────────────");

  delay(5000); // mise à jour toutes les 5 secondes
}
