// minMaxAndRangeChecker

// A simple tool to investigate the ADC values that are seen at the

// first four analogue inputs of an Atmega chip, as used on an emonTx

//
// Robin Emley (calypso_rae on the Open Energy Monitor forum)
//
// 20th April 2013
//
// 2026 03 correctif vie IA claude pour ESP32 core 3
// minMaxAndRangeChecker - Corrigé pour ESP32 Core 3.x

#include <Esp.h>

const byte voltageSensorPin = 34;
const byte currentSensorPin = 35;

int val_a0, val_a1;
int minVal_a0, minVal_a1;
int maxVal_a0, maxVal_a1;

int loopCount = 0;
unsigned long timeAtLastDisplay = 0;
byte displayLineCounter = 0;

void setup(void) {
  Serial.begin(115200);
  Serial.print("ready ...");
  
  resetMinAndMaxValues();  // ✅ Initialisation correcte dès le départ
  
  delay(7000);
  Serial.println();
  Serial.println(" The Min, Max and Range ADC values for analog inputs :");
}

void loop(void) {
  val_a0 = analogRead(voltageSensorPin) / 4;
  val_a1 = analogRead(currentSensorPin) / 4;

  if (val_a0 < minVal_a0) { minVal_a0 = val_a0; }
  if (val_a0 > maxVal_a0) { maxVal_a0 = val_a0; }
  if (val_a1 < minVal_a1) { minVal_a1 = val_a1; }
  if (val_a1 > maxVal_a1) { maxVal_a1 = val_a1; }

  unsigned long timeNow = millis();
  if ((timeNow - timeAtLastDisplay) >= 3000) {
    timeAtLastDisplay = timeNow;

    displayVal(minVal_a0);
    displayVal(maxVal_a0);
    displayVal(maxVal_a0 - minVal_a0);
    Serial.print(";  ");

    displayVal(minVal_a1);
    displayVal(maxVal_a1);
    displayVal(maxVal_a1 - minVal_a1);
    Serial.println(";  ");

    resetMinAndMaxValues();

    displayLineCounter++;
    if (displayLineCounter >= 5) {
      Serial.println();
      displayLineCounter = 0;
      delay(2000);
    }
  }
}

void resetMinAndMaxValues() {
  minVal_a0 = 1023; minVal_a1 = 1023;
  maxVal_a0 = 0;    maxVal_a1 = 0;
}

void displayVal(int intVal) {
  char strVal[6];  // ✅ Taille correcte : 5 chiffres max + '\0'
  byte lenOfStrVal;

  itoa(intVal, strVal, 10);
  lenOfStrVal = strlen(strVal);

  for (int i = 0; i < (4 - lenOfStrVal); i++) {
    Serial.print(' ');
  }
  Serial.print(strVal);
}