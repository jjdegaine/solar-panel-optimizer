#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#include <Esp.h>

hw_timer_t *timer = NULL;
volatile bool flag_10ms = false;

// Routine d'interruption
void IRAM_ATTR onTimer()
{
  flag_10ms = true;   // faire le minimum dans l'ISR !
}

void setup()
{
  Serial.begin(115200);

  // Timer 0 avec prescaler 80 → 1 tick = 1 µs (80 MHz / 80)
  timer = timerBegin(1000000);  // fréquence 1 MHz

  // Attache l'interruption
  timerAttachInterrupt(timer, &onTimer);

  // Déclenche toutes les 10 000 µs = 10 ms
  timerAlarm(timer, 10000, true, 0);

  timerStart(timer);
}

void loop()
{
  if (flag_10ms)
  {
    flag_10ms = false;

    // Code exécuté toutes les 10 ms
    Serial.println("Tick 10 ms");
  }
}