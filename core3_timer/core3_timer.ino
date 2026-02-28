#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#include <Esp.h>

hw_timer_t *timer = NULL;
volatile bool flag_timer = false;

// Routine d'interruption
void IRAM_ATTR onTimer()
{
  flag_timer = true;   // faire le minimum dans l'ISR !
}

void setup()
{
  Serial.begin(115200);

  // Timer 0 avec prescaler 80 → 1 tick = 1 µs (80 MHz / 80)
  timer = timerBegin(1000000);  // fréquence 1 MHz

  // Attache l'interruption
  timerAttachInterrupt(timer, &onTimer);

  // Déclenche toutes les 73 µs = 73*128 => 10ms
  timerAlarm(timer, 73 , true, 0);

  timerStart(timer);
}

void loop()
{
  if (flag_timer)
  {
    flag_timer = false;

    // Code exécuté toutes les 73 us
    //Serial.println("Tick 73us");
  }
}