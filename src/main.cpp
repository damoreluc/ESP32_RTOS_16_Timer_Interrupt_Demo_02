/*
 * FreeRTOS Esempio 16: sincronizzazione tra interrupt da timer hardware e task mediante sezione critica.
 *
 * Acquisizione A/D nella ISR del timer hardware, periodo 100ms e sincronizzazione con task
 * mediante sezione critica (mutex di tipo spinlock)
 *
 * Il task ha un tempo di ciclo di 2000ms, quindi la ISR verrà eseguita più volte durante ogni ciclo del task
 *
 * Nota: nel file soc.h sono definiti i riferimenti ai due core della ESP32:
 *   #define PRO_CPU_NUM (0)
 *   #define APP_CPU_NUM (1)
 *
 * Qui viene adoperata la APP_CPU
 *
 */

#include <Arduino.h>

// pin driver del Led
#define pinLed GPIO_NUM_23

// variabili globali
// divisore del timer hardware (per tick = 1us)
static const uint16_t timer_divider = 80;
// costante di tempo del timer hardware (100ms)
static const uint64_t timer_max_count = 100000;
// tempo di ciclo del task
static const TickType_t task_delay = pdMS_TO_TICKS(2000);

// Globals
// handler del timer hardware
static hw_timer_t *timer = NULL;
// conteggio degli interrupt occorsi
static volatile int16_t isr_counter = 0;
// MUTEX per la protezione delle critical section
// in cui ISR e task accedono alla variabile condivisa
static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

//*****************************************************************************
// Interrupt Service Routines (ISRs)

// ISR del timer hardware, eseguita quando il timer raggiunge il valore timer_max_count
void IRAM_ATTR onTimer()
{

  // Toggle LED
  int pin_state = digitalRead(pinLed);
  digitalWrite(pinLed, !pin_state);

  // sezione critica nella ISR (con le macro ESP-IDF)
  portENTER_CRITICAL_ISR(&spinlock);
  isr_counter++;
  portEXIT_CRITICAL_ISR(&spinlock);
  // fine della sezione critica

  // versione se si utilizza la versione base di FreeRTOS
  // UBaseType_t saved_int_status;
  // saved_int_status = taskENTER_CRITICAL_FROM_ISR();
  // isr_counter++;
  // taskEXIT_CRITICAL_FROM_ISR(saved_int_status);
}

//*****************************************************************************
// Tasks

// decrementa e stampa il valore della variabile ocndivisa
void printValues(void *parameters)
{
  // ciclo infinito
  while (1)
  {
    while (isr_counter > 0)
    {

      if(isr_counter == 20) {
        Serial.println("-------------------------------------");
      }

      // stampa il valore della variabile condivisa
      Serial.println(isr_counter);

      // decrementa la variabile condivisa
      // sezione critica nel task (con le macro ESP-IDF)
      portENTER_CRITICAL(&spinlock);
      isr_counter--;
      portEXIT_CRITICAL(&spinlock);
      // fine della sezione critica

      // versione se si utilizza la versione base di FreeRTOS
      // taskENTER_CRITICAL();
      // isr_counter--;
      // taskEXIT_CRITICAL();
    }

    // simula un tempo di ciclo lungo
    vTaskDelay(task_delay);
  }
}

//*****************************************************************************
// Main (sul core 1, con priorità 1)

// configurazione del sistema
void setup()
{
  // Configurazione della seriale
  Serial.begin(115200);

  // breve pausa
  vTaskDelay(pdMS_TO_TICKS(1000));
  Serial.println();
  Serial.println("FreeRTOS Software timer: demo 2 - sezione critica");

  // Crea il task per la stampa della variabile
  xTaskCreatePinnedToCore(
              printValues,    // Funzione da eseguire
              "Print values", // Nome del task
              1024,           // Stack del task
              NULL,           // parametri per il task
              1,              // Livello di priorità
              NULL,           // Puntatore al task
              APP_CPU_NUM);   // Core su sui eseguire il task

  // Configurazione del pin del led
  pinMode(pinLed, OUTPUT);

  // Crea e avvia il timer hardware num. 0 (num, divider, countUp)
  timer = timerBegin(0, timer_divider, true);

  // Associa la ISR al timer (timer, function, edge)
  timerAttachInterrupt(timer, &onTimer, true);

  // Imposta il valore di conteggio al quale eseguire la ISR (timer, count, autoreload)
  timerAlarmWrite(timer, timer_max_count, true);

  // Abilita la generazione degli interrupt del timer
  timerAlarmEnable(timer);

  // Elimina il task con "Setup e Loop"
  vTaskDelete(NULL);
}

void loop()
{
  // lasciare vuoto
}