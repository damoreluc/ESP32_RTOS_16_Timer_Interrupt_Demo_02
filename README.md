# ESP32_RTOS_16_Timer_Interrupt_Demo_02

Esempio d'uso di un mutex spinlock per proteggere le sezioni critiche di codice in un task e nella ISR associata ad un timer hardware.

* ISR: ogni 100ms commutazione periodica dello stato di un led direttamente dalla ISR del timer hardware e incremento di una variabile condivisa;
* Task: ogni 2 secondi decremento e stampa della variabile condivisa.
