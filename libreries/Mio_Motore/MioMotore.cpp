#include "MioMotore.h"

MioMotore::MioMotore() {
    // Il costruttore può rimanere vuoto
}

void MioMotore::begin(uint8_t pin) {
    _pin = pin;
    // Attach aggancia il pin e imposta i limiti assoluti (min 1000µs, max 2000µs)
    _esc.attach(_pin, 1000, 2000);
}

void MioMotore::arm() {
    // 1. Manda il segnale di "Gas a Zero" (1000 µs)
    _esc.writeMicroseconds(1000);

    // 2. Aspetta 3 secondi.
    // NOTA BENE: Questo delay() va usato SOLO nel setup(),
    // mai nel loop() altrimenti il drone cade!
    delay(3000);
}

void MioMotore::setSpeed(int percentuale) {
    // Sicurezza: blocca valori fuori range
    if (percentuale < 0) percentuale = 0;
    if (percentuale > 100) percentuale = 100;

    // Mappa la percentuale (0-100) nel segnale reale (1000-2000)
    int pwm_out = map(percentuale, 0, 100, 1000, 2000);

    _esc.writeMicroseconds(pwm_out);
}

void MioMotore::setRawPWM(int pwm_value) {
    // Questa è la funzione che userà il tuo algoritmo PID in volo.
    // Nessun calcolo, pura velocità. Solo un check di sicurezza.
    if (pwm_value < 1000) pwm_value = 1000;
    if (pwm_value > 2000) pwm_value = 2000;

    _esc.writeMicroseconds(pwm_value);
}

void MioMotore::ManualSet(int valore){
    if(valore<=2000&&valore>=1000){
        _esc.writeMicroseconds(valore);
    }
}
