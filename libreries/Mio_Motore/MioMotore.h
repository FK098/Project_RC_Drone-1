#ifndef MIO_MOTORE_H
#define MIO_MOTORE_H

#include <Arduino.h>
#include <Servo.h>

class MioMotore {
public:
    // Costruttore
    MioMotore();

    // Collega l'ESC al pin specificato
    void begin(uint8_t pin);

    // Esegue la sequenza di sicurezza per abilitare l'ESC
    void arm();

    // Imposta la potenza da 0 a 100% (Utile per test manuali)
    void setSpeed(int percentuale);

    // Imposta il valore PWM grezzo da 1000 a 2000 (Veloce, usato dal PID del drone)
    void setRawPWM(int pwm_value);

    // Imposta Manualmente il valore da inserire nei motori
    void ManualSet(int valore);
private:
    Servo _esc;
    uint8_t _pin;
};

#endif
