# Arduino Nano RC Drone Flight Controller

> Flight controller per quadricottero X basato su Arduino Nano / ATmega328P, con driver I2C/SPI custom, controllo PID e radiocomando Arduino Uno.

![Status](https://img.shields.io/badge/status-in_development-yellow)
![License](https://img.shields.io/badge/license-MIT-blue)
![Platform](https://img.shields.io/badge/platform-AVR%20%7C%20Arduino-red)

---

## Stato del progetto

Il progetto e in sviluppo. Il firmware e stato strutturato e controllato staticamente, ma non e stato ancora validato in volo. Ogni prova iniziale deve essere eseguita senza eliche e con il drone fissato.

Documentazione dettagliata: [wiki locale](docs/wiki/README.md).

## Obiettivo tecnico

Il repository documenta il firmware del drone: lettura dei registri dei sensori, filtro complementare, controllo PID cascato e mixer dei quattro motori. Le librerie `Mio*` sono scritte per questo progetto; RF24 e una libreria esterna inclusa nel repository.

- **GitHub:** [@FK098](https://github.com/FK098)

---

## 📌 Project Overview

Most commercial flight controllers rely on high-level libraries or pre-compiled firmware (Betaflight, ArduPilot, etc.). This project takes the opposite approach: **every driver, filter, and control algorithm is written from the component datasheets**, with a strict focus on the severe constraints of the ATmega328P:

- ⚡ **2 KB SRAM** — every byte counts
- ⏱️ **16 MHz clock** — deterministic timing is mandatory
- 📉 **8-bit architecture** — no floating-point luxury (but we use it where unavoidable)

### ✨ Key Features

- **Deterministic Flight Loop @ 250 Hz:** Fixed-cycle 4 ms scheduling ensures stable convergence of cascaded PID loops (Roll, Pitch, Yaw rate).
- **Custom Sensor Libraries (`Mio*`):** Bare-metal I2C drivers written from datasheets for the MPU-6050, BMP280, and QMC5883L, with register-level control, DLPF filtering, and factory calibration parsing.
- **Sensor Fusion:** Filtro complementare attivo per roll/pitch; heading tilt-compensato con QMC5883L disponibile nella libreria ma non ancora integrato nel loop di volo.
- **Cascaded PID Architecture:** Outer angle loop → inner rate loop → motor mixer, with integral windup protection and derivative-on-measurement to avoid setpoint kicks.
- **2.4 GHz Radio Link (`nRF24L01+`):** Collegamento radio unidirezionale controller -> flight controller con payload binario fisso e timeout failsafe.
- **ESC & Motor Management:** Segnale da 1000 a 2000 µs e disarmo iniziale; l'armamento viene richiesto dal controller e controllato dal flight controller.
- **Sensori opzionali:** BMP280 e QMC5883L vengono inizializzati, ma non sono ancora usati nel loop PID a 250 Hz.

------

## 📐 Pinout & Wiring Diagram

### I2C Bus (Sensors)
All I2C devices share the same bus. The MPU-6050 and BMP280 are standard, while the QMC5883L is mounted as far away from the high-current ESC wires as possible to avoid magnetic interference.

| Sensor Pin | Arduino Nano Pin | Notes |
| :--- | :--- | :--- |
| **SDA** | A4 | I2C Data Line |
| **SCL** | A5 | I2C Clock Line (400 kHz Fast Mode) |

### SPI Bus (nRF24L01 Radio)
> ⚠️ **Note:** Powered strictly via the AMS1117 3.3V step-down regulator. A 100µF bypass capacitor is soldered directly to the radio's VCC/GND pins to prevent voltage drops during transmission spikes.

| nRF24L01 Pin | Arduino Nano Pin | Description |
| :--- | :--- | :--- |
| **CE** | D7 | RX/TX activation |
| **CSN** | D8 | SPI Chip Select |
| **SCK** | D13 | Serial Clock |
| **MOSI** | D11 | Master Out Slave In |
| **MISO** | D12 | Master In Slave Out |

### Motor Outputs (PWM)
| ESC Channel | Arduino Nano Pin | Quadcopter Position (X Config) |
| :--- | :--- | :--- |
| **Motor 1** | D3 | Front Right (CCW) |
| **Motor 2** | D5 | Rear Right (CW) |
| **Motor 3** | D6 | Rear Left (CCW) |
| **Motor 4** | D9 | Front Left (CW) |

### Ground controller (Arduino Uno)

Il file [sketch_controller.ino](sketches/sketch_controller.ino) legge due joystick analogici e trasmette il payload al Nano. Il firmware del Nano e [sketch1_FlightController.ino](sketches/sketch1_FlightController.ino).

| Funzione | Pin Uno |
| :--- | :---: |
| RF24 CE | D7 |
| RF24 CSN | D8 |
| RF24 MOSI / MISO / SCK | D11 / D12 / D13 |
| Joystick sinistro X / Y | A0 / A1 |
| Joystick destro X / Y | A2 / A3 |
| Interruttore armamento | D4 verso GND |

Mappatura predefinita:

- joystick sinistro X: yaw;
- joystick sinistro Y: throttle;
- joystick destro X: roll;
- joystick destro Y: pitch.

L'interruttore usa `INPUT_PULLUP`: per armare deve essere attivo e il throttle deve essere al minimo. Dopo l'armamento il throttle puo essere aumentato; disattivando l'interruttore il controller trasmette immediatamente `armed = 0`.

---

## 🏗️ Hardware Assembly & 3D Printed Parts

To ensure the sensors operate correctly, vibration isolation is critical. If you are fabricating custom FDM mounts or structural chassis components for the drone, they will be uploaded to the `hardware/stl/` directory. Currently, the flight controller stack requires:
- Anti-vibration rubber standoffs for the Arduino/IMU mount.
- A non-magnetic mast/spacer for the QMC5883L compass.

---

## Installazione e primo avvio

### Prerequisiti

- Arduino IDE 2.x oppure PlatformIO.
- Core Arduino per Arduino Nano e Arduino Uno.
- Libreria RF24 disponibile nel percorso delle librerie.
- Tutte le cartelle `libreries/Mio_*` aggiunte come librerie locali.

La cartella `libreries` mantiene il nome storico del progetto; Arduino IDE puo richiedere di copiare o aggiungere manualmente le librerie nella cartella `libraries` dell'utente.

### Caricamento

1. Selezionare **Arduino Nano** per `sketch1_FlightController.ino` e **Arduino Uno** per `sketch_controller.ino`.
2. Selezionare la porta seriale corretta e il processore Nano corretto (`ATmega328P` oppure `ATmega328P Old Bootloader` se necessario).
3. Caricare prima il controller e poi il flight controller.
4. Aprire il Monitor Seriale a 115200 baud.
5. Verificare che IMU e RF24 risultino `OK` sul Nano.

Il Nano calibra il giroscopio durante il `setup()`: deve rimanere immobile per circa un secondo.

### Protocollo radio

Entrambi gli sketch devono mantenere questa struttura, senza modificare ordine o tipi:

```cpp
struct RcCommand {
    int16_t rollAngleCdeg;
    int16_t pitchAngleCdeg;
    int16_t yawRateCdeg;
    uint16_t throttleUs;
    uint8_t armed;
    uint8_t reserved;
};
```

Configurazione condivisa:

- indirizzo: `DRONE`;
- canale: `108`;
- data rate: `RF24_250KBPS`;
- payload: 10 byte;
- controller: trasmettitore;
- Nano: ricevitore;
- timeout failsafe del Nano: 250 ms.

### Test obbligatori prima del volo

- alimentare il modulo nRF24 a 3,3 V stabile, con condensatore vicino al modulo;
- verificare il riconoscimento dell'MPU6050;
- verificare che ogni motore corrisponda al numero e alla posizione previsti;
- verificare il verso degli assi e dei correttivi con eliche rimosse;
- scollegare il controller e verificare che dopo 250 ms i motori vadano a 1000 µs;
- verificare armamento e disarmo con throttle basso;
- tarare i PID gradualmente, prima con il drone fissato e poi con prove brevi.

Non eseguire prove con eliche montate finche ordine motori, versi, failsafe e disarmo non sono stati verificati separatamente.

### Repository RF24

`libreries/RF24` e una dipendenza inclusa nel repository e dispone della propria documentazione e dei propri esempi. Le modifiche al driver RF24 devono essere trattate separatamente dal firmware del drone.

## Struttura del repository

- `sketches/sketch1_FlightController.ino`: firmware del Nano.
- `sketches/sketch_controller.ino`: radiocomando per Uno.
- `libreries/Mio_MPU6050`: accelerometro e giroscopio.
- `libreries/Mio_BMP280`: pressione e altitudine, non ancora nel controllo.
- `libreries/Mio_QMC5883L`: magnetometro, non ancora nel controllo.
- `libreries/Mio_Motore`: interfaccia ESC tramite `Servo`.
- `libreries/RF24`: driver nRF24L01+ e materiale upstream.
- `docs/wiki`: documentazione tecnica del progetto.

## Limiti attuali e prossimi miglioramenti

- aggiungere isteresi e una macchina di armamento piu robusta;
- salvare calibrazioni in EEPROM;
- verificare e correggere l'identificazione del QMC5883L per i moduli che non restituiscono il valore atteso `0xFF`;
- aggiungere controllo tensione batteria;
- completare heading con QMC5883L e quota con BMP280;
- aggiungere un progetto PlatformIO per rendere riproducibile la compilazione;
- validare memoria SRAM e tempo di esecuzione sul Nano reale;
- aggiungere test del mixer e del protocollo su host.

## Comandi utili

Per controlli locali senza tool Arduino installati:

```bash
git diff --check
```

La compilazione finale deve essere eseguita con Arduino IDE o PlatformIO, usando il core AVR e le librerie installate.
