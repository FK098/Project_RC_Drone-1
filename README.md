# Arduino Nano RC Drone Flight Controller

> Flight controller per quadricottero X basato su Arduino Nano / ATmega328P, con driver I2C/SPI custom, controllo PID e radiocomando Arduino Uno.

![Status](https://img.shields.io/badge/status-in_development-yellow)
![License](https://img.shields.io/badge/license-MIT-blue)
![Platform](https://img.shields.io/badge/platform-AVR%20%7C%20Arduino-red)

---

## Stato del progetto

Il progetto è in sviluppo. Il firmware è stato strutturato e controllato staticamente, ma non è stato ancora validato in volo. Ogni prova iniziale deve essere eseguita senza eliche e con il drone fissato.

Documentazione dettagliata: [wiki locale](docs/wiki/README.md).

## Obiettivo tecnico

Il repository documenta il firmware del drone: lettura dei registri dei sensori, filtro complementare, controllo PID in cascata e mixer dei quattro motori. Le librerie `Mio*` sono scritte per questo progetto; RF24 è una libreria esterna inclusa nel repository.

- **GitHub:** [@FK098](https://github.com/FK098)

---

## 📌 Panoramica del Progetto

La maggior parte dei flight controller commerciali si affida a librerie ad alto livello o firmware precompilati (Betaflight, ArduPilot, ecc.). Questo progetto adotta l'approccio opposto: **ogni driver, filtro e algoritmo di controllo è scritto a partire dai datasheet dei componenti**, con una rigorosa attenzione alle severe limitazioni dell'ATmega328P:

- ⚡ **2 KB SRAM** — ogni byte conta
- ⏱️ **16 MHz clock** — un timing deterministico è obbligatorio
- 📉 **Architettura a 8-bit** — nessun lusso per la virgola mobile (ma la usiamo dove inevitabile)

### ✨ Funzionalità Chiave

- **Loop di Volo Deterministico a 250 Hz:** Lo scheduling a ciclo fisso di 4 ms garantisce una convergenza stabile dei loop PID in cascata (rate di Roll, Pitch, Yaw).
- **Librerie Sensori Custom (`Mio*`):** Driver I2C bare-metal scritti a partire dai datasheet per MPU-6050, BMP280 e QMC5883L, con controllo a livello di registri, filtraggio DLPF e parsing della calibrazione di fabbrica.
- **Sensor Fusion:** Filtro complementare attivo per roll/pitch; heading tilt-compensato con QMC5883L disponibile nella libreria ma non ancora integrato nel loop di volo.
- **Architettura PID in Cascata:** Loop esterno per l'angolo → loop interno per il rate → motor mixer, con protezione anti-windup integrale e calcolo della derivata sulla misurazione per evitare sbalzi improvvisi (setpoint kick).
- **Collegamento Radio a 2.4 GHz (`nRF24L01+`):** Collegamento radio unidirezionale controller -> flight controller con payload binario fisso e timeout failsafe.
- **Gestione Motori & ESC:** Segnale da 1000 a 2000 µs e disarmo iniziale; l'armamento viene richiesto dal controller e gestito dal flight controller.
- **Sensori opzionali:** BMP280 e QMC5883L vengono inizializzati, ma non sono ancora usati nel loop PID a 250 Hz.
- Mantenere fermi gli assi roll/pitch/yaw durante la calibrazione iniziale del controller.
---

## 📐 Pinout e Schema di Collegamento

### Bus I2C (Sensori)
Tutti i dispositivi I2C condividono lo stesso bus. L'MPU-6050 e il BMP280 sono standard, mentre il QMC5883L è montato il più lontano possibile dai cavi ad alta corrente degli ESC per evitare interferenze magnetiche.

| Pin Sensore | Pin Arduino Nano | Note |
| :--- | :--- | :--- |
| **SDA** | A4 | Linea Dati I2C |
| **SCL** | A5 | Linea Clock I2C (Modalità Fast a 400 kHz) |

### Bus SPI (Radio nRF24L01)
> ⚠️ **Nota:** Alimentato rigorosamente tramite il regolatore step-down AMS1117 a 3.3V. Un condensatore di bypass da 100µF è saldato direttamente sui pin VCC/GND della radio per prevenire cali di tensione durante i picchi di trasmissione.

| Pin nRF24L01 | Pin Arduino Nano | Descrizione |
| :--- | :--- | :--- |
| **CE** | D7 | Attivazione RX/TX |
| **CSN** | D8 | Selezione Chip SPI (Chip Select) |
| **SCK** | D13 | Clock Seriale |
| **MOSI** | D11 | Master Out Slave In |
| **MISO** | D12 | Master In Slave Out |

### Output Motori (PWM)
| Canale ESC | Pin Arduino Nano | Posizione sul Quadricottero (Configurazione a X) |
| :--- | :--- | :--- |
| **Motore 1** | D3 | Anteriore Destro (CCW - Antiorario) |
| **Motore 2** | D5 | Posteriore Destro (CW - Orario) |
| **Motore 3** | D6 | Posteriore Sinistro (CCW - Antiorario) |
| **Motore 4** | D9 | Anteriore Sinistro (CW - Orario) |

### Controller di Terra (Arduino Uno)

Il file [sketch_controller.ino](sketches/sketch_controller.ino) legge due joystick analogici e trasmette il payload al Nano. Il firmware del Nano è [sketch1_FlightController.ino](sketches/sketch1_FlightController.ino).

| Funzione | Pin Uno |
| :--- | :---: |
| RF24 CE | D7 |
| RF24 CSN | D8 |
| RF24 MOSI / MISO / SCK | D11 / D12 / D13 |
| Joystick sinistro X / Y | A0 / A1 |
| Joystick destro X / Y | A2 / A3 |
| Interruttore armamento | D4 verso GND |

Mappatura predefinita:
- joystick sinistro X: yaw
- joystick sinistro Y: throttle
- joystick destro X: roll
- joystick destro Y: pitch

L'interruttore usa `INPUT_PULLUP`: per armare deve essere attivo e il throttle deve essere al minimo. Dopo l'armamento il throttle può essere aumentato; disattivando l'interruttore il controller trasmette immediatamente `armed = 0`.

---

## 🏗️ Assemblaggio Hardware e Parti Stampate in 3D

Per garantire il corretto funzionamento dei sensori, l'isolamento dalle vibrazioni è critico. Se stai realizzando supporti FDM personalizzati o componenti strutturali del telaio per il drone, questi verranno caricati nella directory `hardware/stl/`. Attualmente, lo stack del flight controller richiede:
- Distanziali in gomma antivibrazione per il supporto di Arduino/IMU.
- Un albero/distanziale amagnetico per la bussola QMC5883L.

---

## Installazione e primo avvio

### Prerequisiti

- Arduino IDE 2.x oppure PlatformIO.
- Core Arduino per Arduino Nano e Arduino Uno.
- Libreria RF24 disponibile nel percorso delle librerie.
- Tutte le cartelle `libreries/Mio_*` aggiunte come librerie locali.

La cartella `libreries` mantiene il nome storico del progetto; l'IDE di Arduino può richiedere di copiare o aggiungere manualmente le librerie nella cartella `libraries` dell'utente.

### Caricamento

1. Selezionare **Arduino Nano** per `sketch1_FlightController.ino` e **Arduino Uno** per `sketch_controller.ino`.
2. Selezionare la porta seriale corretta e il processore Nano corretto (`ATmega328P` oppure `ATmega328P Old Bootloader` se necessario).
3. Caricare prima il controller e poi il flight controller.
4. Aprire il Monitor Seriale a 115200 baud.
5. Verificare che IMU e RF24 risultino `OK` sul Nano.

Il Nano calibra il giroscopio durante il `setup()`: il drone deve rimanere immobile per circa un secondo all'avvio.

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
