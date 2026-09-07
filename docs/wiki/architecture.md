# Architettura

## Flight controller

`sketches/sketch1_FlightController.ino` gira sull'Arduino Nano / ATmega328P.

Il Nano:

1. inizializza i motori, l'MPU6050, BMP280, QMC5883L e RF24;
2. calibra il bias del giroscopio durante `setup()`;
3. legge il payload radio;
4. esegue il controllo ogni 4 ms, cioe 250 Hz;
5. porta tutti i motori a 1000 us quando l'IMU non e disponibile, il comando e disarmato o la radio e in timeout.

BMP280 e QMC5883L sono predisposti per quota e heading, ma non vengono letti nel percorso critico del PID attuale.

## Ground controller

`sketches/sketch_controller.ino` gira sull'Arduino Uno. Legge quattro ingressi analogici, applica deadband e limiti, gestisce l'armamento e invia un pacchetto ogni 20 ms, cioe 50 Hz.

L'armamento e latched: l'interruttore deve essere attivo con throttle basso; una volta armato, il controller resta armato finche l'interruttore non viene disattivato.

## Librerie

- `MioMPU6050`: letture scalate e calibrazione gyro.
- `MioBMP280`: temperatura, pressione e altitudine.
- `MioQMC5883L`: campo magnetico, heading e calibrazione hard/soft iron di base.
- `MioMotore`: segnali ESC con `Servo`, limitati a 1000-2000 us.
- `RF24`: comunicazione nRF24L01+.