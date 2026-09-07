# Installazione e test

## Installazione

1. Installare Arduino IDE 2.x o PlatformIO.
2. Installare il core AVR per Arduino Uno/Nano.
3. Installare RF24 e aggiungere le librerie `Mio_*` al percorso riconosciuto dall'IDE.
4. Selezionare `sketches/sketch_controller.ino` per Arduino Uno.
5. Selezionare `sketches/sketch1_FlightController.ino` per Arduino Nano.
6. Usare Monitor Seriale a 115200 baud.

Con un Nano clone puo essere necessario selezionare `ATmega328P Old Bootloader`.

## Sequenza di test

1. Testare la radio senza ESC collegati.
2. Verificare sul controller i valori analogici e lo stato armamento.
3. Verificare sul Nano che IMU e RF24 risultino `OK`.
4. Tenere il frame fermo durante la calibrazione gyro.
5. Testare un motore alla volta a bassa potenza, senza eliche.
6. Verificare ordine, posizione e verso di rotazione dei motori.
7. Inclinare il frame manualmente e controllare che il mixer reagisca nella direzione che lo raddrizza.
8. Spegnere il controller e verificare il disarmo dopo 250 ms.
9. Solo dopo questi test procedere alla taratura PID con il drone fissato.

## Limiti noti

- non c'e ancora telemetria applicativa di ritorno;
- BMP280 e QMC5883L non partecipano al PID;
- il driver QMC5883L accetta attualmente il chip ID `0xFF`: verificare il valore del proprio modulo prima di considerarlo operativo;
- la calibrazione non viene salvata in EEPROM;
- non c'e ancora misura della batteria;
- la compilazione deve essere verificata nell'ambiente Arduino/PlatformIO, perche il container di sviluppo puo non avere il core AVR e gli header Arduino installati.