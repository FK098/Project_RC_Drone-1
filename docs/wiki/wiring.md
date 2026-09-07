# Cablaggio

## Flight controller: Arduino Nano

| Bus o funzione | Pin Nano |
| --- | --- |
| I2C SDA | A4 |
| I2C SCL | A5 |
| SPI MOSI | D11 |
| SPI MISO | D12 |
| SPI SCK | D13 |
| nRF24 CE | D7 |
| nRF24 CSN | D8 |
| Motore 1, front right CCW | D3 |
| Motore 2, rear right CW | D5 |
| Motore 3, rear left CCW | D6 |
| Motore 4, front left CW | D9 |

MPU6050, BMP280 e QMC5883L condividono il bus I2C. Gli indirizzi predefiniti nei driver sono rispettivamente `0x68`, `0x76` e `0x0D`.

## Ground controller: Arduino Uno

| Funzione | Pin Uno |
| --- | --- |
| Joystick sinistro X / Y | A0 / A1 |
| Joystick destro X / Y | A2 / A3 |
| Interruttore armamento | D4 verso GND |
| nRF24 CE / CSN | D7 / D8 |
| nRF24 MOSI / MISO / SCK | D11 / D12 / D13 |

## Alimentazione nRF24

Il nRF24L01+ deve ricevere 3,3 V stabili, non 5 V. Usare un regolatore adeguato e un condensatore di bypass vicino al modulo. La massa del modulo, dell'Arduino e degli ESC deve avere un riferimento comune, secondo lo schema di alimentazione del proprio telaio.

Prima di alimentare, verificare la piedinatura specifica del modulo acquistato: le serigrafie e l'orientamento possono cambiare tra moduli.