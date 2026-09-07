# Protocollo radio

Il collegamento attuale e unidirezionale: il controller Uno trasmette e il Nano riceve. L'ack RF24 conferma la consegna al trasmettitore, ma non esiste ancora un payload di telemetria applicativa dal Nano all'Uno.

## Configurazione comune

```text
address       DRONE
channel       108
data rate     250 kbps
payload       10 byte statici
controller    TX
flight unit   RX
```

## Payload

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

Unita:

- `rollAngleCdeg`: centesimi di grado, limite predefinito ±30 gradi;
- `pitchAngleCdeg`: centesimi di grado, limite predefinito ±30 gradi;
- `yawRateCdeg`: centesimi di grado al secondo, limite predefinito ±90 gradi/s;
- `throttleUs`: 1000-2000 us;
- `armed`: 0 o 1;
- `reserved`: zero, mantenuto per compatibilita futura.

Non cambiare ordine, tipo o dimensione dei campi senza aggiornare entrambi gli sketch.