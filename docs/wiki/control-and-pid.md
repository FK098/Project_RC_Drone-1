# Controllo e PID

## Frequenza

Il controllo e pianificato ogni 4000 us:

```text
1 / 0,004 s = 250 Hz
```

L'MPU6050 e configurato con DLPF e divisore per fornire il campionamento nominale a 250 Hz.

## Stima assetto

Il gyro viene integrato per ottenere una risposta rapida. Roll e pitch vengono corretti con l'accelerometro tramite filtro complementare:

```text
assetto = 0,98 * gyro_integrato + 0,02 * accelerometro
```

Yaw usa la velocita angolare del giroscopio. Il magnetometro non e ancora parte del controllo.

## Cascata

Roll e pitch usano due PID:

```text
angolo desiderato -> PID angolo -> rate desiderato -> PID rate -> mixer
```

Yaw usa un PID rate diretto. La derivata e calcolata sulla misura per ridurre il derivative kick quando cambia il setpoint.

I guadagni nello sketch sono valori iniziali e non sono una taratura certificata per il telaio, i motori o le eliche utilizzate.

## Mixer X

Il mixer attuale calcola quattro comandi, poi li limita a 1000-2000 us. I segni devono essere verificati con eliche rimosse: se un comando produce una correzione opposta, invertire il relativo asse o il segno nel mixer dopo aver identificato il problema.