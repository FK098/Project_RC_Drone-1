# Wiki del progetto RC Drone

Questa wiki descrive l'architettura attuale del progetto e la procedura di integrazione. Il codice resta la fonte primaria: quando una pagina e in conflitto con uno sketch, prevale lo sketch.

## Pagine

- [Architettura](architecture.md): responsabilita del Nano, dell'Uno e delle librerie.
- [Cablaggio](wiring.md): pin, alimentazione e collegamenti dei moduli.
- [Protocollo radio](radio-protocol.md): payload e configurazione nRF24L01+.
- [Controllo e PID](control-and-pid.md): filtro complementare, PID cascati e mixer X.
- [Installazione e test](setup-and-test.md): caricamento, controlli a banco e limiti attuali.

## Regola di sicurezza

Le prime prove vanno eseguite senza eliche. Il codice puo essere compilato senza hardware, ma solo un test reale puo confermare orientamento degli assi, ordine dei motori, verso delle eliche, alimentazione del radio-modulo e comportamento degli ESC.