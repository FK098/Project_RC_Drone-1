#include <Wire.h>
#include <MioQMC5883L.h>

MioQMC5883L mag(0x0D);

void setup() {
    Serial.begin(115200);
    delay(500);

    if (!mag.begin()) {
        Serial.println("QMC5883L non trovato!");
        while (true) { delay(1000); }
    }

    Serial.println("QMC5883L trovato.");
    Serial.println();
    Serial.println("=== CALIBRAZIONE ===");
    Serial.println("Ruota il sensore in tutte le direzioni per 30 secondi.");
    Serial.println("Parto tra 3 secondi...");

    for (int i = 3; i > 0; i--) {
        Serial.print(i);
        Serial.print(" ");
        delay(1000);
    }

    Serial.println("VIA!");

    bool ok = mag.calibrate(30000);

    if (!ok) {
        Serial.println("Calibrazione fallita!");
        while (true) { delay(1000); }
    }

    // Stampa i risultati
    float offX, offY, offZ;
    mag.getOffset(offX, offY, offZ);

    int16_t offRawX, offRawY, offRawZ;
    mag.getOffsetRaw(offRawX, offRawY, offRawZ);

    Serial.println();
    Serial.println("=== CALIBRAZIONE COMPLETATA ===");
    Serial.print("Offset raw:  X=");
    Serial.print(offRawX);
    Serial.print("  Y=");
    Serial.print(offRawY);
    Serial.print("  Z=");
    Serial.println(offRawZ);

    Serial.print("Offset Gauss: X=");
    Serial.print(offX, 6);
    Serial.print("  Y=");
    Serial.print(offY, 6);
    Serial.print("  Z=");
    Serial.println(offZ, 6);

    Serial.println();
    Serial.println("Heading in tempo reale. Ruota il sensore.");
    Serial.println();
}

void loop() {
    float mx, my, mz;

    if (mag.readScaled(mx, my, mz)) {
        float heading = mag.getHeading(mx, my);

        Serial.print("Heading: ");
        Serial.print(heading, 1);
        Serial.print(" deg");
        Serial.print("  X: ");
        Serial.print(mx, 4);
        Serial.print(" G");
        Serial.print("  Y: ");
        Serial.print(my, 4);
        Serial.print(" G");
        Serial.print("  Z: ");
        Serial.println(mz, 4);
    }

    delay(100);
}
