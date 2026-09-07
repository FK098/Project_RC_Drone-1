#ifndef MIO_BMP280_H
#define MIO_BMP280_H

#include <Arduino.h>
#include <Wire.h>

class MioBMP280 {
public:
    MioBMP280(uint8_t address = 0x76);

    bool begin(bool initWire = true);
    bool checkChipID();

    // Letture compensate
    float readTemperature();
    float readPressure();       // ritorna hPa
    float readAltitude();       // altitudine relativa al decollo
    float readAltitude(float seaLevelhPa); // altitudine assoluta

    // Letture raw
    bool readRawData(int32_t &rawTemp, int32_t &rawPress);

    // Configurazione
    bool setSeaLevelPressure(float hPa);
    float getSeaLevelPressure() const;

    // Calibrazione (per debug)
    void printCalibration();

    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length);

private:
    uint8_t _address;

    // Coefficienti di calibrazione di fabbrica
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

    int32_t _t_fine;
    float _seaLevelhPa;
    float _baselinePressure; // pressione al decollo

    bool readCalibration();
    float compensateTemperature(int32_t adc_T);
    float compensatePressure(int32_t adc_P);

    static int16_t unisci16LE(uint8_t lsb, uint8_t msb);
    static uint16_t unisci16ULE(uint8_t lsb, uint8_t msb);
};

#endif
