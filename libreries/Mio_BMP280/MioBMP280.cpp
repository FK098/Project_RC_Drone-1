#include "MioBMP280.h"

// Registri del BMP280
static const uint8_t BMP_REG_CHIP_ID    = 0xD0;
static const uint8_t BMP_REG_RESET      = 0xE0;
static const uint8_t BMP_REG_STATUS     = 0xF3;
static const uint8_t BMP_REG_CTRL_MEAS  = 0xF4;
static const uint8_t BMP_REG_CONFIG     = 0xF5;
static const uint8_t BMP_REG_PRESS_MSB  = 0xF7;
static const uint8_t BMP_REG_TEMP_MSB   = 0xFA;
static const uint8_t BMP_REG_CALIB_START = 0x88;

// Valori attesi
static const uint8_t BMP_CHIP_ID = 0x58;

// Comando di reset
static const uint8_t BMP_RESET_CMD = 0xB6;


MioBMP280::MioBMP280(uint8_t address) {
    _address = address;
    _t_fine = 0;
    _seaLevelhPa = 1013.25;
    _baselinePressure = 0.0;
}


bool MioBMP280::begin(bool initWire) {
    if (initWire) {
        Wire.begin();
    }

    delay(50);

    if (!checkChipID()) {
        return false;
    }

    // Soft reset
    if (!writeRegister(BMP_REG_RESET, BMP_RESET_CMD)) {
        return false;
    }
    delay(100);

    // Leggi i coefficienti di calibrazione
    if (!readCalibration()) {
        return false;
    }

    // Configura:
    // ctrl_meas: oversampling temp x1, oversampling press x4, Normal mode
    // 001 011 11 = 0x2F -> Tempo di conversione di circa 14ms (70 Hz)
    if (!writeRegister(BMP_REG_CTRL_MEAS, 0x2F)) {
        return false;
    }

    // config: standby 0.5ms, filtro IIR x4, SPI off
    // 000 010 00 = 0x08 -> Risposta molto più rapida (circa 50-60ms di latenza)
    if (!writeRegister(BMP_REG_CONFIG, 0x08)) {
        return false;
    }

    delay(50);

    // Fai fare al sensore un po' di "ginnastica" per far assestare il filtro IIR
    for (int i = 0; i < 20; i++) {
        readTemperature();
        readPressure();
        delay(20);
    }

    // Ora che il filtro interno è stabile, salva la pressione reale
    _baselinePressure = readPressure();

    return true;
}



bool MioBMP280::checkChipID() {
    uint8_t id = 0;

    if (!readRegisters(BMP_REG_CHIP_ID, &id, 1)) {
        return false;
    }

    // BMP280 originale restituisce 0x58
    // Alcuni cloni possono restituire 0x56 o 0x57
    return (id == 0x58 || id == 0x56 || id == 0x57);
}


bool MioBMP280::readCalibration() {
    uint8_t calib[24];

    // Leggi 24 byte a partire da 0x88
    if (!readRegisters(BMP_REG_CALIB_START, calib, 24)) {
        return false;
    }

    dig_T1 = unisci16ULE(calib[0], calib[1]);
    dig_T2 = unisci16LE(calib[2], calib[3]);
    dig_T3 = unisci16LE(calib[4], calib[5]);

    dig_P1 = unisci16ULE(calib[6], calib[7]);
    dig_P2 = unisci16LE(calib[8], calib[9]);
    dig_P3 = unisci16LE(calib[10], calib[11]);
    dig_P4 = unisci16LE(calib[12], calib[13]);
    dig_P5 = unisci16LE(calib[14], calib[15]);
    dig_P6 = unisci16LE(calib[16], calib[17]);
    dig_P7 = unisci16LE(calib[18], calib[19]);
    dig_P8 = unisci16LE(calib[20], calib[21]);
    dig_P9 = unisci16LE(calib[22], calib[23]);

    return true;
}


bool MioBMP280::readRawData(int32_t &rawTemp, int32_t &rawPress) {
    uint8_t b[6];

    // Leggi 6 byte a partire da 0xF7
    // 0xF7-0xF9: pressione, 0xFA-0xFC: temperatura
    if (!readRegisters(BMP_REG_PRESS_MSB, b, 6)) {
        return false;
    }

    // Pressione: 20 bit
    // b[0]=MSB, b[1]=LSB, b[2]=XLSB (solo 4 bit alti)
    rawPress = ((int32_t)b[0] << 12) |
    ((int32_t)b[1] << 4) |
    ((int32_t)b[2] >> 4);

    // Temperatura: 20 bit
    // b[3]=MSB, b[4]=LSB, b[5]=XLSB (solo 4 bit alti)
    rawTemp = ((int32_t)b[3] << 12) |
    ((int32_t)b[4] << 4) |
    ((int32_t)b[5] >> 4);

    return true;
}


float MioBMP280::compensateTemperature(int32_t adc_T) {
    // Formula dal datasheet BMP280 (floating point)
    float var1 = ((float)adc_T / 16384.0 - (float)dig_T1 / 1024.0) * (float)dig_T2;
    float var2 = (((float)adc_T / 131072.0 - (float)dig_T1 / 8192.0) *
    ((float)adc_T / 131072.0 - (float)dig_T1 / 8192.0)) * (float)dig_T3;

    _t_fine = (int32_t)(var1 + var2);

    float T = (var1 + var2) / 5120.0;
    return T;
}


float MioBMP280::compensatePressure(int32_t adc_P) {
    // Formula dal datasheet BMP280 (floating point)
    float var1 = (float)_t_fine / 2.0 - 64000.0;
    float var2 = var1 * var1 * (float)dig_P6 / 32768.0;
    var2 = var2 + var1 * (float)dig_P5 * 2.0;
    var2 = var2 / 4.0 + (float)dig_P4 * 65536.0;
    var1 = ((float)dig_P3 * var1 * var1 / 524288.0 + (float)dig_P2 * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * (float)dig_P1;

    if (var1 == 0.0) {
        return 0.0; // evita divisione per zero
    }

    float p = 1048576.0 - (float)adc_P;
    p = (p - var2 / 4096.0) * 6250.0 / var1;
    var1 = (float)dig_P9 * p * p / 2147483648.0;
    var2 = p * (float)dig_P8 / 32768.0;
    p = p + (var1 + var2 + (float)dig_P7) / 16.0;

    // p è in Pascal, converti in hPa
    return p / 100.0;
}


float MioBMP280::readTemperature() {
    int32_t rawTemp, rawPress;

    if (!readRawData(rawTemp, rawPress)) {
        return 0.0;
    }

    return compensateTemperature(rawTemp);
}


float MioBMP280::readPressure() {
    int32_t rawTemp, rawPress;

    if (!readRawData(rawTemp, rawPress)) {
        return 0.0;
    }

    // La temperatura deve essere compensata prima della pressione
    // perché aggiorna _t_fine
    compensateTemperature(rawTemp);

    return compensatePressure(rawPress);
}


float MioBMP280::readAltitude() {
    // Altitudine relativa al punto di decollo
    float pressure = readPressure();

    if (_baselinePressure == 0.0) {
        _baselinePressure = pressure;
    }

    // Formula semplificata per piccole differenze di quota
    // 1 hPa ≈ 8.43 metri a livello del mare
    float altitude = (_baselinePressure - pressure) * 8.43;

    return altitude;
}


float MioBMP280::readAltitude(float seaLevelhPa) {
    // Altitudine assoluta rispetto al livello del mare
    float pressure = readPressure();

    float altitude = 44330.0 * (1.0 - pow(pressure / seaLevelhPa, 1.0 / 5.255));

    return altitude;
}


bool MioBMP280::setSeaLevelPressure(float hPa) {
    if (hPa < 800.0 || hPa > 1200.0) {
        return false;
    }
    _seaLevelhPa = hPa;
    return true;
}


float MioBMP280::getSeaLevelPressure() const {
    return _seaLevelhPa;
}


void MioBMP280::printCalibration() {
    Serial.println("=== Coefficienti di calibrazione BMP280 ===");
    Serial.print("dig_T1 = "); Serial.println(dig_T1);
    Serial.print("dig_T2 = "); Serial.println(dig_T2);
    Serial.print("dig_T3 = "); Serial.println(dig_T3);
    Serial.print("dig_P1 = "); Serial.println(dig_P1);
    Serial.print("dig_P2 = "); Serial.println(dig_P2);
    Serial.print("dig_P3 = "); Serial.println(dig_P3);
    Serial.print("dig_P4 = "); Serial.println(dig_P4);
    Serial.print("dig_P5 = "); Serial.println(dig_P5);
    Serial.print("dig_P6 = "); Serial.println(dig_P6);
    Serial.print("dig_P7 = "); Serial.println(dig_P7);
    Serial.print("dig_P8 = "); Serial.println(dig_P8);
    Serial.print("dig_P9 = "); Serial.println(dig_P9);
    Serial.println("===========================================");
}


bool MioBMP280::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}


bool MioBMP280::readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length) {
    Wire.beginTransmission(_address);
    Wire.write(reg);

    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    uint8_t ricevuti = Wire.requestFrom(_address, length);

    if (ricevuti != length) {
        return false;
    }

    for (uint8_t i = 0; i < length; i++) {
        buffer[i] = Wire.read();
    }

    return true;
}


int16_t MioBMP280::unisci16LE(uint8_t lsb, uint8_t msb) {
    return (int16_t)(((uint16_t)msb << 8) | lsb);
}


uint16_t MioBMP280::unisci16ULE(uint8_t lsb, uint8_t msb) {
    return (uint16_t)(((uint16_t)msb << 8) | lsb);
}
