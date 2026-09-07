#include "MioQMC5883L.h"

// Registri del QMC5883L
static const uint8_t QMC_REG_DATA_X_LSB  = 0x00;
static const uint8_t QMC_REG_STATUS_1    = 0x06;
static const uint8_t QMC_REG_CONTROL_1   = 0x09;
static const uint8_t QMC_REG_CONTROL_2   = 0x0A;
static const uint8_t QMC_REG_SET_RESET   = 0x0B;
static const uint8_t QMC_REG_CHIP_ID     = 0x0D;


MioQMC5883L::MioQMC5883L(uint8_t address) {
    _address = address;
    _range = 0;

    _offsetX = 0.0;
    _offsetY = 0.0;
    _offsetZ = 0.0;

    _offsetRawX = 0;
    _offsetRawY = 0;
    _offsetRawZ = 0;
}


bool MioQMC5883L::begin(bool initWire) {
    if (initWire) {
        Wire.begin();
    }

    delay(50);

    if (!checkChipID()) {
        return false;
    }

    // Soft reset
    writeRegister(QMC_REG_CONTROL_2, 0x80);
    delay(10);

    // SET/RESET period
    writeRegister(QMC_REG_SET_RESET, 0x01);

    // Configura: Continuous, 100Hz, ±2G, OSR 512
    if (!configure(0x01, 0x02, 0x00, 0x00)) {
        return false;
    }

    return true;
}


bool MioQMC5883L::checkChipID() {
    uint8_t id = 0;
    if (!readRegisters(QMC_REG_CHIP_ID, &id, 1)) {
        return false;
    }
    return (id == 0xFF);
}


bool MioQMC5883L::readRaw(int16_t &mx, int16_t &my, int16_t &mz) {
    uint8_t b[6];

    if (!readRegisters(QMC_REG_DATA_X_LSB, b, 6)) {
        return false;
    }

    mx = unisci16LSB(b[0], b[1]);
    my = unisci16LSB(b[2], b[3]);
    mz = unisci16LSB(b[4], b[5]);

    return true;
}


bool MioQMC5883L::readScaled(float &mx_gauss, float &my_gauss, float &mz_gauss) {
    int16_t mxRaw, myRaw, mzRaw;

    if (!readRaw(mxRaw, myRaw, mzRaw)) {
        return false;
    }

    float scale = getScale();

    mx_gauss = (mxRaw - _offsetRawX) * scale;
    my_gauss = (myRaw - _offsetRawY) * scale;
    mz_gauss = (mzRaw - _offsetRawZ) * scale;

    return true;
}


float MioQMC5883L::getScale() const {
    switch (_range) {
        case 0:  return 1.0 / 12000.0;  // ±2G
        case 1:  return 1.0 / 3000.0;   // ±8G
        default: return 1.0 / 12000.0;
    }
}


float MioQMC5883L::getHeading(float mx, float my) {
    float heading = atan2(my, mx);
    heading = heading * 180.0 / PI;
    if (heading < 0) heading += 360.0;
    return heading;
}


float MioQMC5883L::getHeadingTiltCompensated(
    float mx, float my, float mz,
    float roll_rad, float pitch_rad
) {
    float Xh = mx * cos(pitch_rad) + mz * sin(pitch_rad);
    float Yh = mx * sin(roll_rad) * sin(pitch_rad)
    + my * cos(roll_rad)
    - mz * sin(roll_rad) * cos(pitch_rad);

    float heading = atan2(Yh, Xh);
    heading = heading * 180.0 / PI;
    if (heading < 0) heading += 360.0;
    return heading;
}


bool MioQMC5883L::calibrate(uint32_t durationMs) {
    int16_t minX = 32767, minY = 32767, minZ = 32767;
    int16_t maxX = -32768, maxY = -32768, maxZ = -32768;

    uint32_t startTime = millis();
    uint16_t samples = 0;

    while (millis() - startTime < durationMs) {
        int16_t mx, my, mz;

        if (readRaw(mx, my, mz)) {
            if (mx < minX) minX = mx;
            if (mx > maxX) maxX = mx;
            if (my < minY) minY = my;
            if (my > maxY) maxY = my;
            if (mz < minZ) minZ = mz;
            if (mz > maxZ) maxZ = mz;
            samples++;
        }

        delay(10);
    }

    if (samples == 0) {
        return false;
    }

    // Calcola offset raw
    _offsetRawX = (minX + maxX) / 2;
    _offsetRawY = (minY + maxY) / 2;
    _offsetRawZ = (minZ + maxZ) / 2;

    // Calcola offset in Gauss
    float scale = getScale();
    _offsetX = _offsetRawX * scale;
    _offsetY = _offsetRawY * scale;
    _offsetZ = _offsetRawZ * scale;

    return true;
}


void MioQMC5883L::setOffset(float x, float y, float z) {
    _offsetX = x;
    _offsetY = y;
    _offsetZ = z;

    // Converti in raw per la lettura
    float scale = getScale();
    _offsetRawX = (int16_t)(x / scale);
    _offsetRawY = (int16_t)(y / scale);
    _offsetRawZ = (int16_t)(z / scale);
}


void MioQMC5883L::getOffset(float &x, float &y, float &z) {
    x = _offsetX;
    y = _offsetY;
    z = _offsetZ;
}


void MioQMC5883L::getOffsetRaw(int16_t &x, int16_t &y, int16_t &z) {
    x = _offsetRawX;
    y = _offsetRawY;
    z = _offsetRawZ;
}


bool MioQMC5883L::configure(uint8_t mode, uint8_t odr, uint8_t range, uint8_t osr) {
    if (mode > 1 || odr > 3 || range > 1 || osr > 3) {
        return false;
    }

    _range = range;

    uint8_t controlByte = (osr << 6) | (range << 4) | (odr << 2) | mode;
    return writeRegister(QMC_REG_CONTROL_1, controlByte);
}


bool MioQMC5883L::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}


bool MioQMC5883L::readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length) {
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


int16_t MioQMC5883L::unisci16LSB(uint8_t lsb, uint8_t msb) {
    return (int16_t)(((uint16_t)msb << 8) | lsb);
}
