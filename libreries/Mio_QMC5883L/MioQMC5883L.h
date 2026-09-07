#ifndef MIO_QMC5883L_H
#define MIO_QMC5883L_H

#include <Arduino.h>
#include <Wire.h>

class MioQMC5883L {
public:
    MioQMC5883L(uint8_t address = 0x0D);

    bool begin(bool initWire = true);
    bool checkChipID();

    bool readRaw(int16_t &mx, int16_t &my, int16_t &mz);
    bool readScaled(float &mx_gauss, float &my_gauss, float &mz_gauss);

    float getHeading(float mx, float my);
    float getHeadingTiltCompensated(float mx, float my, float mz,
                                    float roll_rad, float pitch_rad);

    bool configure(uint8_t mode, uint8_t odr, uint8_t range, uint8_t osr);

    // Calibrazione
    bool calibrate(uint32_t durationMs = 30000);
    void setOffset(float x, float y, float z);
    void getOffset(float &x, float &y, float &z);
    void getOffsetRaw(int16_t &x, int16_t &y, int16_t &z);

    float getScale() const;

    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length);

private:
    uint8_t _address;
    uint8_t _range;

    float _offsetX;
    float _offsetY;
    float _offsetZ;

    int16_t _offsetRawX;
    int16_t _offsetRawY;
    int16_t _offsetRawZ;

    static int16_t unisci16LSB(uint8_t lsb, uint8_t msb);
};

#endif
