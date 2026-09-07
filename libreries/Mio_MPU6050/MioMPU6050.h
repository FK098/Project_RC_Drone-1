#ifndef MIO_MPU6050_H
#define MIO_MPU6050_H

#include <Arduino.h>
#include <Wire.h>

class MioMPU6050 {
public:
    MioMPU6050(uint8_t address = 0x68);

    int begin(bool initWire = true);

    bool reset();
    bool wake();
    bool checkWhoAmI();

    bool setDLPF(uint8_t dlpf);
    bool setSampleRateDivider(uint8_t divider);
    bool setGyroRange(uint8_t range);
    bool setAccelRange(uint8_t range);

    bool readRaw(
        int16_t &ax,
        int16_t &ay,
        int16_t &az,
        int16_t &temp,
        int16_t &gx,
        int16_t &gy,
        int16_t &gz
    );

    bool readScaled(
        float &ax_g,
        float &ay_g,
        float &az_g,
        float &temp_c,
        float &gx_dps,
        float &gy_dps,
        float &gz_dps
    );

    float getAccelSensitivityG() const;
    float getGyroSensitivityDPS() const;

    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length);
    bool calibrateGyro(uint16_t samples = 1000);

    void setGyroOffset(float x, float y, float z);
    void getGyroOffset(float &x, float &y, float &z);

    void setAccelOffset(float x, float y, float z);
    void getAccelOffset(float &x, float &y, float &z);

private:
    uint8_t _address;
    uint8_t _gyroRange;
    uint8_t _accelRange;

    static int16_t unisci16(uint8_t alto, uint8_t basso);

    float _gyroOffsetDPS[3];
    float _accelOffsetG[3];
};

#endif
