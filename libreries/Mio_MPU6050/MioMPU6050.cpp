#include "MioMPU6050.h"


static const uint8_t MPU6050_REG_SMPLRT_DIV   = 0x19;
static const uint8_t MPU6050_REG_CONFIG       = 0x1A;
static const uint8_t MPU6050_REG_GYRO_CONFIG  = 0x1B;
static const uint8_t MPU6050_REG_ACCEL_CONFIG = 0x1C;
static const uint8_t MPU6050_REG_PWR_MGMT_1   = 0x6B;
static const uint8_t MPU6050_REG_WHO_AM_I     = 0x75;
static const uint8_t MPU6050_REG_DATA_START   = 0x3B;


MioMPU6050::MioMPU6050(uint8_t address) {
    _address = address;
    _gyroRange = 0;
    _accelRange = 0;
    _address = address;
    _gyroRange = 0;
    _accelRange = 0;

    _gyroOffsetDPS[0] = 0.0;
    _gyroOffsetDPS[1] = 0.0;
    _gyroOffsetDPS[2] = 0.0;

    _accelOffsetG[0] = 0.0;
    _accelOffsetG[1] = 0.0;
    _accelOffsetG[2] = 0.0;
}

bool MioMPU6050::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.write(value);

    return Wire.endTransmission() == 0;
}


bool MioMPU6050::readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length) {
    Wire.beginTransmission(_address);
    Wire.write(reg);

    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    size_t ricevuti = Wire.requestFrom(_address, (size_t)length);

    if (ricevuti != length) {
        return false;
    }

    for (uint8_t i = 0; i < length; i++) {
        buffer[i] = Wire.read();
    }

    return true;
}


bool MioMPU6050::wake() {
    return writeRegister(MPU6050_REG_PWR_MGMT_1, 0x01);
}


bool MioMPU6050::checkWhoAmI() {
    uint8_t who = 0;

    if (!readRegisters(MPU6050_REG_WHO_AM_I, &who, 1)) {
        return false;
    }
    // Il valore tipico è 0x68.
    // Accetto anche 0x69 per robustezza in caso di AD0 alto o cloni.
    return (who == 0x68 || who == 0x69||who==0x70);
}


bool MioMPU6050::setDLPF(uint8_t dlpf) {
    if (dlpf > 6) {
        return false;
    }

    return writeRegister(MPU6050_REG_CONFIG, dlpf);
}


bool MioMPU6050::setSampleRateDivider(uint8_t divider) {
    return writeRegister(MPU6050_REG_SMPLRT_DIV, divider);
}


bool MioMPU6050::setGyroRange(uint8_t range) {
    if (range > 3) {
        return false;
    }

    _gyroRange = range;

    // FS_SEL nei bit 4:3
    // range 0 -> 0x00
    // range 1 -> 0x08
    // range 2 -> 0x10
    // range 3 -> 0x18
    return writeRegister(MPU6050_REG_GYRO_CONFIG, range << 3);
}


bool MioMPU6050::setAccelRange(uint8_t range) {
    if (range > 3) {
        return false;
    }

    _accelRange = range;

    // AFS_SEL nei bit 4:3
    // range 0 -> 0x00
    // range 1 -> 0x08
    // range 2 -> 0x10
    // range 3 -> 0x18
    return writeRegister(MPU6050_REG_ACCEL_CONFIG, range << 3);
}


bool MioMPU6050::readRaw(
    int16_t &ax,
    int16_t &ay,
    int16_t &az,
    int16_t &temp,
    int16_t &gx,
    int16_t &gy,
    int16_t &gz
) {
    uint8_t b[14];

    if (!readRegisters(MPU6050_REG_DATA_START, b, 14)) {
        return false;
    }

    ax   = unisci16(b[0], b[1]);
    ay   = unisci16(b[2], b[3]);
    az   = unisci16(b[4], b[5]);
    temp = unisci16(b[6], b[7]);
    gx   = unisci16(b[8], b[9]);
    gy   = unisci16(b[10], b[11]);
    gz   = unisci16(b[12], b[13]);

    return true;
}


bool MioMPU6050::readScaled(
    float &ax_g,
    float &ay_g,
    float &az_g,
    float &temp_c,
    float &gx_dps,
    float &gy_dps,
    float &gz_dps
) {
    int16_t axRaw, ayRaw, azRaw, temp;
    int16_t gxRaw, gyRaw, gzRaw;

    if (!readRaw(axRaw, ayRaw, azRaw, temp, gxRaw, gyRaw, gzRaw)) {
        return false;
    }

    float accelSensitivity = getAccelSensitivityG();
    float gyroSensitivity = getGyroSensitivityDPS();

    ax_g = axRaw / accelSensitivity;
    ay_g = ayRaw / accelSensitivity;
    az_g = azRaw / accelSensitivity;

    gx_dps = gxRaw / gyroSensitivity;
    gy_dps = gyRaw / gyroSensitivity;
    gz_dps = gzRaw / gyroSensitivity;

    ax_g -= _accelOffsetG[0];
    ay_g -= _accelOffsetG[1];
    az_g -= _accelOffsetG[2];

    gx_dps -= _gyroOffsetDPS[0];
    gy_dps -= _gyroOffsetDPS[1];
    gz_dps -= _gyroOffsetDPS[2];

    return true;
}


float MioMPU6050::getAccelSensitivityG() const {
    switch (_accelRange) {
        case 0:
            return 16384.0;
        case 1:
            return 8192.0;
        case 2:
            return 4096.0;
        case 3:
            return 2048.0;
        default:
            return 16384.0;
    }
}


float MioMPU6050::getGyroSensitivityDPS() const {
    switch (_gyroRange) {
        case 0:
            return 131.0;
        case 1:
            return 65.5;
        case 2:
            return 32.8;
        case 3:
            return 16.4;
        default:
            return 131.0;
    }
}







int16_t MioMPU6050::unisci16(uint8_t alto, uint8_t basso) {
    return (int16_t)((alto << 8) | basso);
}

bool MioMPU6050::calibrateGyro(uint16_t samples) {
  float sumX = 0.0;
  float sumY = 0.0;
  float sumZ = 0.0;

  uint16_t validReads = 0;

  // Scarta qualche campione iniziale
  for (uint8_t i = 0; i < 20; i++) {
    int16_t ax, ay, az, temp;
    int16_t gx, gy, gz;

    readRaw(ax, ay, az, temp, gx, gy, gz);
    delay(2);
  }

  for (uint16_t i = 0; i < samples; i++) {
    int16_t ax, ay, az, temp;
    int16_t gx, gy, gz;

    if (readRaw(ax, ay, az, temp, gx, gy, gz)) {
      float gyroSensitivity = getGyroSensitivityDPS();

      sumX += gx / gyroSensitivity;
      sumY += gy / gyroSensitivity;
      sumZ += gz / gyroSensitivity;

      validReads++;
    }

    delay(2);
  }

  if (validReads == 0) {
    return false;
  }

  _gyroOffsetDPS[0] = sumX / validReads;
  _gyroOffsetDPS[1] = sumY / validReads;
  _gyroOffsetDPS[2] = sumZ / validReads;

  return true;
}


void MioMPU6050::setGyroOffset(float x, float y, float z) {
  _gyroOffsetDPS[0] = x;
  _gyroOffsetDPS[1] = y;
  _gyroOffsetDPS[2] = z;
}


void MioMPU6050::getGyroOffset(float &x, float &y, float &z) {
  x = _gyroOffsetDPS[0];
  y = _gyroOffsetDPS[1];
  z = _gyroOffsetDPS[2];
}


void MioMPU6050::setAccelOffset(float x, float y, float z) {
  _accelOffsetG[0] = x;
  _accelOffsetG[1] = y;
  _accelOffsetG[2] = z;
}


void MioMPU6050::getAccelOffset(float &x, float &y, float &z) {
  x = _accelOffsetG[0];
  y = _accelOffsetG[1];
  z = _accelOffsetG[2];
}



bool MioMPU6050::reset() {
    // Reset device
    if (!writeRegister(MPU6050_REG_PWR_MGMT_1, 0x80)) {
        return false;
    }

    delay(500);

    // Wake e clock PLL X gyro
    if (!writeRegister(MPU6050_REG_PWR_MGMT_1, 0x01)) {
        return false;
    }

    delay(10);

    return true;
}


int MioMPU6050::begin(bool initWire) {
    if (initWire) {
        Wire.begin();
    }

    Wire.setClock(400000);

    if (!reset()) {
        return 1;
    }

    if (!checkWhoAmI()) {
        return 2;
    }

    // Filtro digitale passa-basso
    // 3 = circa 42 Hz gyro, 44 Hz accel
    if (!setDLPF(3)) {
        return 3;
    }

    // Sample rate divider
    // Con DLPF attivo: 1 kHz / (1 + 3) = 250 Hz
    if (!setSampleRateDivider(3)) {
        return 4;
    }

    // Gyro ±250 °/s
    if (!setGyroRange(0)) {
        return 5;
    }

    // Accelerometro ±2 g
    if (!setAccelRange(0)) {
        return 6;
    }

    return 0;
}


