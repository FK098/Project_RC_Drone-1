#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include <RF24.h>
#include <MioMPU6050.h>
#include <MioBMP280.h>
#include <MioQMC5883L.h>
#include <MioMotore.h>

// Pin assegnati nel README: nRF24L01 CE=D7, CSN=D8.
static const uint8_t RADIO_CE_PIN = 7;
static const uint8_t RADIO_CSN_PIN = 8;
static const uint8_t MOTOR_1_PIN = 3;  // Front right, CCW
static const uint8_t MOTOR_2_PIN = 5;  // Rear right, CW
static const uint8_t MOTOR_3_PIN = 6;  // Rear left, CCW
static const uint8_t MOTOR_4_PIN = 9;  // Front left, CW

static const uint32_t CONTROL_PERIOD_US = 4000UL;  // 250 Hz
static const uint32_t RADIO_TIMEOUT_MS = 250UL;
static const int MIN_MOTOR_US = 1000;
static const int MAX_MOTOR_US = 2000;

// The transmitter must send this exact payload on the "DRONE" pipe.
struct RcCommand {
    int16_t rollAngleCdeg;   // desired roll angle, centidegrees
    int16_t pitchAngleCdeg;  // desired pitch angle, centidegrees
    int16_t yawRateCdeg;     // desired yaw rate, centidegrees/second
    uint16_t throttleUs;     // 1000..2000 microseconds
    uint8_t armed;
    uint8_t reserved;
};

static_assert(sizeof(RcCommand) <= 32, "nRF24 payload too large");

RF24 radio(RADIO_CE_PIN, RADIO_CSN_PIN);
const uint8_t RADIO_ADDRESS[6] = "DRONE";

MioMPU6050 imu;
MioBMP280 barometer;
MioQMC5883L compass;
MioMotore motor1;
MioMotore motor2;
MioMotore motor3;
MioMotore motor4;

RcCommand command = {};
uint32_t lastRadioPacketMs = 0;
uint32_t nextControlUs = 0;
bool radioReady = false;
bool imuReady = false;
bool barometerReady = false;
bool compassReady = false;
bool flightArmed = false;

float rollDeg = 0.0f;
float pitchDeg = 0.0f;

struct Pid {
    float kp;
    float ki;
    float kd;
    float integral;
    float previousMeasurement;
    float integralLimit;
    float outputLimit;
};

// These are starting values only. Tune them with propellers removed first.
Pid angleRoll = {4.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 120.0f};
Pid anglePitch = {4.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 120.0f};
Pid rateRoll = {0.8f, 0.35f, 0.02f, 0.0f, 0.0f, 100.0f, 400.0f};
Pid ratePitch = {0.8f, 0.35f, 0.02f, 0.0f, 0.0f, 100.0f, 400.0f};
Pid rateYaw = {1.2f, 0.25f, 0.0f, 0.0f, 0.0f, 100.0f, 300.0f};

float clampFloat(float value, float minimum, float maximum) {
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

float updatePid(Pid &pid, float setpoint, float measurement, float dt) {
    const float error = setpoint - measurement;
    pid.integral += error * dt;
    pid.integral = clampFloat(pid.integral, -pid.integralLimit, pid.integralLimit);

    // Derivative on measurement avoids a derivative kick when the stick moves.
    const float derivative = (measurement - pid.previousMeasurement) / dt;
    pid.previousMeasurement = measurement;

    const float output = pid.kp * error + pid.ki * pid.integral - pid.kd * derivative;
    return clampFloat(output, -pid.outputLimit, pid.outputLimit);
}

void resetPid(Pid &pid) {
    pid.integral = 0.0f;
    pid.previousMeasurement = 0.0f;
}

void resetControllers() {
    resetPid(angleRoll);
    resetPid(anglePitch);
    resetPid(rateRoll);
    resetPid(ratePitch);
    resetPid(rateYaw);
}

void stopMotors() {
    motor1.setRawPWM(MIN_MOTOR_US);
    motor2.setRawPWM(MIN_MOTOR_US);
    motor3.setRawPWM(MIN_MOTOR_US);
    motor4.setRawPWM(MIN_MOTOR_US);
}

void updateRadio() {
    if (!radioReady) return;

    while (radio.available()) {
        RcCommand received = {};
        const uint8_t payloadSize = radio.getPayloadSize();
        if (payloadSize != sizeof(RcCommand)) {
            uint8_t discarded[32];
            radio.read(discarded, payloadSize > sizeof(discarded) ? sizeof(discarded) : payloadSize);
            continue;
        }

        radio.read(&received, sizeof(received));
        command = received;
        command.throttleUs = constrain(command.throttleUs, MIN_MOTOR_US, MAX_MOTOR_US);
        lastRadioPacketMs = millis();
    }
}

bool readImu(float &gxDps, float &gyDps, float &gzDps) {
    float axG, ayG, azG, temperatureC;
    if (!imu.readScaled(axG, ayG, azG, temperatureC, gxDps, gyDps, gzDps)) {
        return false;
    }

    const float measuredRoll = atan2(ayG, azG) * RAD_TO_DEG;
    const float measuredPitch = atan2(-axG, sqrt(ayG * ayG + azG * azG)) * RAD_TO_DEG;
    const float dt = CONTROL_PERIOD_US / 1000000.0f;
    const float alpha = 0.98f;

    rollDeg = alpha * (rollDeg + gxDps * dt) + (1.0f - alpha) * measuredRoll;
    pitchDeg = alpha * (pitchDeg + gyDps * dt) + (1.0f - alpha) * measuredPitch;
    return true;
}

void writeMotorMix(float throttle, float rollCorrection, float pitchCorrection, float yawCorrection) {
    // X configuration. Verify signs and propeller directions on the real frame.
    const int motor1Us = (int)(throttle + pitchCorrection - rollCorrection - yawCorrection);
    const int motor2Us = (int)(throttle - pitchCorrection - rollCorrection + yawCorrection);
    const int motor3Us = (int)(throttle - pitchCorrection + rollCorrection - yawCorrection);
    const int motor4Us = (int)(throttle + pitchCorrection + rollCorrection + yawCorrection);

    motor1.setRawPWM(constrain(motor1Us, MIN_MOTOR_US, MAX_MOTOR_US));
    motor2.setRawPWM(constrain(motor2Us, MIN_MOTOR_US, MAX_MOTOR_US));
    motor3.setRawPWM(constrain(motor3Us, MIN_MOTOR_US, MAX_MOTOR_US));
    motor4.setRawPWM(constrain(motor4Us, MIN_MOTOR_US, MAX_MOTOR_US));
}

void controlStep() {
    float gxDps, gyDps, gzDps;
    if (!imuReady || !readImu(gxDps, gyDps, gzDps)) {
        flightArmed = false;
        resetControllers();
        stopMotors();
        return;
    }

    const bool radioTimedOut = millis() - lastRadioPacketMs > RADIO_TIMEOUT_MS;
    const bool throttleLow = command.throttleUs <= 1050;
    flightArmed = command.armed != 0 && !radioTimedOut;

    if (!flightArmed) {
        resetControllers();
        stopMotors();
        return;
    }

    const float dt = CONTROL_PERIOD_US / 1000000.0f;
    const float desiredRollDeg = command.rollAngleCdeg / 100.0f;
    const float desiredPitchDeg = command.pitchAngleCdeg / 100.0f;
    const float desiredRollRate = updatePid(angleRoll, desiredRollDeg, rollDeg, dt);
    const float desiredPitchRate = updatePid(anglePitch, desiredPitchDeg, pitchDeg, dt);
    const float desiredYawRate = command.yawRateCdeg / 100.0f;

    const float rollCorrection = updatePid(rateRoll, desiredRollRate, gxDps, dt);
    const float pitchCorrection = updatePid(ratePitch, desiredPitchRate, gyDps, dt);
    const float yawCorrection = updatePid(rateYaw, desiredYawRate, gzDps, dt);

    if (throttleLow) {
        resetControllers();
        stopMotors();
        return;
    }

    writeMotorMix(command.throttleUs, rollCorrection, pitchCorrection, yawCorrection);
}

void setupRadio() {
    radioReady = radio.begin();
    if (!radioReady) return;

    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(108);
    radio.setAutoAck(true);
    radio.setPayloadSize(sizeof(RcCommand));
    radio.openReadingPipe(1, RADIO_ADDRESS);
    radio.startListening();
}

void setup() {
    Serial.begin(115200);

    motor1.begin(MOTOR_1_PIN);
    motor2.begin(MOTOR_2_PIN);
    motor3.begin(MOTOR_3_PIN);
    motor4.begin(MOTOR_4_PIN);
    stopMotors();

    imuReady = imu.begin(true) == 0;
    if (imuReady) {
        // The frame must remain completely still during this calibration.
        imu.calibrateGyro(400);
    }

    // These devices are initialized for later altitude/heading features, but
    // neither is read inside the 250 Hz stabilization path.
    barometerReady = barometer.begin(false);
    compassReady = compass.begin(false);
    setupRadio();

    command.throttleUs = MIN_MOTOR_US;
    lastRadioPacketMs = millis();
    nextControlUs = micros() + CONTROL_PERIOD_US;

    Serial.println(F("RC drone controller ready; motors disarmed"));
    Serial.print(F("IMU: ")); Serial.println(imuReady ? F("OK") : F("FAIL"));
    Serial.print(F("BMP280: ")); Serial.println(barometerReady ? F("OK") : F("FAIL"));
    Serial.print(F("QMC5883L: ")); Serial.println(compassReady ? F("OK") : F("FAIL"));
    Serial.print(F("RF24: ")); Serial.println(radioReady ? F("OK") : F("FAIL"));
}

void loop() {
    updateRadio();

    const uint32_t nowUs = micros();
    if ((int32_t)(nowUs - nextControlUs) < 0) return;

    // Recover cleanly if a blocking I2C/radio operation made us miss a cycle.
    if ((uint32_t)(nowUs - nextControlUs) > CONTROL_PERIOD_US * 4UL) {
        nextControlUs = nowUs;
    }
    nextControlUs += CONTROL_PERIOD_US;
    controlStep();
}