#include <Arduino.h>
#include <SPI.h>

#include <RF24.h>

// nRF24L01 on Arduino Uno: CE and CSN are configurable; SPI uses D11/D12/D13.
static const uint8_t RADIO_CE_PIN = 7;
static const uint8_t RADIO_CSN_PIN = 8;
static const uint8_t ARM_SWITCH_PIN = 4;

// Joystick mapping:
// left X = yaw, left Y = throttle, right X = roll, right Y = pitch.
static const uint8_t JOYSTICK_LEFT_X_PIN = A0;
static const uint8_t JOYSTICK_LEFT_Y_PIN = A1;
static const uint8_t JOYSTICK_RIGHT_X_PIN = A2;
static const uint8_t JOYSTICK_RIGHT_Y_PIN = A3;

static const uint32_t SEND_PERIOD_MS = 20UL;  // 50 Hz
static const int16_t MAX_ANGLE_CDEG = 3000;  // +/-30 degrees
static const int16_t MAX_YAW_RATE_CDEG = 9000;  // +/-90 degrees/second
static const uint8_t JOYSTICK_DEADBAND = 20;
static const uint8_t CENTER_SAMPLE_COUNT = 50;

static const uint16_t MIN_THROTTLE_US = 1000;
static const uint16_t MAX_THROTTLE_US = 2000;

// This structure must remain identical to the one in sketch1_FlightController.ino.
struct RcCommand {
    int16_t rollAngleCdeg;
    int16_t pitchAngleCdeg;
    int16_t yawRateCdeg;
    uint16_t throttleUs;
    uint8_t armed;
    uint8_t reserved;
};

static_assert(sizeof(RcCommand) == 10, "Unexpected RcCommand size");

RF24 radio(RADIO_CE_PIN, RADIO_CSN_PIN);
const uint8_t RADIO_ADDRESS[6] = "DRONE";

uint32_t lastSendMs = 0;
uint32_t lastStatusMs = 0;
bool armedState = false;
int joystickYawCenter = 512;
int joystickRollCenter = 512;
int joystickPitchCenter = 512;

int16_t centeredAxisToCommand(int rawValue, int center, int16_t maximumCommand, bool invert) {
    int centeredValue = rawValue - center;

    if (abs(centeredValue) <= JOYSTICK_DEADBAND) {
        return 0;
    }

    if (centeredValue > 0) {
        centeredValue -= JOYSTICK_DEADBAND;
    } else {
        centeredValue += JOYSTICK_DEADBAND;
    }

    const int usableRange = 511 - JOYSTICK_DEADBAND;
    int16_t commandValue = (int16_t)((long)centeredValue * maximumCommand / usableRange);
    commandValue = constrain(commandValue, -maximumCommand, maximumCommand);

    return invert ? -commandValue : commandValue;
}

uint16_t throttleFromJoystick(int rawValue) {
    const long throttle = map(rawValue, 0, 1023, MIN_THROTTLE_US, MAX_THROTTLE_US);
    return (uint16_t)constrain(throttle, (long)MIN_THROTTLE_US, (long)MAX_THROTTLE_US);
}

RcCommand readControls() {
    RcCommand nextCommand = {};

    const int leftX = analogRead(JOYSTICK_LEFT_X_PIN);
    const int leftY = analogRead(JOYSTICK_LEFT_Y_PIN);
    const int rightX = analogRead(JOYSTICK_RIGHT_X_PIN);
    const int rightY = analogRead(JOYSTICK_RIGHT_Y_PIN);

    nextCommand.yawRateCdeg = centeredAxisToCommand(leftX, joystickYawCenter, MAX_YAW_RATE_CDEG, false);
    nextCommand.rollAngleCdeg = centeredAxisToCommand(rightX, joystickRollCenter, MAX_ANGLE_CDEG, false);

    // With a typical joystick, moving the stick upward lowers the ADC value.
    nextCommand.pitchAngleCdeg = centeredAxisToCommand(rightY, joystickPitchCenter, MAX_ANGLE_CDEG, true);
    nextCommand.throttleUs = throttleFromJoystick(leftY);

    // Arm only after the switch is enabled with throttle low. Once armed,
    // keep the state latched so raising throttle does not disarm the FC.
    const bool armSwitchOn = digitalRead(ARM_SWITCH_PIN) == LOW;
    const bool throttleLow = nextCommand.throttleUs <= 1050;
    if (!armSwitchOn) {
        armedState = false;
    } else if (!armedState && throttleLow) {
        armedState = true;
    }
    nextCommand.armed = armedState ? 1 : 0;
    nextCommand.reserved = 0;

    return nextCommand;
}

void calibrateJoystickCenters() {
    long yawSum = 0;
    long rollSum = 0;
    long pitchSum = 0;

    for (uint8_t sample = 0; sample < CENTER_SAMPLE_COUNT; sample++) {
        yawSum += analogRead(JOYSTICK_LEFT_X_PIN);
        rollSum += analogRead(JOYSTICK_RIGHT_X_PIN);
        pitchSum += analogRead(JOYSTICK_RIGHT_Y_PIN);
        delay(5);
    }

    joystickYawCenter = yawSum / CENTER_SAMPLE_COUNT;
    joystickRollCenter = rollSum / CENTER_SAMPLE_COUNT;
    joystickPitchCenter = pitchSum / CENTER_SAMPLE_COUNT;
}

void printStatus(const RcCommand &command, bool transmitted) {
    if (millis() - lastStatusMs < 500) return;
    lastStatusMs = millis();

    Serial.print(F("armed="));
    Serial.print(command.armed);
    Serial.print(F(" throttle="));
    Serial.print(command.throttleUs);
    Serial.print(F(" roll="));
    Serial.print(command.rollAngleCdeg);
    Serial.print(F(" pitch="));
    Serial.print(command.pitchAngleCdeg);
    Serial.print(F(" yaw="));
    Serial.print(command.yawRateCdeg);
    Serial.print(F(" radio="));
    Serial.println(transmitted ? F("OK") : F("FAIL"));
}

void setup() {
    Serial.begin(115200);
    pinMode(ARM_SWITCH_PIN, INPUT_PULLUP);
    calibrateJoystickCenters();

    if (!radio.begin()) {
        Serial.println(F("RF24 not detected"));
        while (true) {
            delay(1000);
        }
    }

    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(108);
    radio.setAutoAck(true);
    radio.setRetries(2, 5);
    radio.setPayloadSize(sizeof(RcCommand));
    radio.openWritingPipe(RADIO_ADDRESS);
    radio.stopListening();

    Serial.println(F("RC controller ready"));
    Serial.println(F("Arm switch: D4 to GND, only with throttle low"));
    Serial.print(F("Centers yaw/roll/pitch: "));
    Serial.print(joystickYawCenter);
    Serial.print('/');
    Serial.print(joystickRollCenter);
    Serial.print('/');
    Serial.println(joystickPitchCenter);
}

void loop() {
    if (millis() - lastSendMs < SEND_PERIOD_MS) return;
    lastSendMs = millis();

    const RcCommand command = readControls();
    const bool transmitted = radio.write(&command, sizeof(command));
    printStatus(command, transmitted);
}