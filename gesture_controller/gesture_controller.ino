// Wearable ESP32 sketch: MPU6050 -> I2C -> threshold decisions -> XBee UART.
#include <Arduino.h>
#include <Wire.h>
#include <cstddef>
#include <cstdint>
#include "ControlLogic.h"

// Wearable ESP32 pin assignments.
constexpr int IMU_SDA = 21;
constexpr int IMU_SCL = 22;
constexpr std::uint8_t IMU_ADDRESS = 0x68; // AD0 LOW; use 0x69 if AD0 is HIGH
constexpr int XBEE_RX = 16;
constexpr int XBEE_TX = 17;
constexpr unsigned long XBEE_BAUD = 9600;
constexpr unsigned long SAMPLE_INTERVAL_MS = 80;
constexpr unsigned long SENSOR_RETRY_MS = 1000;
constexpr float DEADZONE_G = 0.30f;
constexpr int FORWARD_AXIS = 0; // 0 = X; 1 = Y. Right uses the other axis.
constexpr float FORWARD_SIGN = 1.0f;
constexpr float RIGHT_SIGN = 1.0f;
static_assert(FORWARD_AXIS == 0 || FORWARD_AXIS == 1, "Choose X or Y for the forward axis");


struct Acceleration { float xG; float yG; float zG; };

// Minimal register-level MPU6050 driver; no UART and no external IMU library.
class Mpu6050 {
public:
    explicit Mpu6050(TwoWire &bus, std::uint8_t address) : bus_(bus), address_(address) {}
    bool begin() {
        std::uint8_t identity = 0;
        if (!readBytes(0x75, &identity, 1) || identity != 0x68) return false;
        if (!writeRegister(0x6B, 0x01)) return false; // Wake, PLL with X gyro reference
        delay(50);
        return writeRegister(0x1C, 0x00); // Accelerometer +/-2 g
    }
    bool readAcceleration(Acceleration &out) {
        std::uint8_t bytes[6];
        if (!readBytes(0x3B, bytes, sizeof(bytes))) return false;
        constexpr float scale = 16384.0f;
        out = {signedWord(bytes[0], bytes[1]) / scale,
               signedWord(bytes[2], bytes[3]) / scale,
               signedWord(bytes[4], bytes[5]) / scale};
        return true;
    }
private:
    static std::int16_t signedWord(std::uint8_t high, std::uint8_t low) {
        return static_cast<std::int16_t>((static_cast<std::uint16_t>(high) << 8) | low);
    }
    bool writeRegister(std::uint8_t reg, std::uint8_t value) {
        bus_.beginTransmission(address_); bus_.write(reg); bus_.write(value);
        return bus_.endTransmission(true) == 0;
    }
    bool readBytes(std::uint8_t reg, std::uint8_t *destination, std::size_t count) {
        bus_.beginTransmission(address_); bus_.write(reg);
        if (bus_.endTransmission(false) != 0) return false;
        if (bus_.requestFrom(address_, count, true) != count) {
            while (bus_.available()) bus_.read();
            return false;
        }
        for (std::size_t i = 0; i < count; ++i) destination[i] = bus_.read();
        return true;
    }
    TwoWire &bus_;
    std::uint8_t address_;
};

HardwareSerial xbee(2);
Mpu6050 imu(Wire, IMU_ADDRESS);
bool sensorReady = false;
unsigned long lastSample = 0;
unsigned long lastRetry = 0;
unsigned long lastLog = 0;

void setup() {
    Serial.begin(115200);
    xbee.begin(XBEE_BAUD, SERIAL_8N1, XBEE_RX, XBEE_TX);
    Wire.begin(IMU_SDA, IMU_SCL);
    Wire.setClock(100000);
    Wire.setTimeOut(20);
    sensorReady = imu.begin();
    if (!sensorReady) Serial.println("MPU6050 missing: STOP packets only until recovery.");
}
void loop() {
    const unsigned long now = millis();
    if (static_cast<unsigned long>(now - lastSample) < SAMPLE_INTERVAL_MS) return;
    lastSample = now;
    robot::Motion motion = robot::Motion::Stop;
    Acceleration acceleration = {0, 0, 0};
    if (!sensorReady && static_cast<unsigned long>(now - lastRetry) >= SENSOR_RETRY_MS) {
        lastRetry = now;
        sensorReady = imu.begin();
    }
    if (sensorReady) {
        sensorReady = imu.readAcceleration(acceleration);
        if (sensorReady) {
            const float forward = (FORWARD_AXIS == 0 ? acceleration.xG : acceleration.yG) * FORWARD_SIGN;
            const float right = (FORWARD_AXIS == 0 ? acceleration.yG : acceleration.xG) * RIGHT_SIGN;
            motion = robot::classifyTilt(forward, right, DEADZONE_G);
        }
    }
    // UART -> local XBee -> wireless RF -> vehicle XBee -> vehicle UART.
    xbee.write(static_cast<std::uint8_t>(robot::encodeCommand(motion)));
    xbee.write('\n');
    if (static_cast<unsigned long>(now - lastLog) >= 800) {
        lastLog = now;
        Serial.printf("IMU:%s X:%.2fg Y:%.2fg Z:%.2fg command:%c\n",
                      sensorReady ? "OK" : "FAIL", acceleration.xG,
                      acceleration.yG, acceleration.zG, robot::encodeCommand(motion));
    }
}

