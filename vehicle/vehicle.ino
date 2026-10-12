// Vehicle ESP32 sketch: physical mode button, XBee UART, HTTP/TCP and L298N.
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "ControlLogic.h"
#include "WebPage.h"

#include <IPAddress.h>
// Vehicle ESP32 pin assignments.
constexpr int LEFT_IN1 = 25;   // L298N IN1
constexpr int LEFT_IN2 = 26;   // L298N IN2
constexpr int RIGHT_IN1 = 27;  // L298N IN3
constexpr int RIGHT_IN2 = 33;  // L298N IN4
constexpr int LEFT_ENABLE = 18;  // L298N ENA
constexpr int RIGHT_ENABLE = 19; // L298N ENB
constexpr bool INVERT_LEFT = false;
constexpr bool INVERT_RIGHT = false;
constexpr int MODE_BUTTON = 32; // Robot-mounted button between this pin and GND
constexpr int MODE_LED = 23;    // HIGH = Gesture, LOW = Wi-Fi
constexpr unsigned long BUTTON_DEBOUNCE_MS = 40;
constexpr int XBEE_RX = 16; // ESP RX <- XBee S2C DOUT, through carrier breakout
constexpr int XBEE_TX = 17; // ESP TX -> XBee S2C DIN, through carrier breakout
constexpr unsigned long XBEE_BAUD = 9600;
constexpr unsigned long COMMAND_TIMEOUT_MS = 500;
constexpr unsigned long FRAME_TIMEOUT_MS = 150;
constexpr const char *AP_NAME = "DualModeRobot";
constexpr const char *AP_PASSWORD = "robotcar2026";
const IPAddress AP_IP(192, 168, 4, 1);
const IPAddress AP_SUBNET(255, 255, 255, 0);


// Fixed on/off motor control using L298N direction and enable pins.
class L298NMotorDriver {
public:
    bool begin() {
        const int pins[] = {LEFT_IN1, LEFT_IN2, RIGHT_IN1, RIGHT_IN2,
                            LEFT_ENABLE, RIGHT_ENABLE};
        for (int pin : pins) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
        ready_ = true;
        return true;
    }
    void apply(robot::Motion motion) {
        if (!ready_ || (hasPrevious_ && motion == previous_)) return;
        hasPrevious_ = true;
        previous_ = motion;
        enable(true, false); enable(false, false);
        digitalWrite(LEFT_IN1, LOW); digitalWrite(LEFT_IN2, LOW);
        digitalWrite(RIGHT_IN1, LOW); digitalWrite(RIGHT_IN2, LOW);
        delayMicroseconds(100);
        const auto wheels = robot::wheelDirections(motion);
        direction(LEFT_IN1, LEFT_IN2, wheels.left, INVERT_LEFT);
        direction(RIGHT_IN1, RIGHT_IN2, wheels.right, INVERT_RIGHT);
        enable(true, wheels.left != 0);
        enable(false, wheels.right != 0);
    }
private:
    static void enable(bool left, bool active) {
        digitalWrite(left ? LEFT_ENABLE : RIGHT_ENABLE, active ? HIGH : LOW);
    }
    static void direction(int in1, int in2, int value, bool inverted) {
        if (inverted) value = -value;
        digitalWrite(in1, value > 0 ? HIGH : LOW);
        digitalWrite(in2, value < 0 ? HIGH : LOW);
    }
    bool ready_ = false;
    bool hasPrevious_ = false;
    robot::Motion previous_ = robot::Motion::Stop;
};

WebServer server(80);
HardwareSerial xbee(2);
robot::Controller controller(COMMAND_TIMEOUT_MS);
robot::DebouncedButton modeButton(BUTTON_DEBOUNCE_MS);
robot::CommandParser parser;
L298NMotorDriver motors;
bool motorReady = false;
bool webReady = false;
unsigned long lastSerialByte = 0;

void sendState() {
    String body = "{\"mode\":\"";
    body += controller.mode() == robot::Mode::Web ? "web" : "gesture";
    body += "\",\"mode_version\":" + String(controller.modeVersion());
    body += ",\"command\":\"";
    body += robot::encodeCommand(controller.motion());
    body += "\",\"halted\":";
    body += controller.halted() ? "true" : "false";
    body += ",\"needs_neutral\":";
    body += controller.needsNeutral() ? "true" : "false";
    body += "}";
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", body);
}
void handleCommand() {
    robot::Motion motion;
    const String value = server.arg("value");
    if (value.length() != 1 || !robot::decodeCommand(value[0], motion)) {
        server.send(400, "text/plain", "Expected F, B, L, R or S");
        return;
    }
    if (!motorReady) {
        server.send(503, "text/plain", "Motor driver initialization failed");
        return;
    }
    // Reject requests belonging to a previous physical mode selection.
    if (server.arg("mode_version") != String(controller.modeVersion()) ||
        !controller.accept(robot::Mode::Web, motion, millis())) {
        server.send(409, "text/plain", "Use the physical button to select Web mode; STOP may be latched");
        return;
    }
    motors.apply(controller.motion());
    sendState();
}
void handleStop() {
    controller.halt();
    motors.apply(robot::Motion::Stop);
    sendState();
}
void pollModeButton() {
    if (!modeButton.update(digitalRead(MODE_BUTTON) == LOW, millis())) return;
    controller.toggleMode();
    motors.apply(robot::Motion::Stop);
    // Do not replay buffered packets from before the mode change.
    parser.reset();
    while (xbee.available()) xbee.read();
    lastSerialByte = millis();
    digitalWrite(MODE_LED, controller.mode() == robot::Mode::Gesture ? HIGH : LOW);
    Serial.println(controller.mode() == robot::Mode::Web ? "Mode: WEB" : "Mode: GESTURE; level the glove first");
}
void setup() {
    Serial.begin(115200);
    pinMode(MODE_BUTTON, INPUT_PULLUP);
    pinMode(MODE_LED, OUTPUT);
    digitalWrite(MODE_LED, LOW);
    motorReady = motors.begin();
    motors.apply(robot::Motion::Stop);
    if (!motorReady) Serial.println("Motor driver initialization failed; motor outputs stay disabled.");
    xbee.begin(XBEE_BAUD, SERIAL_8N1, XBEE_RX, XBEE_TX);
    WiFi.mode(WIFI_AP);
    webReady = WiFi.softAPConfig(AP_IP, AP_IP, AP_SUBNET) && WiFi.softAP(AP_NAME, AP_PASSWORD);
    if (!webReady) {
        Serial.println("Wi-Fi AP initialization failed; Gesture mode remains selectable.");
        return;
    }
    server.on("/", HTTP_GET, []() {
        server.sendHeader("Cache-Control", "no-store");
        server.send_P(200, "text/html; charset=utf-8", CONTROL_PAGE);
    });
    server.on("/command", HTTP_POST, handleCommand);
    server.on("/stop", HTTP_POST, handleStop);
    server.on("/state", HTTP_GET, sendState);
    // There is intentionally no /mode endpoint: mode selection is physical.
    server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
    server.begin();
    Serial.print("Connect to Wi-Fi: "); Serial.println(AP_NAME);
    Serial.print("Browser address: http://"); Serial.println(WiFi.softAPIP());
}
void loop() {
    pollModeButton();
    controller.tick(millis());
    motors.apply(controller.motion());
    if (webReady) server.handleClient();
    pollModeButton();
    if (static_cast<unsigned long>(millis() - lastSerialByte) >= FRAME_TIMEOUT_MS) parser.reset();
    robot::Motion received;
    // Bound serial work per iteration so the button and web server keep running.
    for (unsigned int count = 0; count < 64 && xbee.available(); ++count) {
        lastSerialByte = millis();
        if (parser.feed(static_cast<char>(xbee.read()), received) && motorReady)
            controller.accept(robot::Mode::Gesture, received, millis());
    }
    controller.tick(millis());
    motors.apply(controller.motion());
}

