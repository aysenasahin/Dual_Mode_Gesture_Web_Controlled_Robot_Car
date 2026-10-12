# Dual-Mode Gesture and Web-Controlled Robot Car

A robot car operated either with a wearable hand-motion controller or through a
local browser interface. Two ESP32 boards handle wearable sensing and vehicle
control. A pair of XBee S2C modules transfers gesture commands over a
point-to-point wireless link.

A physical button on the robot selects **Gesture** or **Wi-Fi** mode. The robot
uses commands from the selected source to drive two DC motors through an L298N.
A passive front wheel supports the chassis.

## System architecture

~~~mermaid
flowchart TD
    subgraph Wearable["Wearable controller"]
        IMU["MPU6050"] -->|"I2C acceleration readings"| ESP["Wearable ESP32: axis thresholds"]
        ESP -->|"UART via Explorer carrier"| TX["XBee S2C transmitter"]
    end
    TX -->|"Point-to-point wireless link"| RX["Vehicle XBee S2C"]
    RX -->|"UART movement commands"| SELECT
    BROWSER["Phone or computer browser"] -->|"Wi-Fi: HTTP over TCP"| SELECT
    subgraph Vehicle["Vehicle ESP32"]
        BUTTON["Robot-mounted button"] -->|"Choose Gesture or Wi-Fi"| SELECT{"Selected input source"}
        SELECT -->|"Accept commands from the active source"| CONTROL["Movement logic"]
        CONTROL -->|"Digital direction and enable signals"| DRIVER["L298N"]
        DRIVER --> MOTORS["Left and right DC motors"]
    end
~~~

## Gesture control: direct axis thresholds

The wearable ESP32 reads the MPU6050 X, Y and Z accelerometer measurements over
**I2C**. X and Y are used for direction decisions; Z remains available in the
sensor readout. These measurements describe acceleration along the sensor axes,
not the position of the hand in space.

The code compares the selected forward/reverse and left/right components with
positive and negative thresholds. Small readings inside the neutral region
produce STOP. A component must exceed its threshold to produce a movement
command. When both components exceed the threshold, the larger absolute
component determines the direction.

`DEADZONE_G` defines the threshold in the wearable sketch. The current value is
0.30 g; the axis selection and signs are also defined at the top of that sketch.
With the +/-2 g accelerometer range, raw readings are converted using 16384
counts/g. Each decision uses the current sensor reading directly.

The ESP32 sends one direction command every 80 ms. A failed I2C read produces
STOP instead of repeating a previous movement command.

## XBee communication

Each ESP32 exchanges serial data with its local XBee through **UART, 9600 baud,
8N1**. The radios carry the data between the wearable and vehicle over their
**P2P wireless link**. The firmware uses transparent serial operation.

XBee Explorer-style carrier boards provide the module socket, power interface
and access to the radio serial pins. The ESP32 data path uses XBee DIN/DOUT via
the carrier breakout; the carrier USB interface is a separate interface.

The transmitted data is a direction command, not a stream of raw IMU coordinates.
Each command is one uppercase character followed by a newline:

| Command | Meaning |
| --- | --- |
| F | Forward |
| B | Reverse |
| L | Left |
| R | Right |
| S | Stop / neutral |

The vehicle checks complete command frames before applying them. Invalid frames
are discarded. Gesture commands are ignored while Wi-Fi mode is selected.

## Wi-Fi browser control

The vehicle ESP32 creates a local Wi-Fi access point and serves an HTML/JavaScript
control page. A phone or computer browser sends HTTP requests directly to this
ESP32; TCP carries the requests over Wi-Fi. This path is independent of XBee.

The page is served at **http://192.168.4.1** on TCP port 80. This is the static
address assigned to the robot access-point interface in the sketch. It stays
the same with this configuration; client addresses can vary.

The page provides forward, reverse, left, right and STOP controls. In Wi-Fi mode,
holding a direction button sends repeated commands; releasing it sends STOP.
The page displays the mode selected by the physical robot button. It does not
change the mode itself.

## Physical mode button and motor control

A debounced press on the robot button switches the active command source.
Holding it does not cause repeated switches. A mode change stops the motors and
clears pending gesture frames. Gesture mode waits for a neutral command before
accepting movement. Commands from the inactive source are rejected.

The vehicle also stops after 500 ms without an accepted command. The browser
STOP control holds the vehicle stopped in either mode until a physical mode
button press. These checks are implemented in `ControlLogic.h`.

The L298N is controlled with digital direction and enable signals. Each motor
is either active or off; there is no variable-speed motor control.

| Movement | Left motor | Right motor |
| --- | --- | --- |
| Forward | Forward | Forward |
| Reverse | Reverse | Reverse |
| Turn left | Off | Forward |
| Turn right | Forward | Off |
| Stop | Off | Off |

The passive front wheel is not motor-driven. Turns are produced by driving only
one of the two rear wheels.

## Arduino IDE sketches

The firmware is written in C++ for ESP32 using the Arduino framework. Each board
has its own Arduino IDE sketch, with pin and timing constants at the top.

| File | Role |
| --- | --- |
| `gesture_controller/gesture_controller.ino` | MPU6050 I2C readout, thresholds and XBee UART transmission |
| `gesture_controller/ControlLogic.h` | Direction encoding and direct threshold comparisons |
| `vehicle/vehicle.ino` | Physical mode button, XBee reception, HTTP server and L298N signals |
| `vehicle/ControlLogic.h` | Frame parsing, mode selection, button debounce and command timeout |
| `vehicle/WebPage.h` | Browser controls written in HTML, CSS and JavaScript |
