# Ball Balancer - Arduino PID Control

This project implements a ball balancer using a PID control algorithm on an Arduino board. The goal of the project is to balance a ball at a desired position by adjusting the servo motor based on sensor readings.

## Hardware requirements

- **Arduino board**
- **Servo motor**
- **Sensor** (Sharp GP2Y)
- **Ball balancing mechanism** (custom setup or 3D printed)

## Pin configuration

- **Servo motor pin**: by default set to pin `9` (Connected to the servo motor)
- **Sensor pin**: Pin `A0` (Analog input from the ball position sensor)

## Code overview

- **`setup()`**: Initializes serial communication, attaches the servo, runs a basic servo test (optional), and sets up the PID controller.
- **`loop()`**: Continuously reads serial input (target position) and periodically updates the PID controller.
- **`process_serial()`**: Handles incoming serial input to adjust the target position for the ball to balance.
- **`PID()`**: Performs the PID control algorithm based on the sensor readings and updates the servo motor position.
- **`readSensor()`**: Averages multiple sensor readings, and applies corrective calculations appropriate for the non-linear infrared Sharp GP2Y sensor, to calculate the ball's position in cm.

## Calibration

The PID coefficients (`Kp`, `Ki`, and `Kd`) and the sensor readings may require calibration.

Additionally, the values inside `readSensor()`: `18600 * pow(average, -1.162602) - 36.0;` assume a setup with the Sharp GP2Y sensor on one side around 20 cm away from the center, and a ruler with a 0 cm marking in the exact center of the platform.

## Usage

1. Once the system is up and running, the target position of the ball can be set through the Serial Monitor. For example, sending the value `10` would attempt to balance the ball at position 10 cm away from 0 (center).
2. The servo motor will adjust accordingly based on the calculated PID output to maintain the target position.

### Serial commands

- **Target position**: Send an integer value (e.g., `10`) to set the target position of the ball.
- **Position limits**: The target position is constrained between `-20` and `20`.
- **Error handling**: Sending a value outside the allowable range will output an `ERROR` message.

## Testing Servo (Optional)

The code includes a servo testing feature that moves the servo to its minimum, mid, and maximum positions. You can enable this by uncommenting the `#define TEST_SERVO` line.

```cpp
#define TEST_SERVO
