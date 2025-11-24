#define MOTOR_PIN 9
#define SENSOR_PIN A0

// https://docs.arduino.cc/learn/electronics/servo-motors/
#include <Servo.h>

// Process serial input. 
// When sending an integer position (e.g. "10"), the balancer should adjust the ball position
void process_serial();

// Process PID control.
// Same rules for dt and regular timings as Project 2
void PID(); 

Servo servo;

void setup() {
    Serial.begin(9600);

    servo.attach(MOTOR_PIN);
}

int targetPos = 500;

double Kp = 0.4;
double Ki = 0.00001;
double Kd = 0.01;

uint32_t previousTime = 0;
int previousError = 0;
double integral = 0.0;

void loop() {
    // 1. Process serial input, alter balancing position if necessary
    if(Serial.available() > 0){
      int readInt = Serial.parseInt();
    }

    // 2. At regular intervals, update PID
    int sensorPos = analogRead(SENSOR_PIN);

    int error = targetPos - sensorPos; // calculate error/proportional

    // calculate dt
    uint32_t currentTime = micros();  // https://docs.arduino.cc/language-reference/en/functions/time/micros/
    double dt = (double)(currentTime - previousTime) / 1000000.0;
    previousTime = currentTime;

    // https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller (pseudocode)
    integral = integral + error * dt;  // calculate integral

    double derivative = (error - previousError) / dt;  // calculate derivative
    previousError = error;

    double output = Kp * error + Ki * integral + Kd * derivative;

    //Servo.writeMicroseconds();
}
