#define MOTOR_PIN 9
#define SENSOR_PIN A0

#define SERVO_MIN 1200
#define SERVO_MID 1500
#define SERVO_MAX 1800

#define INTEGRAL_MIN -100000
#define INTEGRAL_MAX 100000

#define POS_MIN 0
#define POS_MAX 1023

// https://docs.arduino.cc/learn/electronics/servo-motors/
#include <Servo.h>

// Process serial input. 
// When sending an integer position (e.g. "10"), the balancer should adjust the ball position
void process_serial();

// Process PID control.
// Same rules for dt and regular timings as Project 2
void PID(); 
void pid_init();

Servo servo;

int targetPos = 500;

double Kp = 0.1;
double Ki = 0.0;
double Kd = 0.0;

uint32_t previousTime = 0;
int previousError = 0;
double integral = 0.0;

void setup() {
    Serial.begin(9600);

    servo.attach(MOTOR_PIN);

    #define TEST_SERVO

    #ifdef TEST_SERVO
    servo.writeMicroseconds(SERVO_MID);
    delay(3000);
    servo.writeMicroseconds(SERVO_MIN);
    delay(3000);
    servo.writeMicroseconds(SERVO_MAX);
    #endif

    Serial.println("setup");

    pid_init();
}

void pid_init() {
    previousTime = micros();
}

void loop() {
    // 1. Process serial input, alter balancing position if necessary
    process_serial();

    // 2. At regular intervals, update PID
    PID();
}

void process_serial(){
    if(Serial.available() > 0){
        // https://docs.arduino.cc/language-reference/en/functions/communication/serial/parseInt/
        int readInt = Serial.parseInt();
        if(readInt >= POS_MIN && readInt <= POS_MAX){
            targetPos = readInt;
            Serial.print("Target set: ");
            Serial.println(readInt);
        }else{
            Serial.println("ERROR");
        }
    }
}

void PID(){
    int sensorPos = analogRead(SENSOR_PIN);

    int error = targetPos - sensorPos; // calculate error/proportional

    // calculate dt
    uint32_t currentTime = micros();  // https://docs.arduino.cc/language-reference/en/functions/time/micros/
    double dt = (double)(currentTime - previousTime) / 1000000.0;
    if(dt <= 0) dt = 0.000001;
    previousTime = currentTime;

    // https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller (pseudocode)
    integral = integral + error * dt;  // calculate integral
    if(integral > INTEGRAL_MAX) integral = INTEGRAL_MAX;
    if(integral < INTEGRAL_MIN) integral = INTEGRAL_MIN;

    double derivative = (error - previousError) / dt;  // calculate derivative
    previousError = error;

    double output = Kp * error + Ki * integral + Kd * derivative;

    int servoOutput = (int)((double)(SERVO_MID) + output);

    if(servoOutput < SERVO_MIN) servoOutput = SERVO_MIN;
    if(servoOutput > SERVO_MAX) servoOutput = SERVO_MAX;

    servo.writeMicroseconds(servoOutput);
}
