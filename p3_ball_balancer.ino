#define MOTOR_PIN 9
#define SENSOR_PIN A0

#define SERVO_MIN 1400
#define SERVO_MID 1500
#define SERVO_MAX 1600

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

int targetPos = 300;

double Kp = 0.17;
double Ki = 0.00001;
double Kd = 0.01;

uint32_t previousTime = 0;
int previousError = 0;
double integral = 0.0;

// every 10 ms = 10000 microseconds
double dt = 0.01;
#define PID_INTERVAL_US 10000

void setup() {
    Serial.begin(9600);

    servo.attach(MOTOR_PIN);

    //#define TEST_SERVO

    #ifdef TEST_SERVO
    Serial.println("servo test min");
    servo.writeMicroseconds(SERVO_MIN);
    delay(5000);
    Serial.println("servo test mid");
    servo.writeMicroseconds(SERVO_MID);
    delay(5000);
    Serial.println("servo test max");
    servo.writeMicroseconds(SERVO_MAX);
    delay(5000);
    #endif

    Serial.println("center");

    servo.writeMicroseconds(SERVO_MID);
    delay(3000);

    Serial.println("setup");

    pid_init();
}

void pid_init() {
    previousTime = micros()+PID_INTERVAL_US;
}

void loop() {
    // 1. Process serial input, alter balancing position if necessary
    process_serial();

    // 2. At regular intervals, update PID
    if(micros() > previousTime){
        previousTime = micros()+PID_INTERVAL_US;
        PID();
    }
}

void process_serial(){
    if(Serial.available() > 0){
        // https://docs.arduino.cc/language-reference/en/functions/communication/serial/parseInt/
        int readInt = Serial.parseInt();
        int targetPos = readInt;
        if(targetPos >= POS_MIN && targetPos <= POS_MAX){
            Serial.print("Target set: ");
            Serial.println(readInt);
        }else{
            Serial.println("ERROR");
        }

        while(Serial.available() > 0) Serial.read();
    }
}

int readSensor(){
    long sum = 0;
    for(int i = 0; i < 10; i++){
        sum += analogRead(SENSOR_PIN);
    }
    return (int)(sum / 10);
}

long lastPrint = 0;

int lastServoOutput = SERVO_MID;

void PID(){
    int sensorPos = readSensor();

    int error = targetPos - sensorPos; // calculate error/proportional

    // https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller (pseudocode)
    integral = integral + error * dt;  // calculate integral
    if(integral > INTEGRAL_MAX) integral = INTEGRAL_MAX;
    if(integral < INTEGRAL_MIN) integral = INTEGRAL_MIN;

    //double rawDerivative = -(sensorPos - previousSensor) / dt;
    //previousSensor = sensorPos;

    double rawDerivative = (error - previousError) / dt;
    previousError = error;

    double filteredDerivative = 0;
    double alpha = 0.1;   // tune 0.05–0.3

    filteredDerivative = filteredDerivative + alpha*(rawDerivative - filteredDerivative);

    double output = Kp * error + Ki * integral + Kd * filteredDerivative;

    int servoOutput = (int)((double)(SERVO_MID) + output);

    if(servoOutput < SERVO_MIN) servoOutput = SERVO_MIN;
    if(servoOutput > SERVO_MAX) servoOutput = SERVO_MAX;

    if(servoOutput < lastServoOutput-15) servoOutput = lastServoOutput-15;
    if(servoOutput > lastServoOutput+15) servoOutput = lastServoOutput+15;

    servo.writeMicroseconds(servoOutput);
    lastServoOutput = servoOutput;

    if(millis() > lastPrint){
        Serial.print(sensorPos);
        Serial.print(" ");
        Serial.println(servoOutput);
        lastPrint = millis()+200;
    }
}
