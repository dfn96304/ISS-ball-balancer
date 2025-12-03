#define MOTOR_PIN 9
#define SENSOR_PIN A0

#define SERVO_MIN 0
#define SERVO_MID 85
#define SERVO_MAX 180

#define INTEGRAL_MIN -100000
#define INTEGRAL_MAX 100000

#define POS_MIN -20
#define POS_MAX 20

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

int targetPos = 0;

double Kp = 0.6;
double Ki = 0.0;
double Kd = 0.2;

uint32_t previousTime = 0;
int previousError = 0;
double integral = 0.0;

// every 10 ms = 10000 microseconds
double dt = 0.02;
#define PID_INTERVAL_US 20000

void setup() {
    Serial.begin(9600);

    servo.attach(MOTOR_PIN);

    //#define TEST_SERVO

    #ifdef TEST_SERVO
    Serial.println("servo test min");
    servo.write(SERVO_MIN);
    delay(3000);
    Serial.println("servo test mid");
    servo.write(SERVO_MID);
    delay(3000);
    Serial.println("servo test max");
    servo.write(SERVO_MAX);
    delay(3000);
    #endif

    Serial.println("center");

    servo.write(SERVO_MID);
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

    int time = micros();

    // 2. At regular intervals, update PID
    if(time > previousTime+PID_INTERVAL_US){
        previousTime = time;
        PID();
    }
}

void process_serial(){
    if(Serial.available() > 0){
        // https://docs.arduino.cc/language-reference/en/functions/communication/serial/parseInt/
        int readInt = Serial.parseInt();
        targetPos = readInt;
        if(targetPos >= POS_MIN && targetPos <= POS_MAX){
            Serial.print("Target set: ");
            Serial.println(targetPos);
        }else{
            Serial.println("ERROR");
        }

        while(Serial.available() > 0) Serial.read();
    }
}

double readSensor(){
    long sum = 0;
    for(int i = 0; i < 20; i++){
        sum += analogRead(SENSOR_PIN);
    }
    double average = (double)sum / 20.0;
    return 18600 * pow(average, -1.162602) - 36.0;
}

long lastPrint = 0;

int lastServoOutput = SERVO_MID;

void PID(){
    double sensorPos = readSensor();

    double error = targetPos - sensorPos; // calculate error/proportional

    // https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller (pseudocode)
    integral = integral + error * dt;  // calculate integral
    if(integral > INTEGRAL_MAX) integral = INTEGRAL_MAX;
    if(integral < INTEGRAL_MIN) integral = INTEGRAL_MIN;

    //double rawDerivative = -(sensorPos - previousSensor) / dt;
    //previousSensor = sensorPos;

    double rawDerivative = (error - previousError) / dt;
    previousError = error;

    /*double filteredDerivative = 0;
    double alpha = 0.1;   // tune 0.05–0.3

    filteredDerivative = filteredDerivative + alpha*(rawDerivative - filteredDerivative);*/

    double output = Kp * error + Ki * integral + Kd * rawDerivative;

    int servoOutput = (int)((double)(SERVO_MID) - output);

    if(servoOutput < SERVO_MIN) servoOutput = SERVO_MIN;
    if(servoOutput > SERVO_MAX) servoOutput = SERVO_MAX;

    //if(servoOutput < lastServoOutput-MAX_SERVO_MOVE) servoOutput = lastServoOutput-MAX_SERVO_MOVE;
    //if(servoOutput > lastServoOutput+MAX_SERVO_MOVE) servoOutput = lastServoOutput+MAX_SERVO_MOVE;

    servo.write(servoOutput);
    lastServoOutput = servoOutput;

    if(millis() > lastPrint){
        Serial.print(sensorPos);
        Serial.print(" ");
        Serial.print(targetPos);
        Serial.print(" ");
        Serial.println(servoOutput);
        lastPrint = millis()+200;
    }
}
