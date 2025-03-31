#include <Wire.h>
#include "MPU6050.h"

MPU6050 mpu;

// Motor pins
const int motor1PWM = 2, motor2PWM = 5, motor3PWM = 19, motor4PWM = 13;
const int motor1DIR = 15, motor2DIR = 4, motor3DIR = 18, motor4DIR = 12;

const int motorPWM[4] = {motor1PWM, motor2PWM, motor3PWM, motor4PWM};  
const int motorDIR[4] = {motor1DIR, motor2DIR, motor3DIR, motor4DIR}; 

// Motor direction control (1 for normal, -1 for reverse)
int motorDirection[4] = {-1, 1, 1, -1};

// PID Parameters (Tune these!)
float Kp_pitch = 15.0, Ki_pitch = 0.0, Kd_pitch = 3.5;
float Kp_roll = 15.0, Ki_roll = 0.0, Kd_roll = 3.5;

// PID variables
float errorPitch, prevErrorPitch = 0, integralPitch = 0;
float errorRoll, prevErrorRoll = 0, integralRoll = 0;
float derivativePitch, derivativeRoll;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    mpu.initialize();

    for (int i = 0; i < 4; i++) {
        pinMode(motorPWM[i], OUTPUT);
        pinMode(motorDIR[i], OUTPUT);
    }
}

void loop() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float accelPitch = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
    float accelRoll = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
    float gyroPitchRate = gx / 131.0;
    float gyroRollRate = gy / 131.0;

    // PID Control for pitch
    errorPitch = 0 - accelPitch;
    integralPitch += errorPitch;
    derivativePitch = errorPitch - prevErrorPitch;
    float outputPitch = Kp_pitch * errorPitch + Ki_pitch * integralPitch + Kd_pitch * derivativePitch;
    prevErrorPitch = errorPitch;

    // PID Control for roll
    errorRoll = 0 - accelRoll;
    integralRoll += errorRoll;
    derivativeRoll = errorRoll - prevErrorRoll;
    float outputRoll = Kp_roll * errorRoll + Ki_roll * integralRoll + Kd_roll * derivativeRoll;
    prevErrorRoll = errorRoll;

    // Calculate motor speeds based on new motor configuration
    int motorSpeeds[4];
    motorSpeeds[0] = motorDirection[0] * outputPitch;  // Motor 1 (Positive Y)
    motorSpeeds[1] = motorDirection[1] * -outputRoll; // Motor 2 (Negative X)
    motorSpeeds[2] = motorDirection[2] * outputRoll;  // Motor 3 (Positive X)
    motorSpeeds[3] = motorDirection[3] * -outputPitch; // Motor 4 (Negative Y)

    // Apply motor control
    for (int i = 0; i < 4; i++) {
        setMotor(i, motorSpeeds[i]);
    }

    Serial.print("Pi"); Serial.print(accelPitch);Serial.print(",");
    Serial.print("Rl"); Serial.println(accelRoll);

    delay(10);  // 10ms loop
}

// Motor Control Function
void setMotor(int index, int speed) {
    int pwmValue = constrain(abs(speed), 0, 255);
    digitalWrite(motorDIR[index], (speed >= 0) ? HIGH : LOW);
    analogWrite(motorPWM[index], pwmValue);
}
