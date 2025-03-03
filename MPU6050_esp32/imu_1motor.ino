#include <Wire.h>
#include <MPU6050.h>

MPU6050 imu;

const int motorPin1 = 18;  // L298 IN1
const int motorPin2 = 19;  // L298 IN2
const int enablePin = 5;   // L298 ENA (PWM for speed control)

void setup() {
    Serial.begin(115200);
    Wire.begin();
    imu.initialize();

    if (!imu.testConnection()) {
        Serial.println("IMU connection failed!");
        while (1);
    }

    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);
    pinMode(enablePin, OUTPUT);
}

void loop() {
    int16_t ax, ay, az, gx, gy, gz;
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    int mappedSpeed = map(ax, -17000, 17000, -255, 255);
    mappedSpeed = constrain(mappedSpeed, -255, 255);

    if (mappedSpeed > 0) {
        digitalWrite(motorPin1, HIGH);
        digitalWrite(motorPin2, LOW);
        analogWrite(enablePin, mappedSpeed);
    } else if (mappedSpeed < 0) {
        digitalWrite(motorPin1, LOW);
        digitalWrite(motorPin2, HIGH);
        analogWrite(enablePin, abs(mappedSpeed));
    } else {
        digitalWrite(motorPin1, LOW);
        digitalWrite(motorPin2, LOW);
        analogWrite(enablePin, 0);
    }

    Serial.print("X-axis: ");
    Serial.print(ax);
    Serial.print(" | Mapped Speed: ");
    Serial.println(mappedSpeed);

    delay(100);
}
