#include <Wire.h>
#include <MPU6050_tockn.h>

MPU6050 mpu6050(Wire);

void setup() {
    Serial.begin(115200);
    Wire.begin();
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true); // Calibrate the gyro
}

void loop() {
    mpu6050.update();
    
    float accX = mpu6050.getAccAngleX(); // Roll angle
    float accY = mpu6050.getAccAngleY(); // Pitch angle
    float gyroZ = mpu6050.getGyroAngleZ(); // Yaw angle
    
    Serial.print("Roll: ");
    Serial.print(accX);
    Serial.print("°	Pitch: ");
    Serial.print(accY);
    Serial.print("°	Yaw: ");
    Serial.print(gyroZ);
    Serial.println("°");
    
    delay(100);
}
