#include <Wire.h>
#include <Servo.h>
#include "MPU6050.h"

// PID constants
double Kp = 1.0;
double Ki = 0.1;
double Kd = 0.0;

// Setpoint and measured angle
double setpoint = 10.0;  // desired angle in degrees
double angle = 0.0;      // current angle

// PID variables
double previousError = 0.0;
double integral = 0.0;
unsigned long previousTime = 0;

// Create MPU6050 object
MPU6050 mpu;

// Create a Servo object to control the bidirectional ESC
Servo esc;

// Pin to which the ESC signal is connected
const int escPin = 9;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize the MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while(1);
  }
  // Optionally configure MPU6050 settings here

  // Attach the ESC on pin 9
  esc.attach(escPin);

  // Arm the ESC: send neutral throttle (1500 µs for bidirectional ESC)
  esc.writeMicroseconds(1500);
  delay(2000);  // wait 2 seconds to arm the ESC

  // Initialize previousTime for time-based PID calculations
  previousTime = millis();

  Serial.println("ESC armed. Starting control loop...");
}

void loop() {
  // 1. Read raw accelerometer & gyro data
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // 2. Convert raw accelerometer data to angle (roll)
  //    Adjust axes if your MPU orientation differs
  angle = atan2((double)ay, (double)az) * RAD_TO_DEG;

  // 3. Calculate error between setpoint and current angle
  double error = setpoint - angle;

  // 4. Compute time elapsed since last loop (in seconds)
  unsigned long currentTime = millis();
  double dt = (currentTime - previousTime) / 1000.0;
  if (dt <= 0) dt = 0.001;  // Prevent division by zero

  // 5. Calculate PID terms with time scaling
  double proportional = Kp * error;
  integral += Ki * error * dt;
  double derivative = Kd * (error - previousError) / dt;

  // 6. Compute the PID output
  double pidOutput = proportional + integral + derivative;

  // 7. Send the PID output to the ESC control function
  controlMotor(pidOutput);

  // 8. Update previous error and time for next iteration
  previousError = error;
  previousTime = currentTime;

  // 9. (Optional) Print setpoint and current angle for Serial Plotter
  Serial.print(setpoint);
  Serial.print(",");
  Serial.println(angle);

  // 10. Wait ~20 ms for an update rate of about 50 Hz
  delay(20);
}

void controlMotor(double output) {
  int escValue = 1500; // Neutral throttle value
  // // Assume PID output range is -90 to +90
  // if (output >= 0) {
  //   // For positive output, map from 1500 (neutral) to 2000 µs (full forward)
  //   escValue = map((int)output, 0, 90, 1500, 2000);
  //   escValue = constrain(escValue, 1500, 2000);
  // } else {
  //   // For negative output, map from 1500 (neutral) to 1000 µs (full reverse)
    escValue = map((int)output, -90, 0, 1000, 1500);
    escValue = constrain(escValue, 1000, 1500);
  
  esc.writeMicroseconds(escValue);
}
