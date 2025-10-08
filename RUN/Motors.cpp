#include <Arduino.h>
#include <ESP32Servo.h>

// -- Pin Definitions --
const int SERVO_X_PIN = 26; // Horizontal movement
const int SERVO_Y_PIN = 25; // Vertical movement
const int LASER_PIN = 33;

// -- Servo Configuration --
// These define the physical limits of your servos (0-180 degrees standard)
const int SERVO_X_MIN_ANGLE = 0;
const int SERVO_X_MAX_ANGLE = 180;
const int SERVO_Y_MIN_ANGLE = 0;
const int SERVO_Y_MAX_ANGLE = 180;

// Create servo objects
Servo myServo;  // Corresponds to SERVO_X_PIN (GPIO26)
Servo myServo2; // Corresponds to SERVO_Y_PIN (GPIO25)

void setup() {
  Serial.begin(115200);
  Serial.println("\n--- ESP32 3D Tracker Receiver Ready ---");
  Serial.println("Waiting for direct servo angles (X_angle,Y_angle) from Python...");

  myServo.attach(SERVO_X_PIN);
  myServo2.attach(SERVO_Y_PIN);

  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, HIGH); // Laser ON for testing

  // Center servos initially
  myServo.write(90);
  myServo2.write(90);
  delay(1000); // Give servos time to move
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim(); // Remove any whitespace (like carriage return)

    int commaIndex = data.indexOf(',');

    if (commaIndex > 0) {
      String xAngleStr = data.substring(0, commaIndex);
      String yAngleStr = data.substring(commaIndex + 1);

        int servoXAngle = xAngleStr.toInt();
        int servoYAngle = yAngleStr.toInt();
// Map the original 0-180 range to your desired 25-160 range
      int mappedXAngle = map(servoXAngle, 0, 180, 35, 145);
      int mappedYAngle = map(servoYAngle, 0, 180, 45, 135);

      // For safety, constrain the final value to be absolutely within the limits
      int finalXAngle = constrain(mappedXAngle, 25, 160);
      int finalYAngle = constrain(mappedYAngle, 50, 130);


      // Write the angles directly to the servos
      myServo.write(finalXAngle);
      myServo2.write(finalYAngle);

    }
  }
}