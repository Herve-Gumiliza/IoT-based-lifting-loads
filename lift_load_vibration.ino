#include <Servo.h>

// Create Servo objects for two servos
Servo servo1;
Servo servo2;

// Joystick pin connections
const int joystickXPin = A0;  // X-axis connected to analog pin A0
const int joystickYPin = A1;  // Y-axis connected to analog pin A1
const int buttonPin = 7;      // Joystick button connected to digital pin 7

// Servo pin connections
const int servo1Pin = 9;  // Servo 1 connected to digital pin 9
const int servo2Pin = 10; // Servo 2 connected to digital pin 10

// LED pin connections
const int led1Pin = 4;  // LED for Servo 1
const int led2Pin = 5;  // LED for Servo 2

// Vibration sensor connection
const int vibrationPin = 8;  // Vibration sensor connected to digital pin 8

// Variables to store joystick readings
int joystickXValue = 0;
int joystickYValue = 0;

// Variables to track vibration
int vibrationCount = 0;  // Count the number of vibrations detected
bool isVibrationDetected = false;

// Servo angle limits
int servoMinAngle = 0;
int servoMaxAngle = 180;

// Maintenance threshold (example: 100 vibration events)
const int maintenanceThreshold = 100;

void setup() {
  // Attach the servos to the corresponding pins
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);

  // Set up the joystick button, LEDs, and vibration sensor
  pinMode(buttonPin, INPUT_PULLUP);  // Button input with pull-up resistor
  pinMode(led1Pin, OUTPUT);          // LED for Servo 1
  pinMode(led2Pin, OUTPUT);          // LED for Servo 2
  pinMode(vibrationPin, INPUT);      // Vibration sensor input

  // Set initial servo positions to middle (90 degrees)
  servo1.write(90);
  servo2.write(90);
  
  // Start serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  // Read the joystick X-axis and Y-axis values (0 to 1023)
  joystickXValue = analogRead(joystickXPin);
  joystickYValue = analogRead(joystickYPin);
  
  // Map the joystick values to the servo range (0 to 180 degrees)
  int servo1Angle = map(joystickXValue, 0, 1023, servoMinAngle, servoMaxAngle);
  int servo2Angle = map(joystickYValue, 0, 1023, servoMinAngle, servoMaxAngle);
  
  // Set the servos to the corresponding angles
  servo1.write(servo1Angle);
  servo2.write(servo2Angle);
  
  // Check if X-axis is moving (Servo 1 is active)
  if (joystickXValue > 512 + 50 || joystickXValue < 512 - 50) {
    digitalWrite(led1Pin, HIGH);  // Turn on LED for Servo 1
  } else {
    digitalWrite(led1Pin, LOW);   // Turn off LED for Servo 1
  }

  // Check if Y-axis is moving (Servo 2 is active)
  if (joystickYValue > 512 + 50 || joystickYValue < 512 - 50) {
    digitalWrite(led2Pin, HIGH);  // Turn on LED for Servo 2
  } else {
    digitalWrite(led2Pin, LOW);   // Turn off LED for Servo 2
  }

  // Check for vibration
  if (digitalRead(vibrationPin) == HIGH) {
    if (!isVibrationDetected) {
      isVibrationDetected = true;
      vibrationCount++;  // Increment the vibration counter
      Serial.println("Vibration detected!");

      // Check if maintenance is needed
      if (vibrationCount >= maintenanceThreshold) {
        Serial.println("Maintenance required! Schedule maintenance.");
        // You can add more actions here, such as sending an alert
      }
    }
  } else {
    isVibrationDetected = false;  // Reset when no vibration is detected
  }

  // Print joystick values, servo angles, and vibration count to the serial monitor
  Serial.print("Joystick X: ");
  Serial.print(joystickXValue);
  Serial.print(" -> Servo 1 Angle: ");
  Serial.println(servo1Angle);
  
  Serial.print("Joystick Y: ");
  Serial.print(joystickYValue);
  Serial.print(" -> Servo 2 Angle: ");
  Serial.println(servo2Angle);
  
  Serial.print("Vibration count: ");
  Serial.println(vibrationCount);

  // Small delay to make the movements smoother
  delay(15);
}
