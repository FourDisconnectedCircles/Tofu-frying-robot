#include <Arduino.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Instantiate the servo driver with default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Define pin assignments for the stepper motors
int step_X = 54;
int Dir_X = 55;
int ena_X = 38;

int step_Y = 60;
int Dir_Y = 61;
int ena_Y = 56;

// Initialize stepper motor objects
AccelStepper Step_X(AccelStepper::DRIVER, step_X, Dir_X);
AccelStepper Step_Y(AccelStepper::DRIVER, step_Y, Dir_Y);

// Set a timeout for the motors to stop when no command is received
unsigned long timeout = 100;  // Reduced timeout for immediate stop
unsigned long lastCommandTime = 0;

// Define the conversion factor from mm to steps
const float mm_to_steps = 5.0;  // 1 mm equals 5 steps

// Define deadband threshold (in mm) to prevent oscillation
const float deadband_mm = 2.0;  // Change this value to tune stability

// Variables for servo control:
#define SERVOMIN  125 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  575 // this is the 'maximum' pulse length count (out of 4096)

// Function to convert angle to PWM pulse
int angleToPulse(int ang) {
  int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX); // map angle of 0 to 180 to Servo min and Servo max 
  return pulse;
}

// Servo configuration 1
void servoConfig1() {
  pwm.setPWM(0, 0, angleToPulse(100) );
  pwm.setPWM(1, 0, angleToPulse(60) );
  pwm.setPWM(2, 0, angleToPulse(180) );
  pwm.setPWM(3, 0, angleToPulse(5) );
  pwm.setPWM(4, 0, angleToPulse(127) );

}

// Timing variables for servoConfig2
unsigned long servoConfig2StartTime = 0;
bool servoConfig2Active = false;

// Function to execute servoConfig2 sequence
void servoConfig2Sequence() {
  unsigned long currentTime = millis();

  if (currentTime - servoConfig2StartTime < 2000) {
    pwm.setPWM(0, 0, angleToPulse(100) );
    pwm.setPWM(1, 0, angleToPulse(90) );
    pwm.setPWM(2, 0, angleToPulse(130) );
    pwm.setPWM(3, 0, angleToPulse(5) );
    pwm.setPWM(4, 0, angleToPulse(127) );//open
  } else if (currentTime - servoConfig2StartTime < 4000) {
    pwm.setPWM(3, 0, angleToPulse(5));
    pwm.setPWM(4, 0, angleToPulse(140)); // close
  } else if (currentTime - servoConfig2StartTime < 6000) {
    pwm.setPWM(3, 0, angleToPulse(180));
    pwm.setPWM(4, 0, angleToPulse(140)); // close
  } else if (currentTime - servoConfig2StartTime < 8000) {
    pwm.setPWM(3, 0, angleToPulse(180));
    pwm.setPWM(4, 0, angleToPulse(127));
  } 
   else {
    pwm.setPWM(0, 0, angleToPulse(100) );
    pwm.setPWM(1, 0, angleToPulse(60) );
    pwm.setPWM(2, 0, angleToPulse(180) );
    pwm.setPWM(3, 0, angleToPulse(180));
    pwm.setPWM(4, 0, angleToPulse(130));// open
    servoConfig2Active = false; // End servoConfig2
  }
}

void setup() {
  Serial.begin(115200);

  // Initialize the stepper motors
  Step_X.setEnablePin(ena_X);
  Step_X.setPinsInverted(false, false, true); // Enable inversion if needed
  Step_Y.setEnablePin(ena_Y);
  Step_Y.setPinsInverted(false, false, true); // Enable inversion if needed

  pinMode(ena_X, OUTPUT);
  pinMode(ena_Y, OUTPUT);
  digitalWrite(ena_X, LOW); // Enable the stepper drivers
  digitalWrite(ena_Y, LOW); // Enable the stepper drivers

  Step_X.setMaxSpeed(1000);
  Step_X.setAcceleration(500);
  Step_Y.setMaxSpeed(1000);
  Step_Y.setAcceleration(500);

  Step_X.setCurrentPosition(0);
  Step_Y.setCurrentPosition(0);

  // Initialize the servo driver
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  // Set initial servo configuration
  servoConfig1();
}

void loop() {
  // Check for serial input
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    int firstCommaIndex = input.indexOf(',');
    int secondCommaIndex = input.indexOf(',', firstCommaIndex + 1);

    if (firstCommaIndex > 0 && secondCommaIndex > 0) {
      float x_mm = input.substring(0, firstCommaIndex).toFloat();
      float y_mm = input.substring(firstCommaIndex + 1, secondCommaIndex).toFloat();
      int z = input.substring(secondCommaIndex + 1).toInt();

      // Convert mm to steps
      int x_steps = static_cast<int>(x_mm * mm_to_steps);
      int y_steps = static_cast<int>(y_mm * mm_to_steps);

      // Check if the offset is within the deadband
      if (abs(x_mm) > deadband_mm || abs(y_mm) > deadband_mm) {
        Step_X.move(x_steps);
        Step_Y.move(y_steps);
        Step_X.enableOutputs();
        Step_Y.enableOutputs();
        lastCommandTime = millis();
      }

      // Handle servo configuration based on z value
      if (z == 0) {
        servoConfig1();
        servoConfig2Active = false;
      } else if (z == 1) {
        if (!servoConfig2Active) {
          servoConfig2StartTime = millis();
          servoConfig2Active = true;
        }
      }
    }
  }

  // Update servo configuration if in CONFIG2 state
  if (servoConfig2Active) {
    servoConfig2Sequence();
  }

  // Stop the motors if no command is received within the timeout period
  if (millis() - lastCommandTime > timeout) {
    Step_X.stop();
    Step_Y.stop();
    Step_X.runToPosition();  // Ensure the stepper stops immediately
    Step_Y.runToPosition();  // Ensure the stepper stops immediately
    Step_X.disableOutputs();
    Step_Y.disableOutputs();
  } else {
    Step_X.run();
    Step_Y.run();
  }

  // Add a small delay to prevent overwhelming the hardware
  delay(10);  // Adjust this delay to balance responsiveness and CPU load
}
