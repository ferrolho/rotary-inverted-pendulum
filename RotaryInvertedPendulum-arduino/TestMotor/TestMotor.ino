#include <AccelStepper.h>

// Define the stepper motor connections
#define dirPin 2  // Direction
#define stepPin 3 // Step

// Create an instance of the AccelStepper class
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin, 0, 0, false);

#define stepsPerRevolution 200 * 8 // 200 steps per revolution * 8 microsteps

void setup()
{
    Serial.begin(115200); // Start the serial communication

    // Set the maximum speed and acceleration
    stepper.setMaxSpeed(20000);
    stepper.setAcceleration(10000);

    // Set the enable pin for the stepper motor driver and
    // invert it because we are using a DRV8825 board with an
    // active-low enable signal (LOW = enabled, HIGH = disabled)
    stepper.setEnablePin(5);
    stepper.setPinsInverted(false, false, true);

    // Set the initial position
    stepper.setCurrentPosition(0);

    // Enable the motor outputs
    stepper.enableOutputs();

    Serial.println("Motor Test");
}

void loop()
{
    // Move the motor 1/4 of a revolution
    Serial.println("Moving motor 1/4 of a revolution");
    stepper.runToNewPosition(stepsPerRevolution / 4);

    // Wait for 1 second
    delay(1000);

    // Move the motor back 1/4 of a revolution
    Serial.println("Moving motor back 1/4 of a revolution");
    stepper.runToNewPosition(-stepsPerRevolution / 4);

    // Wait for 1 second
    delay(1000);
}
