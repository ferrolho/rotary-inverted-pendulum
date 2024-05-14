#include <AccelStepper.h>
#include <AS5600.h>
#include <Wire.h>

// Define the stepper motor connections
#define dirPin 2  // Direction
#define stepPin 3 // Step

// Create an instance of the AccelStepper class
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin, 0, 0, false);

// Create an instance of the AS5600 class
AMS_5600 ams5600;

bool motor_running = false;
long motor_target_pos = 0;

word pendulum_rest_position = 0;
word pendulum_actual_position = 0;

void setup()
{
    Serial.begin(115200); // Start the serial communication
    Wire.begin();         // Start the I2C communication

    // Set the maximum speed and acceleration
    stepper.setMaxSpeed(200000);
    stepper.setAcceleration(100000);

    // Set the enable pin for the stepper motor driver and
    // invert it because we are using a DRV8825 board with an
    // active-low enable signal (LOW = enabled, HIGH = disabled)
    stepper.setEnablePin(5);
    stepper.setPinsInverted(false, false, true);

    if (ams5600.detectMagnet() == 0)
    {
        while (true)
        {
            if (ams5600.detectMagnet() == 1)
            {
                Serial.print("Current Magnitude: ");
                Serial.println(ams5600.getMagnitude());
                break;
            }
            else
            {
                Serial.println("Cannot detect magnet.");
            }
            delay(1000);
        }
    }

    pendulum_rest_position = ams5600.getRawAngle();
    Serial.print("Rest Position: ");
    Serial.println(pendulum_rest_position);
}

void loop()
{
    while (Serial.available() > 0)
    {
        char receivedChar = Serial.read();
        if (receivedChar == '\n')
        {
            handleCommand();
        }
        else
        {
            // Append characters to the received message
            parseReceivedMessage(receivedChar);
        }
    }

    if (motor_running)
    {
        // Move the motor to the target position
        stepper.moveTo(motor_target_pos);

        // Run the stepper motor
        stepper.run();
    }
}

float convertRawAngleToDegrees(word rawAngle)
{
    // Raw data reports 0–4095 segments, which is 0.087890625 of a degree
    return rawAngle * 0.087890625;
}

String receivedMessage;

void parseReceivedMessage(char receivedChar)
{
    // Append characters to the received message
    receivedMessage += receivedChar;
}

void handleCommand()
{
    // Implement logic to handle different commands
    if (receivedMessage == "CHECK_READY")
    {
        // Send a ready signal to indicate that Arduino is ready to receive commands
        Serial.println("READY");
    }
    else if (receivedMessage == "GET_POSITION")
    {
        // Send the current position to the laptop
        Serial.println(stepper.currentPosition());
    }
    else if (receivedMessage == "GET_POSITION_PENDULUM")
    {
        pendulum_actual_position = ams5600.getRawAngle();

        float difference = pendulum_actual_position - pendulum_rest_position;

        if (difference < 0)
        {
            difference += 4096;
        }

        float degrees = convertRawAngleToDegrees(difference);

        Serial.println(degrees);
    }
    else if (receivedMessage.startsWith("SET_TARGET"))
    {
        // Extract the target position from the message
        String targetPosString = receivedMessage.substring(10);
        long newTargetPos = targetPosString.toInt();
        // Set the new target position
        motor_target_pos = newTargetPos;
    }
    else if (receivedMessage == "START_MOTOR")
    {
        // Enable the motor outputs
        stepper.enableOutputs();

        // Set the motor_running flag to true
        motor_running = true;
    }
    else if (receivedMessage == "STOP_MOTOR")
    {
        // Stop the motor
        stepper.stop();

        // Disable the motor outputs
        stepper.disableOutputs();

        // Set the motor_running flag to false
        motor_running = false;
    }

    // Reset the received message
    receivedMessage = "";
}
