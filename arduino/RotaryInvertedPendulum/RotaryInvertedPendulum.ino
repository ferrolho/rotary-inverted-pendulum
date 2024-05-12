#include <AccelStepper.h>

// Define the stepper motor connections
#define dirPin 2  // Direction
#define stepPin 3 // Step

// Create an instance of the AccelStepper class
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

long motor_target_pos = 0;

void setup()
{
    // Start the serial communication
    Serial.begin(115200);

    // Set the maximum speed and acceleration:
    stepper.setMaxSpeed(200000);
    stepper.setAcceleration(100000);
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

    // Move the motor to the target position
    stepper.moveTo(motor_target_pos);

    // Run the stepper motor
    stepper.run();
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
    else if (receivedMessage.startsWith("SET_TARGET"))
    {
        // Extract the target position from the message
        String targetPosString = receivedMessage.substring(10);
        long newTargetPos = targetPosString.toInt();
        // Set the new target position
        motor_target_pos = newTargetPos;
    }

    // Reset the received message
    receivedMessage = "";
}
