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
long ams5600_initial_position = 0;

bool motor_running = false;
long motor_target_pos = 0;

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

    while (!ams5600.detectMagnet())
    {
        Serial.println("[AS5600] Waiting for magnet...");
        delay(1000); // Wait for the magnet to be detected
    }

    // Print the current magnitude of the magnet
    Serial.print("[AS5600] Current magnitude: ");
    Serial.println(ams5600.getMagnitude());

    // Print the magnet strength
    int magStrength = ams5600.getMagnetStrength();
    if (magStrength == 1)
    {
        Serial.println("[AS5600] Magnet strength is too weak.");
    }
    else if (magStrength == 2)
    {
        Serial.println("[AS5600] Magnet strength is just right! :chef-kiss:");
    }
    else if (magStrength == 3)
    {
        Serial.println("[AS5600] Magnet strength is too strong.");
    }

    ams5600_initial_position = ams5600.getRawAngle();
    Serial.print("[AS5600] ams5600_initial_position: ");
    Serial.println(ams5600_initial_position);
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

/*
 * Convert the raw angle from the AS5600 magnetic encoder to degrees.
 */
float convertRawAngleToDegrees()
{
    // Get the current position of the AS5600
    short ams5600_current_position = ams5600.getRawAngle();

    // Calculate the difference between the current position and the initial position
    short difference = ams5600_current_position - ams5600_initial_position;

    if (difference < 0)
    {
        difference += 4096;
    }

    // Map the 0–4095 segments of the AS5600 to 0–360 degrees
    // 360 degrees / 4096 segments = 0.087890625 degrees per segment
    return difference * 0.087890625;
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
        // Send the current position of the stepper motor over serial
        Serial.println(stepper.currentPosition());
    }
    else if (receivedMessage == "GET_POSITION_PENDULUM")
    {
        // Get the pendulum position
        float pendulum_actual_deg = convertRawAngleToDegrees();

        // Send the current position of the pendulum over serial
        Serial.println(pendulum_actual_deg);
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
