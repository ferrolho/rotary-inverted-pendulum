#include <AccelStepper.h>
#include <AS5600.h>
#include <Wire.h>

// Define the stepper motor connections
#define dirPin 2  // Direction
#define stepPin 3 // Step

// Create an instance of the AccelStepper class
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin, 0, 0, false);

#define stepsPerRevolution 200 * 8 // 200 steps per revolution * 8 microsteps

// Create an instance of the AS5600 class
AMS_5600 ams5600;
long ams5600_initial_position = 0;
const float pendulum_target_deg = 180.0;
float motor_target_pos = 0.0;

float prevError = 0.0;
float integral = 0.0;

// // Variables for PID control
// float Kp = 3.0;
// float Ki = 0.0;
// float Kd = 0.0;

// Ziegler-Nichols parameters
float Ku = 2.0; // critical gain
float Tu = 100; // oscillation period

// Variables for PID control
float Kp = 0.6 * Ku;
float Ki = 2 * Kp / Tu;
float Kd = Kp * Tu / 8;

// Define the previous time
unsigned long prevTime = 0;

// Define the control frequency and period
const int controlFrequency = 1000;                   // in Hz
const float controlPeriod = 1000 / controlFrequency; // in ms

// Define variables for moving average filter
const int numReadings = 1;   // Number of readings to average
float readings[numReadings]; // Array to store readings
int index = 0;               // Index for current reading
float total = 0.0;           // Total of readings

// State machine variables
enum State
{
    WAITING,
    BALANCING
};
State state = WAITING;

// Plotting variables
int counterPlot = 0;
int frequencyPlot = 20;

void setup()
{
    Serial.begin(115200); // Start the serial communication
    Wire.begin();         // Start the I2C communication

    delay(1000); // Wait for the serial monitor to open

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

    // Initialize the readings array
    for (int i = 0; i < numReadings; i++)
    {
        readings[i] = 0.0;
    }

    // Initialise the previous time
    prevTime = millis();
}

void loop()
{
    // Get the current time
    unsigned long currentTime = millis();

    // Calculate the elapsed time (in milliseconds)
    unsigned long elapsedTime = currentTime - prevTime;

    // Increment counters
    counterPlot++;

    // first, we wait for a person to move the pendulum close to the vertical position.
    // then we start the motor and we try to balance it.
    // we should not command the motor beyond +-90 degrees from its starting position.

    // Get the pendulum position
    float pendulum_actual_deg = convertRawAngleToDegrees();

    // Update the moving average filter
    updateMovingAverage(pendulum_actual_deg);
    pendulum_actual_deg = total / numReadings;

    float margin_in_deg = 25.0; // in degrees
    bool pendulum_close_to_vertical = abs(pendulum_target_deg - pendulum_actual_deg) <= margin_in_deg;

    if (state == WAITING)
    {
        if (pendulum_close_to_vertical)
        {
            // Switch the state to 'balancing'
            state = BALANCING;

            // Enable the motor outputs
            stepper.enableOutputs();

            // Set the motor target position to the current position
            motor_target_pos = stepper.currentPosition();
        }
    }
    else if (state == BALANCING)
    {
        if (!pendulum_close_to_vertical)
        {
            // Switch the state to 'waiting'
            state = WAITING;

            // Stop the motor
            stepper.stop();

            // Disable the motor outputs
            stepper.disableOutputs();

            // Reset the PID control variables
            prevError = 0.0;
            integral = 0.0;
        }
    }

    if (state == BALANCING && elapsedTime >= controlPeriod)
    {
        // Update the previous time
        prevTime = currentTime;

        // Calculate the error
        float error = pendulum_target_deg - pendulum_actual_deg;

        // Calculate the integral
        integral += error * elapsedTime;

        // Calculate the derivative
        float derivative = (error - prevError) / elapsedTime;
        prevError = error;

        // Calculate the PID output
        float output = Kp * error + Ki * integral + Kd * derivative;

        // Limit the output to prevent the motor from moving too fast
        long output_limit = convertDegreesToSteps(10);
        if (output > output_limit)
        {
            output = output_limit;
        }
        else if (output < -output_limit)
        {
            output = -output_limit;
        }

        // Update motor target position based on PID output
        motor_target_pos = stepper.currentPosition() + output;

        // Limit the motor target position to prevent the motor from moving beyond +-90 degrees
        if (abs(motor_target_pos) > convertDegreesToSteps(90))
        {
            motor_target_pos = stepper.currentPosition();
        }

        // Move the motor
        stepper.moveTo(motor_target_pos);
    }

    if (state == BALANCING)
    {
        // Move the motor to the target position
        stepper.moveTo(motor_target_pos);

        // Run the stepper motor
        stepper.run();
    }

    if (counterPlot == frequencyPlot)
    {
        // Print the actual and target motor positions
        Serial.print(",motor_actual_pos:");
        Serial.print(stepper.currentPosition());
        Serial.print(",motor_target_pos:");
        Serial.print(motor_target_pos);
        Serial.print(",pendulum_actual_deg:");
        Serial.print(pendulum_actual_deg);
        Serial.println();

        // Reset the counter
        counterPlot = 0;
    }
}

long convertDegreesToSteps(float degrees)
{
    // Convert degrees to steps
    return degrees * stepsPerRevolution / 360;
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

/*
 * Update the moving average filter.
 */
void updateMovingAverage(float newValue)
{
    total -= readings[index];
    readings[index] = newValue;
    total += newValue;
    index = (index + 1) % numReadings;
}
