#include <AS5600.h>
#include <Wire.h>

// Create an instance of the AS5600 class
AMS_5600 ams5600;

// Plotting variables
int counterPlot = 0;
int frequencyPlot = 20;

// Calibration variables
const int numSamples = 100;      // Number of samples to average
const int calibrationDelay = 10; // Delay between samples (in milliseconds)
long ams5600_initial_position = 0;

void setup()
{
    Serial.begin(115200); // Start the serial communication
    Wire.begin();         // Start the I2C communication

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
        Serial.println("[AS5600] Magnet strength is too weak. ---");
    }
    else if (magStrength == 2)
    {
        Serial.println("[AS5600] Magnet strength is just right! ✔");
    }
    else if (magStrength == 3)
    {
        Serial.println("[AS5600] Magnet strength is too strong. +++");
    }

    // Calibrate the initial position by averaging multiple readings
    ams5600_initial_position = calibrateRestPosition();
    Serial.print("[AS5600] Calibrated ams5600_initial_position: ");
    Serial.println(ams5600_initial_position);
}

void loop()
{
    // Increment counters
    counterPlot++;

    // Get the pendulum position
    float pendulum_actual_deg = convertRawAngleToDegrees();

    // Print to the serial
    if (counterPlot % frequencyPlot == 0)
    {
        // Print the actual and target motor positions
        Serial.print("pendulum_actual_deg:");
        Serial.print(pendulum_actual_deg);
        Serial.println();
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

/*
 * Calibrate the rest position by averaging multiple readings
 */
long calibrateRestPosition()
{
    long sum = 0;

    for (int i = 0; i < numSamples; i++)
    {
        sum += ams5600.getRawAngle();
        delay(calibrationDelay);
    }

    return sum / numSamples;
}
