#include <AccelStepper.h>
#include <AS5600.h>
#include <Wire.h>

// Define the stepper motor connections
#define dirPin 2  // Direction
#define stepPin 3 // Step
#define LED_PIN 13

#define frac(x)     (int(1000*(x - int(x))))

// Create an instance of the AccelStepper class
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin, 0, 0, false);

#define stepsPerRevolution 200 * 8 // 200 steps per revolution * 8 microsteps

// Create an instance of the AS5600 class
AMS_5600 ams5600;
float pendulum_initial_position = 0.0f;
float pendulum_target_deg = 180.0;
float motor_target_pos = 0.0;

float prevError = 0.0;
float integral = 0.0;

// // Variables for PID control
// float Kp = 3.0;
// float Ki = 0.0;
// float Kd = 0.0;

// Ziegler-Nichols parameters
float Ku = 2.0; // critical gain
float Tu = 0.1; // oscillation period

// Variables for PID control
float Kp = 0.6 * Ku;
float Ki = 2 * Kp / Tu;
float Kd = Kp * Tu / 8;

// Define the previous time
unsigned long prevTime = 0;

// Define the control frequency and period
const int controlFrequency = 1000;                   // in Hz
const float controlPeriod = 1 / controlFrequency; // in ms

// Define variables for moving average filter
double pendulum_actual_deg;

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

bool can_print = false;

void tare_pendulum_encoder()
{
  pendulum_initial_position = 0.0;
  pendulum_initial_position = convertRawAngleToDegrees();
}

// Compute the exponential smoothing coefficient between 0.0 and 1.0.
// 0.0: No filter. Only update value is used.
// 1.0: No update. Only state value is used.
double alpha_from_freq(double freq, double dt)
{
  double omega = 2.0*M_PI*freq;
  double coeff = (1.0-omega*dt/2.0)/(1.0+omega*dt/2.0);
  
  //Clamp smoothing coefficient
  if (coeff < 0.0) {
      coeff = 0.0;
  }
  if (coeff > 1.0) {
      coeff = 1.0;
  }

  return coeff;
}

void print_magnet_info()
{
  // Magnet strength
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
  // Magnet current
  Serial.print("[AS5600] Current magnitude: ");
  Serial.println(ams5600.getMagnitude());
}

void print_plot(float target, float pos)
{
  static unsigned long t0 = 0;
  if (can_print)
  {
    unsigned long t1 = millis();
    if (t1 - t0 > 10)
    {
      t0 = t1;
      // Print the actual and target motor positions
      Serial.print(stepper.currentPosition());
      Serial.print(",");
      Serial.print(target);
      Serial.print(",");
      Serial.println(pos);
      // Reset the counter
      counterPlot = 0;
    }
  }
}

void check_serial()
{
  if (Serial.available())
  {
    char buffer[1024];
    int size = Serial.readBytes(buffer, 1024);
    if (size >= 0)
    {
      char cmd = buffer[0];
      switch (cmd)
      {
        case 'P':
        case 'p':
          can_print = !can_print;
          break;
        case 'M':
        case 'm':
          print_magnet_info();
          break;
        case 'T':
        case 't':
          tare_pendulum_encoder();
          break;
      }
    }
  }
}

void setup()
{
    Serial.begin(115200); // Start the serial communication
    Wire.begin();         // Start the I2C communication

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

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
        delay(1000); // Wait for the magnet to be detected
    }

    // Set initial position
    tare_pendulum_encoder();

    // Initialise the previous time
    prevTime = millis();

    digitalWrite(LED_PIN, LOW);

    pendulum_actual_deg = convertRawAngleToDegrees();
}

void blink()
{
  static unsigned long t0 = 0;
  static bool is_on = false;
  int period = can_print ? 500 : 100;
  unsigned long t1 = millis();
  if (t1 - t0 > period)
  {
    digitalWrite(LED_PIN, is_on ? HIGH : LOW);
    is_on = !is_on;
    t0 = t1;
  }
}

void loop()
{
    blink();

    check_serial();
    // Get the current time
    unsigned long currentTime = millis();

    static unsigned long prevTimeUS = 0;
    unsigned long currentTimeUS = micros();
    unsigned long dt = currentTimeUS - prevTimeUS;
    float elapsedTime = (float)dt * 1e-6;
    prevTimeUS = currentTimeUS;

    // Increment counters
    counterPlot++;

    // first, we wait for a person to move the pendulum close to the vertical position.
    // then we start the motor and we try to balance it.
    // we should not command the motor beyond +-90 degrees from its starting position.

    double alpha = alpha_from_freq(500.0, (double)dt * 1e-6);
    // Get the pendulum position
    pendulum_actual_deg = alpha * pendulum_actual_deg + (1.0 - alpha) * convertRawAngleToDegrees();

    // Find closest upright target
    int revs = pendulum_actual_deg / 360;
    pendulum_target_deg = 180.0f * (pendulum_actual_deg > 0 ? 1.0f : -1.0f) + 360.0f * (float)revs;

    float margin_in_deg = 25.0; // in degrees
    bool pendulum_close_to_vertical = fabs(pendulum_target_deg - pendulum_actual_deg) <= margin_in_deg;

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

    print_plot(motor_target_pos, pendulum_actual_deg);
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
    static long raw_prev = 0;
    static bool first_reading = true;
    static float position = 0.0f;
    // Get the current position of the AS5600
    long raw = ams5600.getRawAngle();
    if (first_reading) 
    {
      raw_prev = raw;
      first_reading = false;
    }
    long delta = raw - raw_prev;
    // Handle wrap around
    if (delta > 2047) delta -= 4098;
    if (delta < -2047) delta += 4098;
    // Map the 0–4095 segments of the AS5600 to 0–360 degrees
    // 360 degrees / 4096 segments = 0.087890625 degrees per segment
    position += (float)delta * 0.087890625;
    raw_prev = raw;

    return position;
}
