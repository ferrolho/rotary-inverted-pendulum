#!/usr/bin/env python

import pygame
import time
import serial

# Constants
BAUD_RATE = 115200  # Baud rate of the serial connection
CONTROL_FREQUENCY = 100  # Frequency of the control loop in Hz

# Command constants
COMMAND_CHECK_READY = "CHECK_READY"
COMMAND_GET_POSITION = "GET_POSITION"
COMMAND_SET_TARGET = "SET_TARGET"


def check_ready(arduino):
    # Send the CHECK_READY command to the Arduino
    arduino.write(f"{COMMAND_CHECK_READY}\n".encode())

    # Wait for the response
    response = arduino.readline().decode().strip()

    # Check if the Arduino is ready
    return response == "READY"


def get_position(arduino):
    """
    Get the current position of the stepper motor from the Arduino.
    """
    # Send the GET_POSITION command to the Arduino
    arduino.write(f"{COMMAND_GET_POSITION}\n".encode())

    # Read the response from the Arduino
    response = arduino.readline().decode().strip()

    # Print the response
    print("Response from Arduino:", response)

    # Return the position
    return float(response)


def set_target(arduino, target_position: int):
    """
    Set the target position of the stepper motor on the Arduino.
    """
    # Send the SET_TARGET command to the Arduino
    arduino.write(f"{COMMAND_SET_TARGET} {target_position}\n".encode())

    # Print the target position
    print(f"Sent target position: {target_position}")


def main():
    # Initialize motor position
    actual_position_motor = 0
    target_position_motor = 0

    # Initialize pygame
    pygame.init()

    # Set up the Xbox controller
    joystick = None
    for i in range(pygame.joystick.get_count()):
        if "Xbox" in pygame.joystick.Joystick(i).get_name():
            joystick = pygame.joystick.Joystick(i)
            joystick.init()

    # Set up the serial connection
    ser = serial.Serial("/dev/cu.usbserial-110", BAUD_RATE, timeout=1)

    # Wait for Arduino to be ready
    while not check_ready(ser):
        print("Arduino is not ready. Retrying...")
        time.sleep(0.1)  # Wait for 100 ms before trying again

    print("Arduino is ready to receive commands.")

    # Initialize variables for tracking time
    last_update_time = time.time()

    # Main loop
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Get the state of the controller
        pygame.event.pump()
        x_axis = joystick.get_axis(0)  # Get the x-axis value of the left joystick

        # Deadzone for the joystick
        if abs(x_axis) < 0.1:
            x_axis = 0.0

        # Set the motor speed multiplier
        multiplier = 50.0

        # Adjust motor position based on joystick input
        target_position_motor += int(x_axis * multiplier)

        # Clamp motor position between 0.0 and 1.0
        # target_position_motor = max(0.0, min(1.0, target_position_motor))

        # Get current time
        current_time = time.time()

        # Check if it is time to update the motor
        if current_time - last_update_time >= 1.0 / CONTROL_FREQUENCY:
            # Get the current position of the motor
            actual_position_motor = get_position(ser)

            # Set the target position of the motor
            set_target(ser, target_position_motor)

            # Update last update time
            last_update_time = current_time

        # Sleep for a short period of time
        time.sleep(0.01)  # 10 ms

    # Close the serial connection
    ser.close()

    # Quit pygame
    pygame.quit()


if __name__ == "__main__":
    main()
