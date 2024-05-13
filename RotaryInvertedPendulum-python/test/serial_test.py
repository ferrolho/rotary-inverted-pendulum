import serial
import time

# Command constants
CHECK_READY_COMMAND = "CHECK_READY"
GET_POSITION_COMMAND = "GET_POSITION"
SET_TARGET_COMMAND = "SET_TARGET"


def check_ready(arduino):
    # Send the CHECK_READY command to the Arduino
    arduino.write(f"{CHECK_READY_COMMAND}\n".encode())

    # Wait for the response
    response = arduino.readline().decode().strip()

    # Check if the Arduino is ready
    return response == "READY"


def test_get_position(arduino):
    # Send the GET_POSITION command to the Arduino
    arduino.write(f"{GET_POSITION_COMMAND}\n".encode())

    # Read the response from the Arduino
    response = arduino.readline().decode().strip()

    # Print the response
    print("Response from Arduino:", response)


def test_set_target(arduino, target_position):
    # Send the SET_TARGET command to the Arduino
    arduino.write(f"{SET_TARGET_COMMAND} {target_position}\n".encode())

    # Print the target position
    print(f"Sent target position: {target_position}")


def main():
    # Set up the serial connection
    arduino = serial.Serial("/dev/cu.usbserial-110", 9600, timeout=1)

    # Check if the Arduino is ready to receive commands
    while not check_ready(arduino):
        print("Arduino is not ready. Retrying...")
        time.sleep(0.1)  # Wait for 100 ms before trying again

    print("Arduino is ready to receive commands.")

    for i in range(10):
        test_get_position(arduino)
        time.sleep(0.1)

    test_set_target(arduino, 1600)

    for i in range(10):
        test_get_position(arduino)
        time.sleep(0.1)

    # Close the serial connection
    arduino.close()


if __name__ == "__main__":
    main()
