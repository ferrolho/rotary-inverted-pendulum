module RotaryInvertedPendulum

using Dates
using Joysticks
using LibSerialPort
using Plots

# Constants
const BAUD_RATE = 115200  # Baud rate for the serial communication
CONTROL_FREQUENCY = 100  # Frequency of the control loop (in Hz)

# Command constants
const CHECK_READY_COMMAND = "CHECK_READY"
const GET_POSITION_COMMAND = "GET_POSITION"
const GET_POSITION_PENDULUM_COMMAND = "GET_POSITION_PENDULUM"
const SET_TARGET_COMMAND = "SET_TARGET"
const START_MOTOR_COMMAND = "START_MOTOR"
const STOP_MOTOR_COMMAND = "STOP_MOTOR"

function wait_until_ready(arduino)
    ready = false
    response = ""
    timed_out = true

    while !ready
        try
            if timed_out
                println("Checking if the Arduino is ready...")
            end

            # Wait for the Arduino to send a response
            set_read_timeout(arduino, 1)  # 1 second

            # Try to read a response from the Arduino
            response = readline(arduino)

            if timed_out
                println("  -- Arduino responded:")
                timed_out = false
            end
        catch e
            if isa(e, LibSerialPort.Timeout)
                println("  -- Serial port timed out. Arduino is not ready.")

                # Send the CHECK_READY command to the Arduino
                write(arduino, "$CHECK_READY_COMMAND\n")

                timed_out = true
            else
                rethrow(e)
            end
        else
            # Print the response
            println("  >> $response")

            # Check if the Arduino is ready
            ready = chomp(response) == "READY"
        end
    end

    println("Arduino is ready to receive commands.")
end

include("control_gamepad.jl")
include("control_pid.jl")

export check_ready

end # module RotaryInvertedPendulum
