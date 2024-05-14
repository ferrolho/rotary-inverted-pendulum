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
const SET_TARGET_COMMAND = "SET_TARGET"
const START_MOTOR_COMMAND = "START_MOTOR"
const STOP_MOTOR_COMMAND = "STOP_MOTOR"

function wait_until_ready(arduino)
    ready = false
    response = ""

    while !ready
        try
            println("Checking if the Arduino is ready...")

            # Set the read and write timeouts
            set_read_timeout(arduino, 1)  # 1 second
            set_write_timeout(arduino, 1)  # 1 second

            # Send the CHECK_READY command to the Arduino
            write(arduino, "$CHECK_READY_COMMAND\n")

            # Wait for the response
            response = readline(arduino)
        catch e
            if isa(e, LibSerialPort.Timeout)
                println("Arduino is not ready. Retrying...")
            else
                rethrow(e)
            end
        else
            # Check if the Arduino is ready
            ready = chomp(response) == "READY"
        end
    end

    println("Arduino is ready to receive commands.")
end

include("control_gamepad.jl")

export check_ready

end # module RotaryInvertedPendulum
