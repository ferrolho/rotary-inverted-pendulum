module RotaryInvertedPendulum

using Dates
using Joysticks
using LibSerialPort
using Plots

# Command constants
const CHECK_READY_COMMAND = "1"
const GET_POSITION_COMMAND = "2"
const GET_POSITION_PENDULUM_COMMAND = "3"
const SET_TARGET_COMMAND = "4"
const START_MOTOR_COMMAND = "5"
const STOP_MOTOR_COMMAND = "6"

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
