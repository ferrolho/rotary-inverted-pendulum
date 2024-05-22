function convertDegreesToSteps(degrees; microstepping=8)
    steps_per_revolution = 200 * microstepping
    degrees_per_revolution = 360.0
    steps_per_degree = steps_per_revolution / degrees_per_revolution
    steps = round(Int, degrees * steps_per_degree)
    return steps
end

# [hf] Is this the best way to define the State module?
baremodule State
WAITING = 1
BALANCING = 2
end

function pid_control(baud_rate=2000000, control_frequency=200)
    # Initialise motor variables
    actual_position_motor = 0
    target_position_motor = 0

    # Initialise pendulum variables
    actual_position_pendulum = 0.0
    target_position_pendulum = 180.0

    # Ziegler-Nichols parameters
    Ku = 2.0
    Tu = 100

    # Initialise the PID controller variables
    pid_prev_error = 0.0
    pid_integral = 0.0

    # [hf] could the joystick be slowing down the loop?
    # # Initialise the joystick
    # js = open_joystick()
    # jsaxes = JSState()
    # jsbuttons = JSButtonState()
    # async_read!(js, jsaxes, jsbuttons)

    # Initialise the system state
    state = State.WAITING

    LibSerialPort.open("/dev/cu.usbserial-110", baud_rate) do arduino
        # Wait until the Arduino is ready
        wait_until_ready(arduino)

        # Initialise variables for tracking time
        last_update_time = time()

        # Main loop
        running = true
        while running
            # Set the read and write timeouts
            set_read_timeout(arduino, 0.05)  # 50 ms
            set_write_timeout(arduino, 0.01)  # 10 ms

            # # Update Ku based on the joystick input
            # Ku += 0.01 * -jsaxes.y
            # Ku = clamp(Ku, 0.0, 10.0)
            # # Update Tu based on the joystick input
            # Tu += 0.1 * -jsaxes.u
            # Tu = clamp(Tu, 0.0, 200.0)

            # # Temporary values for finding the critical gain
            # Kp = Ku
            # Ki = 0.0
            # Kd = 0.0

            # # Calculate the PID controller gains
            # Kp = 0.6 * Ku
            # Ki = 2.0 * Kp / Tu
            # Kd = Kp * Tu / 8.0

            # Manually-tuned PID controller gains
            Kp = 2.2
            Ki = 1.6
            Kd = 0.005

            # Get the actual position of the motor from the Arduino
            write(arduino, "$GET_POSITION_COMMAND\n")
            actual_position_motor = parse(Int, chomp(readline(arduino)))

            # Get the actual position of the pendulum from the Arduino
            write(arduino, "$GET_POSITION_PENDULUM_COMMAND\n")
            actual_position_pendulum = parse(Float64, chomp(readline(arduino)))

            # Calculate the error of the pendulum position
            pid_error = target_position_pendulum - actual_position_pendulum

            # Only send the control signal if the pendulum
            # is within this margin of the target position
            margin_in_deg = 25.0  # in degrees
            pendulum_close_to_vertical = abs(pid_error) < margin_in_deg

            if state == State.WAITING
                # If the pendulum is close to the vertical position,
                # switch to the balancing state
                if pendulum_close_to_vertical
                    println("Switching state to 'balancing'")
                    # Switch the state to 'balancing'
                    state = State.BALANCING
                    # Engage the motor
                    write(arduino, "$START_MOTOR_COMMAND\n")
                    # Set the motor target position to the current position
                    target_position_motor = actual_position_motor
                end
            elseif state == State.BALANCING
                # If the pendulum is not close to the vertical position,
                # switch to the waiting state
                if !pendulum_close_to_vertical
                    println("Switching state to 'waiting'")
                    # Switch the state to 'waiting'
                    state = State.WAITING
                    # Disengage the motor
                    write(arduino, "$STOP_MOTOR_COMMAND\n")
                    # Reset the PID controller variables
                    pid_prev_error = 0.0
                    pid_integral = 0.0
                end
            end

            # Get the current time
            current_time = time()

            # Calculate the elapsed time since the last update
            elapsed_time = current_time - last_update_time

            # Update the last update time
            last_update_time = current_time

            if elapsed_time >= 1 / control_frequency
                if state == State.BALANCING
                    # Calculate the integral
                    pid_integral += pid_error * elapsed_time

                    # Calculate the derivative
                    pid_derivative = (pid_error - pid_prev_error) / elapsed_time
                    pid_prev_error = pid_error  # Update the last error

                    # Calculate the output of the PID controller
                    output = Kp * pid_error + Ki * pid_integral + Kd * pid_derivative

                    # Clamp the output to prevent the motor from moving too fast
                    output_limit = convertDegreesToSteps(10.0)
                    output = clamp(output, -output_limit, output_limit)

                    # Calculate the target position of the motor
                    target_position_motor = actual_position_motor + round(Int, output)

                    # Clamp the target position to prevent the motor from choking the wires
                    motor_limit = convertDegreesToSteps(90.0)
                    target_position_motor = clamp(target_position_motor, -motor_limit, motor_limit)

                    # Send the target position to the Arduino
                    write(arduino, "$SET_TARGET_COMMAND $target_position_motor\n")
                end
            end

            let
                # Print pendulum position and loop frequency
                println("Actual pendulum position: $actual_position_pendulum, Loop frequency: $(1 / elapsed_time)")

                # # Print the Ziegler-Nichols parameters and the target motor position
                # println("Ku: $Ku, Tu: $Tu, actual_position_pendulum: $actual_position_pendulum, target_position_motor: $target_position_motor")

                # # Print the PID controller gains
                # println("Kp: $Kp, Ki: $Ki, Kd: $Kd")

                # # Print the actual and target motor positions
                # print("Actual position: $actual_position_motor, ")
                # println("Target position: $target_position_motor")

                # # Print the actual and target pendulum positions
                # print("Actual pendulum position: $actual_position_pendulum, ")
                # println("Target pendulum position: $target_position_pendulum")

                # # Print the PID controller gains and the actual pendulum position
                # println("Kp: $Kp, Ki: $Ki, Kd: $Kd, Actual pendulum position: $actual_position_pendulum, Actual motor position: $actual_position_motor, Target motor position: $target_position_motor")
            end

            # if jsbuttons.btn1.val  # Xbox A-button
            #     println("[Xbox] A-button pressed")
            #     # Start the motor
            #     write(arduino, "$START_MOTOR_COMMAND\n")
            # elseif jsbuttons.btn2.val  # Xbox B-button
            #     println("[Xbox] B-button pressed")
            #     # Stop the motor
            #     write(arduino, "$STOP_MOTOR_COMMAND\n")
            # elseif jsbuttons.btn5.val  # Xbox Y-button
            #     println("[Xbox] Y-button pressed")
            #     # Stop the motor
            #     write(arduino, "$STOP_MOTOR_COMMAND\n")
            #     # Stop the main loop
            #     running = false
            # end

            # Sleep for a short period of time
            # sleep(Millisecond(1))
        end
    end
end

export pid_control
