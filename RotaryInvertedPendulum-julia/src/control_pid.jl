function pid_control()
    # Initialise motor variables
    actual_position_motor = 0
    target_position_motor = 0

    # Initialise pendulum variables
    actual_position_pendulum = 0.0
    target_position_pendulum = 180.0

    # Initialise the PID controller constants
    Pcrit = 0.0
    Kp = 0.0
    Ki = 0.0
    Kd = 0.0

    # Initialise the PID controller variables
    error = 0.0
    last_error = 0.0
    integral = 0.0
    derivative = 0.0

    # Initialise the joystick
    js = open_joystick()
    jsaxes = JSState()
    jsbuttons = JSButtonState()
    async_read!(js, jsaxes, jsbuttons)

    LibSerialPort.open("/dev/cu.usbserial-110", BAUD_RATE) do arduino
        # Wait until the Arduino is ready
        wait_until_ready(arduino)

        # Initialise variables for tracking time
        last_update_time = time()

        # Main loop
        running = true
        while running
            # Set the read and write timeouts
            set_read_timeout(arduino, 0.1)
            set_write_timeout(arduino, 0.1)

            # Update Pcrit based on the joystick input
            Pcrit += 0.01 * -jsaxes.y
            Pcrit = clamp(Pcrit, 0.0, 10.0)

            # Calculate the PID controller gains
            Kp = 0.6 * Pcrit
            Ki = 2.0 * Pcrit / 100
            Kd = 0.125 * Pcrit * 100

            # Get the current time
            current_time = time()

            # Calculate the elapsed time since the last update
            elapsed_time = current_time - last_update_time

            if elapsed_time >= 1 / CONTROL_FREQUENCY
                # Get the actual position from the Arduino
                write(arduino, "$GET_POSITION_COMMAND\n")
                actual_position_motor = parse(Int, chomp(readline(arduino)))

                # Get the pendulum position from the Arduino
                write(arduino, "$GET_POSITION_PENDULUM_COMMAND\n")
                actual_position_pendulum = parse(Float64, chomp(readline(arduino)))

                # Calculate the error of the pendulum position
                error = target_position_pendulum - actual_position_pendulum

                # Calculate the integral
                integral += error #* elapsed_time

                # Calculate the derivative
                derivative = (error - last_error) #/ elapsed_time

                # Calculate the control signal
                control_signal = Kp * error + Ki * integral + Kd * derivative

                target_position_motor = actual_position_motor + round(Int, control_signal)

                # Only send the control signal if the pendulum
                # is within this margin of the target position
                margin = 20.0
                if 180 - margin <= actual_position_pendulum <= 180 + margin
                    # Send the control signal to the Arduino
                    write(arduino, "$SET_TARGET_COMMAND $target_position_motor\n")
                end

                # Update the last error
                last_error = error

                # Update the last update time
                last_update_time = current_time

                # # Print the PID controller gains
                # println("Kp: $Kp, Ki: $Ki, Kd: $Kd")

                # # Print the actual and target motor positions
                # print("Actual position: $actual_position_motor, ")
                # println("Target position: $target_position_motor")

                # # Print the actual and target pendulum positions
                # print("Actual pendulum position: $actual_position_pendulum, ")
                # println("Target pendulum position: $target_position_pendulum")

                # Print the PID controller gains and the actual pendulum position
                println("Kp: $Kp, Ki: $Ki, Kd: $Kd, Actual pendulum position: $actual_position_pendulum, Actual motor position: $actual_position_motor, Target motor position: $target_position_motor")
            end

            if jsbuttons.btn1.val  # Xbox A-button
                # Start the motor
                write(arduino, "$START_MOTOR_COMMAND\n")
            elseif jsbuttons.btn2.val  # Xbox B-button
                # Stop the motor
                write(arduino, "$STOP_MOTOR_COMMAND\n")
            elseif jsbuttons.btn5.val  # Xbox Y-button
                # Stop the motor
                write(arduino, "$STOP_MOTOR_COMMAND\n")

                # Stop the main loop
                running = false
            end

            # Sleep for a short period of time
            sleep(Millisecond(10))  # 10 ms
        end
    end
end

export pid_control
