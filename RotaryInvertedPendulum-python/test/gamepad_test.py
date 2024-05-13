#!/usr/bin/env python3

import pygame


def main():
    pygame.init()

    # Set up the Xbox controller
    joystick = None
    for i in range(pygame.joystick.get_count()):
        if "Xbox" in pygame.joystick.Joystick(i).get_name():
            joystick = pygame.joystick.Joystick(i)
            joystick.init()

    # Main loop
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Get the state of the controller
        pygame.event.pump()
        axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
        buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
        hats = [joystick.get_hat(i) for i in range(joystick.get_numhats())]

        # Print controller state to the console
        print("Axes:", axes)
        print("Buttons:", buttons)
        print("Hats:", hats)

        # Sleep for a short time
        pygame.time.wait(100)

    pygame.quit()


if __name__ == "__main__":
    main()
