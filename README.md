# Rotary Inverted Pendulum

The rotary inverted pendulum is a classic control problem that is used to demonstrate the principles of control theory. The system consists of a pendulum that is mounted on a rotary base, and the goal is to balance the pendulum in the upright position by controlling the rotary base. The system is inherently unstable, which makes it a challenging controls problem.

The goal of this project is to build a simple and affordable rotary inverted pendulum that can be used to teach control theory to students and hobbyists.

**Table of Contents**
- [Rotary Inverted Pendulum](#rotary-inverted-pendulum)
  - [Mechanics](#mechanics)
  - [Electronics](#electronics)
    - [Current Limiting](#current-limiting)
  - [Software](#software)
    - [Arduino](#arduino)
    - [Julia](#julia)
    - [Python](#python)
  - [Acknowledgments](#acknowledgments)

## Mechanics

The rotary inverted pendulum uses off-the-shelf components that are easy to source and assemble. The mechanical design is done in [Onshape](https://www.onshape.com/en/) and the STL files are available in the [meshes](meshes) folder. If you are looking for the actual project in Onshape, you can find it [here](https://cad.onshape.com/documents/fa8afe5031ca70c78442e408/w/5519455d45464bacd4cf9b1d/e/79273ac76c3305af463951de).

## Electronics

### Current Limiting

The Nema17 motor of the device is rated for 1 A, but we will use 0.9 A (10% lower) to be on the safe side. The current limit of the stepper driver is set using a potentiometer, and the formula to calculate the current limit is different for each driver:

| Driver  | Vref | Formula                                                                     |
| ------- | ---- | --------------------------------------------------------------------------- |
| A4988   | 0.72 | Vref = current limit * (8 x Rcs)                                            |
| DRV8825 | 0.45 | Vref = current limit / 2                                                    |
| TMC2209 | 0.66 | [TMC220X Vref Calculator](https://printpractical.github.io/VrefCalculator/) |

## Software

There are two ways to control the rotary inverted pendulum:

**1 – Using a microcontroller**  
In this case, the microcontroller is used to read sensor data and control the motors. The microcontroller runs a control algorithm that calculates the motor commands based on the sensor data. This architecture is simple and easy to implement, but it is limited in terms of computational power and flexibility.

**2 – Using a computer**  
In this case, the microcontroller is used to read sensor data and control the motors, while the computer is used to run the control algorithms. The microcontroller communicates with the computer over a serial connection, and the computer sends commands to the microcontroller to control the system. The computer also receives sensor data from the microcontroller and uses it to update the control algorithm. This architecture allows for real-time control of the system and makes it easy to experiment with different control algorithms.

| Processing Unit                  | Advantages                                | Disadvantages                                               |
| -------------------------------- | ----------------------------------------- | ----------------------------------------------------------- |
| Arduino Nano                     | Self-contained device.                    | Limited computational power and flexibility.                |
| Computer (via serial connection) | High computational power and flexibility. | Requires an external computer to be connected at all times. |

*— Do you want to take the device around and be able to run it without a computer?*  
Then you should aim to write a control algorithm that runs straight on the Arduino. You can use the code provided in this repository as a starting point.

*— Do you want to use your favourite programming language and are not concerned about portability?*  
In that case, you should connect the device to a computer and communicate with it over a serial connection — this is the approach used in the Julia and Python code examples provided in this repository.

### Arduino

You can find the Arduino code in the [RotaryInvertedPendulum-arduino](RotaryInvertedPendulum-arduino) folder. The code is written in C++ and is intended to be run on an Arduino Nano (onboard the device), but it can also be run on other Arduino boards if you want to tinker with and modify the device. The folder contains code for balancing the pendulum directly from the Arduino, but it also contains code for controlling the pendulum from a computer over a serial connection.

The Arduino code relies on the following two libraries:
- [AccelStepper](https://www.airspayce.com/mikem/arduino/AccelStepper/) for controlling the stepper motor.
- [AS5600](https://github.com/Seeed-Studio/Seeed_Arduino_AS5600) for reading the AS5600 magnetic encoder.

### Julia

You can find the Julia code in the [RotaryInvertedPendulum-julia](RotaryInvertedPendulum-julia) folder.

### Python

You can find the Python code in the [RotaryInvertedPendulum-python](RotaryInvertedPendulum-python) folder.

## Acknowledgments

I would like to thank the following people for their contributions to this project:
- [Mykha](https://github.com/Mika412) for early discussions about this project over a beer in the park.
- Vivek and [Vlad](https://github.com/VladimirIvan) for their feedback on the electronics.
- [心诺 (Xinnuo)](https://github.com/XinnuoXu) for her company and support while working on this project.
 
Finally, I would like to thank the open-source community in general for providing the tools and resources that have also helped make this project possible.
