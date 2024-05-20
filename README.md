# Rotary Inverted Pendulum

The rotary inverted pendulum is a classic control problem that is used to demonstrate the principles of control theory. The system consists of a pendulum that is mounted on a rotary base, and the goal is to balance the pendulum in the upright position by controlling the rotary base. The system is inherently unstable, which makes it a challenging controls problem.

The goal of this project is to build a simple and affordable rotary inverted pendulum that can be used to teach control theory to students and hobbyists.

**Table of Contents**
- [Rotary Inverted Pendulum](#rotary-inverted-pendulum)
  - [Mechanics](#mechanics)
  - [Electronics](#electronics)
    - [Circuit Diagram](#circuit-diagram)
    - [Current Limiting](#current-limiting)
  - [Software](#software)
    - [Arduino](#arduino)
    - [Julia](#julia)
    - [Python](#python)
  - [Related Work](#related-work)
  - [Purchasing Options](#purchasing-options)
  - [Acknowledgments](#acknowledgments)

## Mechanics

The rotary inverted pendulum uses off-the-shelf components that are easy to source and assemble. The mechanical design is done in [Onshape](https://www.onshape.com/en/) and the STL files are available in the [meshes](meshes) folder. If you are looking for the actual project in Onshape, you can find it [here](https://cad.onshape.com/documents/fa8afe5031ca70c78442e408/w/5519455d45464bacd4cf9b1d/e/79273ac76c3305af463951de).

## Electronics

There are two models of the rotary inverted pendulum: one with batteries and one without. The model with batteries is more portable and can be used without any cables, while the model without batteries requires a power supply to be connected at all times. However, both devices require a USB cable if you want to control them from a computer.

### Circuit Diagram

| System without batteries                                                  | System with batteries                                                     |
| ------------------------------------------------------------------------- | ------------------------------------------------------------------------- |
| ![diagrams/system-with-batteries.jpg](diagrams/system-with-batteries.jpg) | ![diagrams/system-with-batteries.jpg](diagrams/system-with-batteries.jpg) |

The **battery management system (BMS)** is a circuit that manages the batteries in the device. It performs the following functions:
- balance charging (ensures that all cells in the battery pack are charged to the same voltage)
- short-circuit protection (prevents the batteries from being damaged in case of a short circuit)
- overcharge protection (prevents the batteries from being overcharged)
- overdischarge protection (prevents the batteries from being overdischarged)

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

## Related Work

Here are some related projects that you might find interesting:

- https://build-its-inprogress.blogspot.com/2016/08/desktop-inverted-pendulum-part-2-control.html
- https://www.dagor.dev/blog/furuta-pendulum
- https://www.youtube.com/watch?v=bY4t6yfBA24
- https://journals.sagepub.com/doi/full/10.1177/00202940211035406
- https://tecsolutions.us/sites/default/files/quanser/The%20Rotary%20Control%20Lab%20Brochure_4.pdf
- https://www.youtube.com/watch?v=2koXcs0IhOc
- https://www.youtube.com/watch?v=VVQ-PGfJMuA
- https://build-its-inprogress.blogspot.com/search/label/Pendulum

## Purchasing Options

|            Description | Price (USD) | Link                                                                                                                             |
| ---------------------: | :---------: | :------------------------------------------------------------------------------------------------------------------------------- |
|   AliExpress – DIY Kit | $100 ~ $200 | [aliexpress.com/w/wholesale-rotary-inverted-pendulum.html](https://www.aliexpress.com/w/wholesale-rotary-inverted-pendulum.html) |
| Quanser – QUBE Servo 2 |  > $2,000   | [quanser.com/products/qube-servo-2](https://www.quanser.com/products/qube-servo-2)                                               |

## Acknowledgments

I would like to thank the following people for their contributions to this project:
- [Mykha](https://github.com/Mika412) for early discussions about this project over a beer in the park.
- [André](https://github.com/Esser50K), [Rafael](https://github.com/rkourdis), and [Vlad](https://github.com/VladimirIvan) for technical discussions, feedback, and support.
- [Vivek](https://github.com/svrkrishnavivek) for his invaluable help and feedback on the electronics of the system.
- [心诺 (Xinnuo)](https://github.com/XinnuoXu) for her company and support while working on this project.
 
Finally, I would like to thank the open-source community in general for providing the tools and resources that have also helped make this project possible.
