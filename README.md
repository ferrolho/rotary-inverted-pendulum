# Rotary Inverted Pendulum

> [!WARNING]
> Documentation and software are still a work in progress. Hardware (mechanical design and electronics) are mostly finished.

The rotary inverted pendulum is a classic control problem that is used to demonstrate the principles of control theory. The system consists of a pendulum that is mounted on a rotary base, and the goal is to balance the pendulum in the upright position by controlling the rotary base. The system is inherently unstable, which makes it a challenging controls problem.

The goal of this project is to build a simple and affordable rotary inverted pendulum that can be used to teach control theory to students and hobbyists.

**Table of Contents**
- [Rotary Inverted Pendulum](#rotary-inverted-pendulum)
  - [Mechanics](#mechanics)
    - [3D Printing](#3d-printing)
  - [Electronics](#electronics)
    - [Circuit Diagram](#circuit-diagram)
    - [Time Estimates:](#time-estimates)
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

### 3D Printing

I have been printint all the pieces in my [Bambu Lab A1 mini](https://bambulab.com/en-gb/a1-mini) — which at the time of me writing this costs £170, and it is great value for the money. I have had success printing in both PLA and PETG using [Bambu Studio](https://bambulab.com/en/download/studio) with the default print settings and with 'Tree' support for the enclosure and arm STL files. When printing the pendulum, add a pause on layer 21 — this is the time when you should place a 2 pence coin (or other similarly sized coin) in the slot, before the printer resumes to cover the slot for the weight. As for the lid STL, I have been printing it in two parts so I can manually change the filament for having a different colour for the 45-degree angle indicator lines. I will add more detailed instructions about the 3D printing here in the future, and maybe even make a video of the printing process for all the parts required.

## Electronics

There are two models of the rotary inverted pendulum: one with batteries and one without. The model with batteries is more portable and can be used without any cables, while the model without batteries requires a power supply to be connected at all times. However, both devices require a USB cable if you want to control them from a computer.

### Circuit Diagram

| System without batteries                                       | System with batteries                                       |
| -------------------------------------------------------------- | ----------------------------------------------------------- |
| <img src="diagrams/system-without-batteries.jpg" height="600"> | <img src="diagrams/system-with-batteries.jpg" height="600"> |

> [!CAUTION]
> The circuit diagram above for the system with batteries is not correct and needs updating. The BMS needs to be capable of managing 3 cells and the batteries required are 3 not 2. In general, the system with batteries is just a prototype at this stage; I have not tried to put the electronics together yet, and the 3D printed parts are also not designed with the extra storage required for holding the batteries and BMS.

The **battery management system (BMS)** is a circuit that manages the batteries in the device. It performs the following functions:
- balance charging (ensures that all cells in the battery pack are charged to the same voltage)
- short-circuit protection (prevents the batteries from being damaged in case of a short circuit)
- overcharge protection (prevents the batteries from being overcharged)
- overdischarge protection (prevents the batteries from being overdischarged)

I have spent countless hours debugging prototype boards that I put together when building the first few versions of these pendulum. At first, I used a wire that was too thick (24 AWG) and then I switched to a thinner wire (30 AWG). The jump in size was a lot more than I expected — but then again, I had no prior experience or notion about the AWG scale. My first protoboard using only the thick wire (24 AWG) worked first try, but then I realised the components were not placed correctly for it to fit properly into the enclosure while accomodating space in the correct places for the device's switch and DC jack. As such, I prepared another protoboard, this time with the header pins positioned correctly; however, after turning the pendulum on,it could not balance properly and the boards (both the Arduino and stepper motor driver) started overheating. I burned through 2 Arduino Nano until I was convinced that there was actualy something wrong. I tested the connectivities again, and everything checked out, so I didn't know what was going on or how to debug it. Thus, I started anew from a new protoboard and tried again. Similarly, nothing worked even though the connectivity with the multimeter checked out. I realised that making boards from scratch until one of them worked was not a viable and sane path, so I picked the latest board — which was looking quite good! — and decided to first try and get one of the subsystems working before moving to the next; I started with the magnetic encoder. I wrote a small test script so I could plot the signal via the serial on the Arduino IDE plotter. As I suspected, the signal was terrible and nonsensical. At this point, I had no idea why that was. I tried debraiding the wires, but that didn't work. I picked up a new magnetic encoder and tried soldering new wires, but it didn't work. I braided the cables again, and then tested, and then debraided them again. In the end, I somehow tried soldering the wire points on the magnetic encoder (which are really tiny) from both front and back — and voila! That did the trick! Now I have a very solid signal, stable, not noisy — and I can also confirm that you can braid the wires, as that was not the culprit after all. Now that the sensing subsystem is working, let's move on to debugging the actuation subsystem.

Tip - It is better to cut the wires slightly longer than you think they'll need to be rather than cutting them exactly the length they need to be (or shorter!) and then struggle with the soldering task.

### Time Estimates:

**1h–1.5h**
- Solder male pins to the Arduino Nano
- Solder header pins to the protoboard
- Solder connection wires on the protoboard


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
- https://build-its-inprogress.blogspot.com/search/label/Pendulum
- https://journals.sagepub.com/doi/full/10.1177/00202940211035406
- https://tecsolutions.us/sites/default/files/quanser/The%20Rotary%20Control%20Lab%20Brochure_4.pdf
- https://www.dagor.dev/blog/furuta-pendulum
- https://www.youtube.com/watch?v=2koXcs0IhOc
- https://www.youtube.com/watch?v=bY4t6yfBA24
- https://www.youtube.com/watch?v=VVQ-PGfJMuA

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
