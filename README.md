# Rotary Inverted Pendulum

**Table of Contents**
- [Rotary Inverted Pendulum](#rotary-inverted-pendulum)
  - [Current Limiting](#current-limiting)
  - [Acknowledgments](#acknowledgments)

## Current Limiting

For the Nema17 motors that we are using, which are rated for 1 A but we will use 0.9 A (10% lower) just to be a bit conservative...

| Driver  | Vref |
| ------- | ---- |
| A4988   | 0.72 |
| DRV8825 | 0.45 |
| TMC2209 | 0.66 |

- A4988: Vref = Current Limit * (8 x Rcs)
- DRV8825: Vref = Current limit / 2
- TMC2209: https://printpractical.github.io/VrefCalculator/

## Acknowledgments

Thank you to Mykha for early discussions about this project over a beer in the park.
Thank you to Vivek and Vlad for feedback on the electronic design.
Thank you to Xinnuo for all the company and support while working on this.
