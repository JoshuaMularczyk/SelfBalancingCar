# Self Balancing Car

This project is being conducted by [Joshua Mularczyk](https://github.com/JoshuaMularczyk) and [Cadyn Maddocks](https://github.com/maddca) for [Dr. Rob Frohne](https://github.com/frohro). The project features control systems design of a self balancing car (inverted pendulum on wheels) for ENGR 454 Control Systems.

## Objectives:
- Decide what software to work with (C++, Python, Arduino IDE, etc)
- Build Two-wheeled Cart
- Test Hardware
- Find and measure all constants
- Develop Linear Small Angle State Space Model
- Hand Place Poles
- Implement LQR
- Implement a Full Order Observer
- Implement a Kalman Filter

## Overview/Process

<img width="443" alt="BalancingCart" src="https://github.com/JoshuaMularczyk/SelfBalancingCar/assets/103919092/2ed0406f-5f06-400a-8936-b1c7c9a10daf">

This project uses state space control to balace a two wheeled cart. [Cadyn Maddocks](https://github.com/maddca) and I started by determining the characteristic equations that describe the model. We used these equations to then find our A, B, C, and D matrices for our state space model. The next step of the process was to make sure the system was controllable and observable. This was accomplished by comparing the rank of the A matrix in the controllable and observable forms to the number of state variables.

Next, Simulations were conducted which can be viewed in more detail below. Once we were confident enough in our simulations we moved to Visual Studio Code to communicate with our ESP32 board using C++. Results can be seen below.

## Calculations

### Constants

- Total wheel mass: 98.2 g

- Total Body mass: 717.7 g

- Center of Mass: 1.7 cm up from x axis : L in model

- Moment of Inertia of wheels: 2.79639E-5            (Iw = 0.5*mw*r*r)

- Moment of Inertia of body: 1.7630E-4	      (Ib = 0.85*mb*L*L)

- wheel diameter: 6.75 cm

- wheel radius: 3.375 cm

- width of wheel: 2.5 cm

- Motor Constant: 0.0298                           (back emf with drill test)

- Resistance: 4.6 ohms

- Gravity: 9.81 m/s


### Matrices

<img width="344" alt="ABCD" src="https://github.com/JoshuaMularczyk/SelfBalancingCar/assets/103919092/6b419bfb-7297-45be-adaf-2b8ed78176c8">

## Initial Testing

- Beeping Test

- Encoder Test

- MPU6050 Test

- 

## Simulations

### Placing Poles



### LQR

### Full Order Observer

## Results


