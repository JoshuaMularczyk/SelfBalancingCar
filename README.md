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

- Total wheel mass: 0.0982 Kg

- Total Body mass: 0.7177 Kg

- Center of Mass: 0.017 m up from x axis : L in model

- Moment of Inertia of wheels: 2.79639E-5 Kg*m^2    (Iw = 0.5*mw*r*r)

- Moment of Inertia of body: 1.7630E-4	Kg*m^2      (Ib = 0.85*mb*L*L)

- wheel diameter: 0.0675 m

- wheel radius: 0.03375 m

- width of wheel: 0.025 m

- Motor Constant: 0.1037 Nm/A      

- Resistance: 4.6 ohms

- Gravity: 9.81 m/s


### Matrices

<img width="444" alt="ABCD" src="https://github.com/JoshuaMularczyk/SelfBalancingCar/assets/103919092/6b419bfb-7297-45be-adaf-2b8ed78176c8">

## Initial Testing

- Beeping Test: Performed a beeping test to confirm the board was functional.
- Encoder Test: Tested the Hall Effect Encoders on the motors to determine a constant to convert the encoder value into degrees. 
- MPU6050 Test: Tested the MPU6050 to see if the output of the accelerometers and gyros made sense. Compared to the results of using the Kalman Filter to determine whether the Kalman filter was working as expected.
- BackEMF Test: Determined the motor constant.
<img width="363" alt="image" src="https://github.com/JoshuaMularczyk/SelfBalancingCar/assets/103593959/2635ab06-9021-46cf-889b-f92fbc837fbf">


## Simulations

### Placing Poles

This simulation uses our matrices and python's ct.place function to get a feedback matrix that can be used to get our pwm values.

<img width="434" alt="PlacingPOles" src="https://github.com/JoshuaMularczyk/SelfBalancingCar/assets/103919092/34e420e2-c010-4fbc-bf32-ea4ae69e7005">

### LQR

This simulation was a more fine tuned way of finding our poles and proved to be better overall then handplacing poles. It uses a Q variable to determine the impact that it has on position and an R variable to determine the impact that the PWM value has on the system. The overall equation is seen below:

![image](https://github.com/JoshuaMularczyk/SelfBalancingCar/assets/103919092/2069e01a-2389-497a-bf15-6237f45341b7)

Below is a simulation of the output after LQR was applied:

<img width="460" alt="LQR2rev" src="https://github.com/JoshuaMularczyk/SelfBalancingCar/assets/103919092/35af9c7b-5848-4a01-a169-a6739c43ff6b">

### Full Order Observer

The full order observer creates a seperate observable system that interacts with the actual system using a few matrices: u, G, and K. The full order oberver simulation below also shows that error goes to zero over time.

<img width="470" alt="FULLORDEROBSrev2PNG" src="https://github.com/JoshuaMularczyk/SelfBalancingCar/assets/103919092/163d0e8c-4230-4fd0-8dd7-9adf1dcc011e">

## Results

The results of this project are not quite where we want them to be yet. The cart has balanced for at most 3 seconds so far. There is still a constant shake as seen in the [results video](https://github.com/JoshuaMularczyk/SelfBalancingCar/wiki/Videos), and after about 2 correction, the system falls down to one side and cannot correct itself. Significant progess has been made over the time of a few months and we believe that we may need to add more to our system model. 

The best results occured while using LQR for our poles and the Kalman filter for our system. The full order observer results in high PWM values constantly.


