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
- 

## Overview

<img width="732" alt="Blockdiagram" src="https://user-images.githubusercontent.com/103919092/174162631-dade012a-9629-4a09-9742-c6e1a0bfea97.PNG">

The power electronics project that [Christian Williams](https://github.com/cwill713) and I decided on was based around the specifications of new generation iPhone. We knew that Apple is picky with their products, so if we could get it to work with an iPhone it should, in theory, work with Android as well. I decided to use the [LM3478](https://www.ti.com/lit/ds/symlink/lm3478.pdf?HQS=dis-dk-null-digikeymode-dsf-pf-null-wwe&ts=1654180125781&ref_url=https%253A%252F%252Fwww.ti.com%252Fgeneral%252Fdocs%252Fsuppproductinfo.tsp%253FdistId%253D10%2526gotoUrl%253Dhttps%253A%252F%252Fwww.ti.com%252Flit%252Fgpn%252Flm3478) High-Efficiency Low-Side N-Channel Controller for our Switching Regulator. This is the heart of our circuit and allows for a reccomended 2.97-40V input volage and a peak driver output current of 1A. This chip can withstand a typical input current of 2.7A and a max of 3.3A. We used a 5W 12V Solar Panel to ensure that our smartphone would charge in average conditions.

## Rev 7 Design

<img width="814" alt="schematic2" src="https://user-images.githubusercontent.com/103919092/171946765-55e72f38-f459-4aac-95c0-715ab55abb05.PNG">

This is the most updated KiCad schematic for my circuit design. I used the Texas Instrument [Webench Power Designer](https://www.ti.com/design-resources/design-tools-simulation/webench-power-designer.html) to generate this step-down circuit. I used the majority of the components specified by the power designer with the exception of some values. These can be found in the [BOM](https://github.com/JoshuaMularczyk/Solar-Smartphone-Charger/tree/main/BOMs) I specified a 5V minimum as well as a 12V maximum based on what would most likely be coming out of the solar panel. I also specified 1A for my maximum output current and 5V for my output voltage. The efficiency of this circuit was given at 84.1% by Webench.

### Voltage Dividers for USB

<img width="231" alt="volatgefix2" src="https://user-images.githubusercontent.com/103919092/171946973-5ae8239b-4c4a-4da7-a379-1b64c578ddf0.PNG"><img width="638" alt="applereq" src="https://user-images.githubusercontent.com/103919092/171945713-fd5c0198-309d-4787-8156-a73bbe2ed5c6.PNG">

The voltages that needed to be applied to the Vbus, D+, and D- pin were found in the following websites: [Pinout for USB](http://static.righto.com/files/charger-schematic.pdf) and [Values based on different Ampheres](https://kb.plugable.com/usb-hubs-cables-switches/usb-device-charging). *Note since we are using 1A ouput the Vbus pin needs 5V, the D- pin needs 2V, and the D+ pin needs 2.7V*
## Rev 7 Simulation
I simulated our circuit in LTSpice

<img width="937" alt="labeledsim" src="https://user-images.githubusercontent.com/103919092/175172676-b8c4a63a-e1e4-4254-94d8-7d1da39b8684.PNG">

LTSpice has most of the components needed, but I had to import a library for the LM3478. I found this library in a [project log](https://hackaday.io/project/27899-nixie-tube-power-supply/log/69767-ltspice) by [Paul Andrews](https://github.com/judge2005) on Hackaday. That library was causeing some errors in LTSpice so I did some research and realized I needed to add two more "0" place holders after each "PULSE" statement. The updated library that I used LM3478 can be downloaded [here](https://github.com/JoshuaMularczyk/Solar-Smartphone-Charger/blob/main/Simulations/LM3478_TRANS.lib). The second library I needed was for the csd17553q5a mosfet. I found this library on the [Texas Instrument](https://www.ti.com/product/CSD17553Q5A#design-tools-simulation) page. This library can be downloaded [here](https://github.com/JoshuaMularczyk/Solar-Smartphone-Charger/blob/main/Simulations/CSD17553Q5A.lib). The schottkey diode used in the simulation is also not the exact same component as used in the schematic but another schottkey diode with the same specifications was used in its place. The simulation ran over night to get to 7 ms. The prolonged period of time was likely due to the laptop it was running on as well as the coupled inductors in the circuit.

### Voltage Outputs (5V 2.7V 2V)

<img width="960" alt="labeledvoltages" src="https://user-images.githubusercontent.com/103919092/175172787-74d416ad-9689-4476-a306-a50df229bb21.PNG">

This simulation shows the voltages going into each of our three USB output pins (5V, 2.7V, 2V).

### Input and Output Current

<img width="958" alt="currentsim" src="https://user-images.githubusercontent.com/103919092/171947246-d86cb5ba-57f3-49fe-8221-589e6aca34f5.PNG">

This simulation shows the two different currents in our circuit. The green trace represents the current going into the LM3478. It starts at about 270mA and trails off close to 0A. The blue trace represents the current being pulled from the 5-ohm load resistor which is taking the place of an iPhone. The iPhone needs to draw around 1A of current and as seen in the simulation it will achieve this.

## Rev 7 PCB Design

<img width="730" alt="pcb2" src="https://user-images.githubusercontent.com/103919092/171952596-56a447e4-df71-455a-acbc-5d2364d089b8.PNG">

We had our boards sourced from [JLCPCB](https://jlcpcb.com/VGS?utm_source=gg_vgs&utm_medium=cpc&gclid=Cj0KCQjwqPGUBhDwARIsANNwjV4Y9aU908uwwHsgXCAJ3L9PZ44l-hPgCvsU4kgto-ll1H0iRJroh1UaAsKwEALw_wcB), who have an offer going of 5, 2-layer, 100x100mm boards for $2. I then designed the board to fit these specifications and send it in for printing.

## Rev 6 Testing

My partner and I decided to construct a [Build and Test Plan](https://github.com/JoshuaMularczyk/Solar-Smartphone-Charger/blob/main/Milestone%20Reports/Build%20and%20Test%20Plan.xlsx) while we waited for our parts to come in from JLCPCB and Digikey. It helped us think through ways of testing things in stages without causing the destructions of parts.

<img width="500" alt="IMG-2079" src="https://user-images.githubusercontent.com/103919092/172071789-ddb3b23f-fdcf-4cd2-8f8e-7058f5be7122.png"><img width="500" alt="IMG-2080" src="https://user-images.githubusercontent.com/103919092/172071805-037e6bf5-900f-40e0-b412-12e99efecea5.png">



My partner and I each constructed a separate board and tested to see if it worked. After constructing the boards we used a 0-18V 1A power supply and realized that the boards were not allowing our iPhones to charge. We plugged in a nearby TI-84 Plus CE calculator that was rated to charge at 5V 0.5A and it started charging. We then decided to switch to a bigger power supply rated for 24V 3A and plugged in an Android which began to charge. The iPhone still would not charge. After some research about Apple devices, a fellow colleague, [Caydn Maddocks](https://github.com/Maddca) and I discovered that Apple was extra particular and required voltages to the D+ and D- pins of the USB connector. The specific voltages differ based on the current being drawn (we used 2.7V to D+ and 2V to D-). This can be seen in the "Voltage Dividers for USB" section above. The process of fixing this can be seen in the issues section [here](https://github.com/JoshuaMularczyk/Solar-Smartphone-Charger/issues?q=is%3Aissue+is%3Aclosed).

Next, we connected our 5W 12V solar panels to the converter and took them outside to get direct sunlight. Once the solar panel was propped up to be perpendicular to the sun, I plugged in my iPhone and it started charging. It takes quite a bit of time to charge with 5W so I recommend getting a higher wattage solar panel for faster charging.

## Rev 6 Results

The image on the left shows the voltage being supplied to the iPhone and the image on the right shows the current being drawn by the iPhone.

<img width="400" alt="IMG-1079" src="https://user-images.githubusercontent.com/103919092/172072088-5cb21dbd-36de-4d5a-aee9-864d0d37841f.jpg"><img width="400" alt="IMG-1081" src="https://user-images.githubusercontent.com/103919092/172072099-f12144a6-9be4-4a8b-8c2a-dc5363fb0343.jpg">

Here we can see that using the power supply, our circuits is around 93.7% efficient ((4.88V*0.96A)/(5W)). We also tested this circuit with voltages from 5-12V from the power supply and the output stayed at a constant 4.8-4.9V.

The Solar Smartphone Charger worked as expected and was able to charge smartphones, Apple and Android, during a sunny day. You can see more photos of the finished product on the [wiki](https://github.com/JoshuaMularczyk/Solar-Smartphone-Charger/wiki/Photos) here!

We measured a 4.74V 0.98A output from our circuit using a solar panel. This gives us an efficiency of 92.9% given that our solar panel was actually outputting 5W.

Click [here](https://youtu.be/SS7m2UqrDPg) to watch a video demonstration of the finished project!
