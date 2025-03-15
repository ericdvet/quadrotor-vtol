# quadrotor-vtol

This project is a quadrotor simulation built on top of the UCSC ECE163 UAV Modeling and Control GUI by Maxwell Dunne and Gabe Elkaim. The simulation models the aerodynamics of an idealized X-4 Flyer II, based on the work of Pounds et al. (cited at the bottom).

## Group Members
Eric Vetha (evetha@ucsc.edu)
Evirpides Nicolaides (enicolai@ucsc.edu)
Dylan Brown (dwbrown@ucsc.edu)

## Features

- Flight Dynamics: Implements a simulation of the X-4 Flyer II quadrotor, including realistic aerodynamics.
- Graphical User Interface: Uses the UCSC ECE163 UAV Modeling and Control GUI for visualization and interaction.
- Flight Control System: A control system has been implemented, but it is currently non-functional.

## Usage

Clone the repository. Install necessary dependencies and run:

```bash
python .\QuadrotorSimulate.py
```

Enter number indicated for each test (1 for take off, 2 for hover, 3 for landing, 4 for complete test)
0. Free Mode - This puts the quadcopter in a state that allows for manual adjustment of each motor of the quadcopter. (motors 1/3 and motors 2/4 need to be spinning in opposite directions to ascend)
1. Take Off - The quadrotor will take off from ground and climb vertically upwards until controls give out.
2. Hover - Starting at a preset height, the quadrotor will maintain level flight within 0.1m.
3. Landing - Starting at a preset height, the quadrotor will descend and slow closer to the ground, then drop to the ground.

Once the mode has been selected, simply press the start button and watch the sim.

## Overview/State of Project
The project is very presentable in its current state. All current 4 demo modes work as intended and there's a host of error checking and exceptions to handle unprecidented input. It would be nice, for the final presentation, to combine our 3 preset demos into one, where we take off, hover at a preset height, then land smoothly, however we have not yet fully completed that demo. Some code is written for it, but currently is commented out to allow for the demos to run smoothly for a user. 

## Citations

```
@article{
    author = {P. Pounds and R. Mahony and P. Corke},
    title = {Modelling and control of a large quadrotor robot},
    journal = {Control Engineering Practice},
    volume = {18},
    number = {7},
    pages = {691--699},
    year = {2010}
}
```