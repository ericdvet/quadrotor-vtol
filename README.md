# quadrotor-vtol

This project is a quadrotor simulation built on top of the UCSC ECE163 UAV Modeling and Control GUI by Maxwell Dunne and Gabe Elkaim. The simulation models the aerodynamics of an idealized X-4 Flyer II, based on the work of Pounds et al. (cited at the bottom).

## Features

- Flight Dynamics: Implements a simulation of the X-4 Flyer II quadrotor, including realistic aerodynamics.
- Graphical User Interface: Uses the UCSC ECE163 UAV Modeling and Control GUI for visualization and interaction.
- Flight Control System: A control system has been implemented, but it is currently non-functional.

## Usage

Clone the repository. Install necessary dependencies and run:

```bash
python .\QuadrotorSimulate.py
```

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