# BnB-DAQP - Experiments

Code for the MIQP examples used in the paper "BnB-DAQP: A Mixed-Integer QP Solver for Embedded Applications"  

## Random MIQPs 
The random MIQP benchmark require:
* MATLAB
* Gurobi

## Inverted pendulum with contact forces 
Running the hybrid MPC example on a laptop requires :
* Julia
* [LinerMPC.jl](https://github.com/darnstrom/LinearMPC.jl)

A STM32CubeIDE project for benchmarking on a NUCLEO-64 F411RE board is also available. This requires, in addition, 

* Julia (for serial communication) 
* STM32CubeIDE
