# Model-Predictive-Control-cpp-python
This repository contains some of my work on MPC in both cpp and python. The codes here are far from any production readiness and are simply meant as demonstrations.

# The Problem

Here we try to design a system that solves perhaps the most cliche MPC problem, which is to avoid an obstacle. The goal for the agent is to avoid a stationary obstacle while minimizing the amount of fuel required to reach its target. The fuel is here, implicitly considered in the model equations, i.e., we assume that is the length of the trajectory is minimized, we are also minimizing the fuel usage.

# The cost Function

Here I have tried to use a different type of cost function, which I've found is easier to converge even with relatively simple optimization algorithms. In fact in the c++ code of this project the optimizer is a simple gradient descent algorithm where the gradient is calculated numerically. In the python part of the codebase, the optimizer used is a global optimization algorithm (dual annealing from the scipy optimize package). 

The cost function is defined as :

