# Model-Predictive-Control-cpp-python
This repository contains some of my work on MPC in both cpp and python.

# The Problem

Here we try to design a system that solves perhaps the most cliche MPC problem, which is to avoid an obstacle. The goal for the agent is to avoid a stationary obstacle while minimizing the amount of fuel required to reach its target. The fuel is here, implicitly considered in the model equations, i.e., we assume that is the length of the trajectory is minimized, we are also minimizing the fuel usage.

# The cost Function

Here I have tried to use a different type of cost function, which I've found is easier to converge even with relatively simple optimization algorithms. In fact in the c++ code of this project the optimizer is a simple gradient descent algorithm where the gradient is calculated numerically. In the python part of the codebase, the optimizer used is a global optimization algorithm (dual annealing from the scipy optimize package). 

The cost function is defined as :<br />
![alt text](https://github.com/NonStopEagle137/Model-Predictive-Control-cpp-python/blob/main/Images/general_cost.png?raw=true)
<br />
where alpha controls the depth of the global minima and beta controls the overall influence/spread of the local minima. The value of alph can be positive or negative depending upon the desired action by the agent. For instance, a positive value of alpha generates a localised peak in the function landscape and a negative alpha generates a trough in the the functional landscape.<br />
gamma is the euclidean distance from the obstacle/target to the current position. <br />
![alt text](https://github.com/NonStopEagle137/Model-Predictive-Control-cpp-python/blob/main/Images/gamma.png?raw=true)
<br />
This Cost function works well because of it's excellent quality of superimposition. Obstacles can be denoted as localised peaks in a global trough with the target position as the global minima.
# Reference Functional Landscape in 2-D
Following is the functional landscape created using the cost function. The upward peak denotes the obstacle which is in an overall inward trough which leads to the global minima. <br />
![alt text](https://github.com/NonStopEagle137/Model-Predictive-Control-cpp-python/blob/main/Images/functional_landscape.jpeg?raw=true)
# Mathematical Model

The mathematical model used in this particular project is a very simple one. we assume a vehicle travelling toward a target position while avoiding a stationary obstacle. The cost function used in this project, however allows for moving obstacles as well. we assume a timestep delta_t = 1. Hence in this problem it follows that, <br />
![alt text](https://github.com/NonStopEagle137/Model-Predictive-Control-cpp-python/blob/main/Images/model_1.png?raw=true)
<br />
The direction of movement in the 2-D plane is denoted by theta. Theta is essentially an implicitly considered term, as at every timestep we generate offsets for delta_x and delta_y.

# Update

The offset discussed before are the updates made to the current position of the vehicle. they are defined by the following equations.
<br />
![alt text](https://github.com/NonStopEagle137/Model-Predictive-Control-cpp-python/blob/main/Images/update.png?raw=true)
<br />

# License

You are not allowed to use the content of this repository in direct or implied form for any commercial activity without my consent. If you plan to use the contents for academic research please contact me at athrva98@gmail.com


