# CarND-Controls-MPC
Below is a description of my submission for the MPC project.

## Highlights of my submission
1. Reaches a top speed of 90mph.
2. Could reach a top speed of 100mph while staying on the track and looping continuously (however, it may get a little close to comfort to the ledge/curb).
3. Accounts for latency in an intelligent manner in both the MPC and the preprocessing.

## The Model
The state consists of the vehicle's x and y coordinates, its current velocity, the bearing angle apart from cross track error (CTE) and psi error. The latter two are derived quantities. The outputs of the MPC are the acceleration and steering angle. The model considers the previous state and obtains the current state by adjusting for each state variable's update in the time that has passed.
 
 1. x<sub>t+1</sub> = x<sub>t</sub> + v<sub>t</sub> * cos(\psi<sub>t</sub>) * dt
 

## Timestep length and Elapsed Duration (N & dt)
## Polynomial Fitting and MPC Preprocessing
## Model Predictive Control with Latency

