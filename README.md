# Unscented Kalman Filter Project
Udacity Self-Driving Car Engineer Nanodegree Program: Term 2
[Master project repo](https://github.com/udacity/CarND-Unscented-Kalman-Filter-Project)

This project implements an "unscented" [Kalman filter](https://www.seas.harvard.edu/courses/cs281/papers/unscented.pdf) (UKF) that uses lidar and radar measurements to track a moving vehicle. Because two types of sensor measurements are used, it is an example of [sensor fusion](https://en.wikipedia.org/wiki/Sensor_fusion). The UKF is an improvement on the Extended Kalman Filter (EKF), since it is better able to handle non-linear system dynamics. (Apparently the name is a joke about how the EKF "stinks").

This project uses the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

## Project structure

The project consists of the following primary components:
* src/main.cpp - Responsible for interfacing with the simulator and reading (simulated) lidar and radar measurement data
* src/ukf.cpp, .h - Responsible for implementing the UKF and holding all values associated with it
* src/tools.cpp, .h - Responsible for calculation of RMSE

## Kalman filter concepts

In this application we use a mathematical model of the motion of the tracked vehicle that assumes it is moving with a constant turn rate and speed (the magnitude of velocity), a.k.a. CTRV. This leads us to using a 5-dimensional state space to describe the vehicle (x and y position, velocity magnitude, yaw angle, and yaw angle rate of change). The equations of motion in this situation are non-linear, as is the coordinate system of one of our sensors (radar). The EKF we used previously does not handle this nonlinearity well.

The UKF is a computationally efficient way of dealing with the nonlinearities. It begins by defining "sigma points", which are representative points in phase space that we will propagate through the equations of motion to estimate how the probability distribution of the vehicle's state changes in time. We use two sigma points for each dimension of the problem, plus an additional one identical to the current estimated state. In order to handle the nonlinear process noise we "augment" the problem with two additional dimensions (addressing linear and yaw angle acceleration noise). 

The function ProcessMeasurement() initializes the UKF with the first measurement, and then handles each successive measurement by predicting the current state and updating our state belief using the algorithm appropriate for the relevant sensor. While the radar measurements need to be handled with the UKF, the lidar measurements can be handled with a simple linear Kalman filter, which is less computationally expensive.

The prediction step propagates the sigma points through the equations of motion and extracts an estimated covariance and predicted state; the update step re-uses the sigma points to project into the measurement coordinate space and calculate the state and covariance update.

## Using NIS to tune noise parameters

A key part of the project is tuning the noise parameters for linear and yaw acceleration. We can do this with trial-and-error, using the RMSE that results from the UKF output in the simulator and the ground truth. A helpful method for this is to calculate the "normalized innovation squared" (NIS) - essentially what we learn on average from each measurement, or equivalently how much we have to correct our estimated state. 

I adjusted these parameters by first choosing physically reasonable numbers and then tuning them to find RMSE improvements. For linear acceleration it's reasonable to choose a value less than 1g (9.8 m/s^22) since most cars won't have acceleration noise that large. For yaw acceleration it's reasonable to choose a relatively small fraction of 2pi/s^2, since most cars won't swing their front rapidly through a full circle in that amount of time. The values I searched for and ultimately chose are listed below.


| Lin. ac. noise. (m/s^2) | Yaw ac. noise. (rad/s^2) | RMSE(x) | RMSE (y) | RMSE(vx) | RMSE(vy) | NIS(L) | NIS(R) |
|:--------------------:|:---------------------:|:-------:|:--------:|:--------:|:--------:|:------:|:------:|
| 3.0				   | 0.30 				   | 0.0786  | 0.0851   | 0.3307   | 0.3067   | 0.0103 | 0.0145 |
| 4.5				   | 0.30 				   | 0.0807  | 0.0873   | 0.3490   | 0.3283   | 0.0105 | 0.0143 |
| 1.5				   | 0.30 				   | 0.0741  | 0.0822   | 0.3100   | 0.2836   | 0.0100 | 0.0155 |
| 1.0				   | 0.30 				   | 0.0712  | 0.0816   | 0.3037   | 0.2761   | 0.0097 | 0.0161 |
| 0.5				   | 0.30 				   | 0.0665  | 0.0839   | 0.2997   | 0.2710   | 0.0093 | 0.0166 |
| 1.0				   | 0.15 				   | 0.0892  | 0.0919   | 0.3460   | 0.3196   | 0.0143 | 0.0154 |
| 1.0				   | 0.60 				   | 0.0668  | 0.0818   | 0.2937   | 0.2669   | 0.0083 | 0.0164 |
| 1.0				   | 1.00 				   | 0.0660  | 0.0830   | 0.2973   | 0.2726   | 0.0080 | 0.0164 |
| 2.0				   | 0.60 				   | 0.0720  | 0.0824   | 0.3055   | 0.2797   | 0.0090 | 0.0157 |

Conclusion of optimal parameters: linear ac.: 1.0 m/s^2; yaw ac.:: 0.6 rad/s^2.
