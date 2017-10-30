# Kidnapped Vehicle Project

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

## Overview
This repository contains the work I did within **Project #8 of Udacity's Self-Driving Car Nanodegree Program** (third project of second term). Objective of this project is to **design a 2-D particle filter to localize a "kidnapped" vehicle**. Implementation is done in **C++. ** 
Vehicle state is defined by three coordinates: x, y and theta (yaw).
The particle filter is given a map with landmarks and some initial localization information (analogous to what a GPS would provide, accuracy ~0.3m). At each time step the filter gets noisy observations (which are the positions of  some of  the landmarks, as measured by a radar or a lidar) and noisy control data (velocity and yaw rate). This data is provided by a car driving simulator that Udacity has developed (it can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)).
The filter predicts the new position of the car using the control data, then updates the particle weights using the noisy observations, and finally resamples the particles.

## Repository content

The repository includes the following files:

 - source code in the */src* folder
	 - main.cpp : communicates with the Simulator, receiving observations and controls, runs the particle filter and calls the associated methods. 
	 - particle_filter.h & .cpp:  initializes the filter, defines the state prediction, weight update and resampling functions
 	 - helper_functions.h : useful functions
 	 - map.h: defines the Map class
 	 - json.hpp: JSON is used for communication with simulator
 - /data: contains _map_data.txt_ which includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns:    |   x position  |     y position |    landmark id

 - CMakeLists.txt: file that is used for compiling code

[Here](https://discussions.udacity.com/t/getting-started-with-docker-and-windows-for-the-ekf-project-a-guide/320236]) is a link which details how to have the simulator and code run with Docker on Windows.

## Communication protocol with simulator

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

* INPUTS: values provided by the simulator to the c++ program
 * sense noisy position data from the simulator (used for initialization)
		["sense_x"]
		["sense_y"]
		["sense_theta"]
 * get the previous velocity and yaw rate to predict the particle's transitioned state
	["previous_velocity"]
	["previous_yawrate"]
 * receive noisy observation data from the simulator, in a respective list of x/y values to update weights
	["sense_observations_x"]
	["sense_observations_y"]

* OUTPUTS: values provided by the c++ program to the simulator
	* best particle values used for calculating the error evaluation
	["best_particle_x"]
	["best_particle_y"]
	["best_particle_theta"]

## Success Criteria
The simulator includes a grading code. The things the grading code is looking for are:

1. **Accuracy**: the particle filter should localize vehicle position and yaw to within the values specified in the parameters `max_translation_error` and `max_yaw_error` in `src/main.cpp`.

2. **Performance**: the particle filter should complete execution within the time of 100 seconds.


## Results

The figure below represents the simulator. The car represents the true position of the tracked object. Lidar measurements are represented by red circles, radar measurements by blue circles with an arrow pointing in the direction of the observed angle, and UKF position estimation by green triangles. We can see that UKF makes a good job in filtering radar and lidar measurements.

![simulator](./simulator.png)

On the right and side of the simulator, values of RMSE (root mean square error) for position (x,y) and velocity (vx,vy) are displayed. Here they are the results of processing both radar and lidar measurements. 
I also provide the performance of the UKF with radar-only and  lidar-only measurements. Final RMSE are given in the table below:

|Type / RMSE|Px|Py |Vx |Vy|
|:--------:|:----:|:----:|:----: |:----: |
|Radar only|0.145|0.213|0.186|0.279|
|Lidar only|0.089|0.096|0.208|0.224|
|Radar+Lidar|0.065|0.083|0.159|0.212|

Below we recall the performance obtained with the EKF (cf. [P6-ExtendedKalmanFilter project](https://github.com/ThomasGrelier/Udacity-SDC-P6-ExtendedKalmanFilter)

|Type / RMSE|Px|Py |Vx |Vy|
|:--------:|:----:|:----:|:----: |:----: |
|Radar only|0.23|0.34|0.58|0.80|
|Lidar only|0.14|0.12|0.63|0.53|
|Radar+Lidar|0.09|0.09|0.45|0.43|

And in the table below we provide the accuracy improvement in %.

|Type / RMSE|Px|Py |Vx |Vy|
|:--------:|:----:|:----:|:----: |:----: |
|Radar only|37.0|37.4|68.0|65.0|
|Lidar only|36.4|20.0|67.0|58.7|
|Radar+Lidar|27.8|7.8|64.7|50.7|

We see that improvement is really significant, especially for the velocity with improvement up to 65%.
The added-value of UKF is more important on LIDAR-only or RADAR-only based KF than on mixed KF.


