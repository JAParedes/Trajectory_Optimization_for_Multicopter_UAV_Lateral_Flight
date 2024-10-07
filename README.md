# Trajectory Optimization for Quadcopter UAV Lateral Flight
Trajectory optimization for quadcopter UAV lateral flight demonstrated for flip maneuver during vertical and horizontal displacement. This code sets up and solves a nonlinear program (NLP) using CasADi. More details about the implementation are given in the **Time-optimal control for quadrotor lateral flight for benchmarking purposes** .pdf file, which provides a comparison of the proposed solution with the one shown in [1], by using the code provided in the Electronic Supplementary Material section in [this link](https://link.springer.com/article/10.1007/s10514-012-9282-3#Sec37).

## Setup for CasADi in Matlab for Windows
Download CasADi binaries from [this link](https://web.casadi.org/get/), unzip the files and place them in a directory of your choice. Then, in Matlab, run the following command depending on the file you downloaded.
```
addpath('<yourpath>/casadi-3.6.6)
```
In order to run this command each time Matlab starts, run
```
open startup.m
```
and paste `addpath('<yourpath>/casadi-3.6.6)` in the *startup.m* file.

## Demonstration Video

The Youtube video in this link (to be updated...) shows the results of each of the main files.

## Main File
* **Trajectory_Optimization_for_Quadcopter_UAV.m** : 

## Support Files
* **Create_Quadcopter_UAV_vertical_motion_animation.m** : 
* **Create_Quadcopter_UAV_horizontal_motion_animation** :

## Citing work

* **[1] Hehn, M., Ritz, R., & D’Andrea, R. (2012). Performance benchmarking of quadrotor systems using time-optimal control. Autonomous Robots, 33, 69-88.