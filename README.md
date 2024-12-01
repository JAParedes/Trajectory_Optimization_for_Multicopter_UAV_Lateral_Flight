# Trajectory Optimization for Multicopter UAV Lateral Flight
Trajectory optimization for multicopter UAV lateral flight demonstrated for flip maneuver during vertical and horizontal displacements. This code sets up and solves a nonlinear program (NLP) using the CasADi software toolbox. More details about the implementation formulation are given in the **Time-optimal trajectory planning for multicopter lateral flight.pdf** file, which provides a comparison of the proposed solution with the one shown in [1], by using the code provided in the Electronic Supplementary Material section in [this link](https://link.springer.com/article/10.1007/s10514-012-9282-3#Sec37).

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

The Youtube video in this link (to be updated...) shows the results of each of the main files. The slides used for the video are shown in the **Time-optimal trajectory planning for multicopter lateral flight_Presentation.pdf** file. (to be added...)

## Main Files
* **Trajectory_Optimization_for_Multicopter.m** : Sets up and solves the trajectory optimization problem NLP using the CasADi software toolbox considering the multicopter lateral flight dynamics shown in [1].
* **Trajectory_Optimization_for_Multicopter_w_Rot_Dyn.m** : Sets up and solves the trajectory optimization problem NLP using the CasADi software toolbox considering multicopter lateral flight dynamics that include rotational dynamics.

## Support Files
* **plot_optimal_states_and_inputs.m** : Plots the optimal state and input trajectories over time, using the .mat files that can be obtained from the main files.
* **Create_Multicopter_vertical_motion_animation.m** : Creates an animation for the vertical displacement with a flip optimal maneuver, using the .mat files that can be obtained from the main files.
* **Create_Multicopter_horizontal_motion_animation.m** : Creates an animation for the horizontal displacement with a flip optimal maneuver, using the .mat files that can be obtained from the main files.

## Citing work

* **[1] Hehn, M., Ritz, R., & D’Andrea, R.** (2012). Performance benchmarking of quadrotor systems using time-optimal control. Autonomous Robots, 33, 69-88.