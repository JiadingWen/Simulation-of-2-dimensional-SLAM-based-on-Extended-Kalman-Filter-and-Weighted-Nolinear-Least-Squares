# Simulation of 2 dimensional Simultaneous Localization And Mapping based on Extended Kalman Filter and Weighted Nolinear Least Squares

|![image](https://github.com/JiadingWen/Simulation_2D_SLAM/blob/master/img/ScreenShot1.gif)|![image](https://github.com/JiadingWen/Simulation_2D_SLAM/blob/master/img/ScreenShot2.gif)|
| - | :-: |

<p align="center">Blue circle is real pose of the robot, red circle is estimated pose of the robot</p>
<p align="center">Two blue stars are real position of the features, two red stars are estimated position of the features</p>

***

## Introduction

Simulating a 2D map with two features and a robot that rotating around feature 1 in matlab. (The robot can observe angle and distance of two features relative to itself). 

Using observation data and control data to estimate the pose of the robot and the position of two features based on **Extended Kalman Filter (EKF)** and **Weighted Nolinear Least Squares (WNLS)** respectively (i.e. ~~solving a simple 2D SLAM by EKF and WNLP~~).

> [**Simultaneous Localization And Mapping**](http://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) (SLAM): is the computational problem of constructing or updating a map of an unknown environment while simultaneously keeping track of an agent's location within it. 

## Prerequisites

All codes have **Only** been tested on 
* Windows10 1809 
* Matlab R2018b 

There is no guarantee that the codes have good compatibility in other versions. 

## Usage
Double-click `F00_Main_EKF.m` to run simulation of 2D SLAM based on EKF.

Double-click `F00_Main_NLS.m` to run simulation of 2D SLAM based on WNLS.

***
That‘s simple~ Enjoy playing robot!!

:star: Star us on GitHub — it helps! 