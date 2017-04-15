# Unscented Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

# Project Overview
The goals / steps of this project are the following:
* Complete the Unscented Kalman Filter algorithm in C++.
* Apply it to pedestrian position/speed estimation problem 
* Evaluate it again two sampled data, the metrics used is RMSE


[//]: # (Image References)
[yaw_angle_est](https://cloud.githubusercontent.com/assets/24623272/25064435/73380324-222c-11e7-9fe9-a8ffac8e67bc.png)
[nis_lidar](https://cloud.githubusercontent.com/assets/24623272/25064437/733936e0-222c-11e7-9011-44699643fc50.png)
[nis_radar](https://cloud.githubusercontent.com/assets/24623272/25064436/7338870e-222c-11e7-9507-069ee1311e09.png)
[position_est](https://cloud.githubusercontent.com/assets/24623272/25064439/735b856a-222c-11e7-91b1-3f0c3a0cb1c5.png)
[velocity_est](https://cloud.githubusercontent.com/assets/24623272/25064440/736032ae-222c-11e7-957e-e5e2106c159a.png)
[rmse_result](https://cloud.githubusercontent.com/assets/24623272/25064479/66c7f864-222d-11e7-9b0d-fcd6f040b12f.JPG)

## 1. Final Result

#### 1.1. sample data 1

Accuracy - RMSE:  
0.0824666
0.0878453
 0.644505
 0.584015

#### 1.2. sample data 2

Accuracy - RMSE: 
0.15118 
0.189515 
0.237983 
0.287951

#### 1.3 Output
![rmse_result](https://cloud.githubusercontent.com/assets/24623272/25064479/66c7f864-222d-11e7-9b0d-fcd6f040b12f.JPG)

Note please check the notebook (ekf-visualization.ipynb) for details.

## 2. Related Charts

#### 2.1. NIS Lidar
![nis_lidar](https://cloud.githubusercontent.com/assets/24623272/25064437/733936e0-222c-11e7-9011-44699643fc50.png)

#### 2.2. NIS Radar
![nis_radar](https://cloud.githubusercontent.com/assets/24623272/25064436/7338870e-222c-11e7-9507-069ee1311e09.png)

#### 2.3. position estimation 
![position_est](https://cloud.githubusercontent.com/assets/24623272/25064439/735b856a-222c-11e7-91b1-3f0c3a0cb1c5.png)

#### 2.4. velocity estimation 
![velocity_est](https://cloud.githubusercontent.com/assets/24623272/25064440/736032ae-222c-11e7-957e-e5e2106c159a.png)

