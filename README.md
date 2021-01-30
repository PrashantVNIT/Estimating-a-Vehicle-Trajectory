# Estimating a Vehicle Trajectory
In this we are recursively estimating the position of a vehicle along a trajectory using available measurements and a motion model.
## Data known beforehand/Assumptions...
The vehicle is equipped with a very simple type of LIDAR sensor, which returns range and bearing measurements corresponding to individual landmarks in the environment. The global positions of the landmarks are assumed to be known beforehand. We will also assume known data association, that is, which measurment belong to which landmark.

## Provided Dataset:

| Variable | Description|
|-----------|------------|
| **t** | timestamps for the measurements provided|
| **x_init** | initial x position of the bot|
| **y_init** | initial y position of the bot|
| **th_init** | initial &theta; of the bot|
| **v** | Odometry reading of the velocity|
| **v_var** | Variance in the odometry reading of the velocity|
| **om** | Odometry Reading of the angular velocity|
| **om_var** | Variance in the odometry reading of the angular velocity|
| **l** | Location of the landmarks [a b]. Where a is the x coordinate of the landmark and b is the y coordinate of the landmark|
| **d** | Distance between the center of the bot and the LiDAR sensor|
| **b** | Bearing measurement provided by the LiDAR sensor|
| **b_var** | Variance in the Bearing measurement of the LiDAR sensor|
| **r** | Range measurement provided by the LiDAR sensor|
| **r_var** | Variance in the Range measurement provided by the LiDAR sensor|

## Motion and Measurement Models

### Motion Model
The vehicle motion model recieves linear and angular velocity odometry readings as inputs, and outputs the state (i.e., the 2D pose) of the vehicle:

![Screenshot (464)](https://user-images.githubusercontent.com/71186496/106354335-1a34d600-6317-11eb-91de-839fac81ff62.png)

### Measurement Model

![Screenshot (465)](https://user-images.githubusercontent.com/71186496/106354358-494b4780-6317-11eb-9d08-4fa4882345c0.png)

## Getting Started

Since the models are nonlinear, we are using the Extended Kalman Filter (EKF) as the state estimator. The EKF consists of following two steps:

- The prediction step, which uses odometry measurements and the motion model to produce a state and covariance estimate at a given timestep, and
- The correction step, which uses the range and bearing measurements provided by the LIDAR to correct the pose and pose covariance estimates

