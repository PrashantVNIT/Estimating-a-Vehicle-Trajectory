# Estimating a Vehicle Trajectory
In this we are recursively estimating the position of a vehicle along a trajectory using available measurements and a motion model.
## Data known beforehand/Assumptions...
The vehicle is equipped with a very simple type of LIDAR sensor, which returns range and bearing measurements corresponding to individual landmarks in the environment. The global positions of the landmarks are assumed to be known beforehand. We will also assume known data association, that is, which measurment belong to which landmark.

## Motion and Measurement Models

### Motion Model
The vehicle motion model recieves linear and angular velocity odometry readings as inputs, and outputs the state (i.e., the 2D pose) of the vehicle:

![Screenshot (457)](https://user-images.githubusercontent.com/71186496/106164207-be9c0880-61af-11eb-976b-8709fb971afb.png)

