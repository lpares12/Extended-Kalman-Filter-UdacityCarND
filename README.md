#Extended Kalman Filter Project

This project is part of the Self-Driving car nanodegree program by udacity. It implements an Extended Kalman Filter for a self driving car in C++.

We receive LIDAR and Rada sensor data through WebSockets from the [term 2 simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/v1.45) and we apply the Kalman Filter Predict and Update to send back the result to the simulator.

## Compilation
Compilation is done by creating a new directory called `build` and executing `cmake .. && make` inside of it.

More info can be found in the [base code repository](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project/).
