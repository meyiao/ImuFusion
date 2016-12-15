# ImuFusion
## EKF IMU Fusion Algorithms
1. orien.m uses Kalman filter for fusing the gyroscope's and accelerometer's readings to get the IMU's attitude(quaternion).<br>
2. zupt.m implenments the so called 'zero-velocity-update' algorithm for pedestrian tracking(gait tracking), it's also a ekf filter.<br>
* Video: 


## Usage
Example data already included. Simply run the orien.m or zupt.m. For zupt, set 'CreateVideo' as true if you'd like to save the results in a video.<br>

## References:
[1] S. Madgwick. An efficient orientation filter for inertial and inertial/magnetic sensor arrays.<br>
[2] Fischer C, Sukumar P T, Hazas M. implementing a Pedestrian Tracker Using inertial Sensors.<br>
[3] Isaac Skog, Peter Händel, John-Olof Nilsson, and Jouni Rantakokko. Zero-Velocity Detection — An Algorithm Evaluation.<br>
