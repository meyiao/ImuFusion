# ImuFusion
## EKF IMU Fusion Algorithms
1. orien.m uses Kalman filter for fusing the gyroscope's and accelerometer's readings to get the IMU's attitude(quaternion).<br>
2. zupt.m implenments the so called 'zero-velocity-update' algorithm for pedestrian tracking(gait tracking), it's also a ekf filter.<br>
* Video: http://v.youku.com/v_show/id_XMTgzOTgzMjMyOA==.html?spm=a2hzp.8244740.userfeed.5!2~5~5~5!2~A


## Usage
Example data already included.<br>
Simply run the orien.m or zupt.m. For zupt, set 'CreateVideo' as true if you'd like to save the results in a video.<br>
Note that the datasets and the code for visualizing the results were from:
https://github.com/xioTechnologies/Gait-Tracking-With-x-IMU

## References:
[1] S. Madgwick. An efficient orientation filter for inertial and inertial/magnetic sensor arrays.<br>
[2] Fischer C, et. implementing a Pedestrian Tracker Using inertial Sensors.<br>
[3] Isaac Skog, et. Zero-Velocity Detection — An Algorithm Evaluation.<br>
