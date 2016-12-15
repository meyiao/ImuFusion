% This is an Extended Kalman Filter which fuses 
% accelerometer & gyroscope readings
% to get the orientation.
% Written by @TanTanTan

% Note that we assume the biases of IMU do not change in the whole
% process. This is approximately true when the sensors operate in a steady
% state.

clear; close all; clc;

%--- Dataset parameters
deltat = 1/200;             % Sampling period
noise_gyro = 2.4e-3;        % Gyroscope noise(discrete), rad/s
noise_accel = 2.83e-2;      % Accelerometer noise, m/s^2
gravity = 9.81007;          % Gravity magnitude, m/s^2

%--- Load data
% Each row of IMU data : 
% (timestamp, wx, wy, wz, ax, ay, az)
% Each row of Ground truth data : 
% (time, position, quaternion, velocity, gyroscope bias, accelerometer bias)
data = load('data/attitude_data.mat');
imu_data = data.imu_data;   % IMU readings
grt_data = data.grt_data;   % Ground truth (GT)
grt_q = grt_data(:, 5:8);   % GT quaternions

bias_w = grt_data(1, 12:14);    % gyroscope bias
bias_a = grt_data(1, 15:17);    % accelerometer bias


%--- Container of the results
N = length(imu_data);
allX = zeros(N, 4);


%--- Initialization
x = grt_q(1,:)';            % Initial state (quaternion)
P = 1e-10 * eye(4);         % Initial covariance
allX(1,:) = x';


% ---Here we go !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
for k = 2 : N
    
    %--- 1. Propagation --------------------------
    % Gyroscope measurements
    w = (imu_data(k-1, 2:4) + imu_data(k, 2:4))/2;
    w = w - bias_w;
    
    % Compute the F matrix
    Omega =[0     -w(1)   -w(2)   -w(3);...
            w(1)  0       w(3)    -w(2); ...
            w(2)  -w(3)   0        w(1); ...
            w(3)  w(2)    -w(1)    0  ];
    F = eye(4) + deltat * Omega / 2;
    
    % Compute the process noise Q
    G = [-x(2)  -x(3)  -x(4); ...
          x(1)  -x(4)   x(3); ...
          x(4)   x(1)  -x(2); ...
         -x(3)   x(2)   x(1)] / 2;
    Q = (noise_gyro * deltat)^2 * (G * G');
    
    % Propagate the state and covariance
    x = F * x;
    x = x / norm(x);    % Normalize the quaternion
    P = F * P * F' + Q;
    
    
    %--- 2. Update----------------------------------
    % Accelerometer measurements
    a = imu_data(k, 5:7);
    a = a - bias_a;
        
    % We use the unit vector of acceleration as observation
    ea = a' / norm(a);
    ea_prediction = [2*(x(2)*x(4)-x(1)*x(3)); ...
                     2*(x(3)*x(4)+x(1)*x(2)); ...
                     x(1)^2-x(2)^2-x(3)^2+x(4)^2];
    
    % Residual
    y = ea - ea_prediction;
    
    % Compute the measurement matrix H
    H = 2*[-x(3)    x(4)    -x(1)   x(2); ...
            x(2)    x(1)     x(4)   x(3); ...
            x(1)   -x(2)    -x(3)   x(4)];
        
    % Measurement noise R
    R_internal = (noise_accel / norm(a))^2 * eye(3);
    R_external = (1-gravity/norm(a))^2 * eye(3);
    R = R_internal + R_external;
    
    % Update
    S = H * P * H' + R;
    K = P * H' / S;
    x = x + K * y;
    P = (eye(4) - K * H) * P;
    
    
    % 3. Ending
    x = x / norm(x);    % Normalize the quaternion
    P = (P + P') / 2;   % Make sure that covariance matrix is symmetric
    allX(k,:) = x';     % Save the result
    
end


%--- Compare the results with ground truth
q_Ws0 = quatinv(grt_q(1,:));
for i=1:N
    grt_q(i,:) = quatmultiply(q_Ws0, grt_q(i,:)); 
    allX(i,:) = quatmultiply(q_Ws0, allX(i,:));
end
[psi, theta, phi] = quat2angle(allX);
[grt_psi, grt_theta, grt_phi] = quat2angle(grt_q);

figure, hold on
plot(1:N, psi,   'r-.', 1:N, grt_psi,   'r');
plot(1:N, theta, 'g-.', 1:N, grt_theta, 'g');
plot(1:N, phi,   'b-.', 1:N, grt_phi,   'b');

    
