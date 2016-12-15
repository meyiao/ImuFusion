% This implements a zero velocity update EKF algorithm, which can be used
% for pedestrian tracking.
% By @TanTanTan
% Email: jinaqiaoqiao@gmail.com

clear; close all; clc;

% --- Dataset parameters
dt = 1/256;
g = 9.81;
sigma_acc  = 0.1;           % accelerometer noise (m/s^2)
sigma_gyro = 0.1*pi/180;    % gyroscope noise (rad/s)
sigma_vel = 0.01;           % zero-velocity update measurement noise (m/s)


%% zero-velocity detector parameters
cova  = 0.01^2; 
covw  = (0.1*pi/180)^2;     
W     = 5; % window size
gamma  = 0.3e5;

%% read data (Choose one)
% % data = load('data/StaightLine.mat');
% % data = load('data/StairsAndCorridor.mat');
data = load('data/SpiralStairs.mat');
imu_data = data.imu_data';
N = size(imu_data, 2);


%% Since we have got all the data, we can run the zero velocity detection
%%  for the whole dataset.
iszv = zeros(1, N);
T = zeros(1, N-W+1);
for k = 1:N-W+1
    mean_a = mean(imu_data(1:3,k:k+W-1), 2);
    for l = k:k+W-1
        temp = imu_data(1:3,l) - g * mean_a / norm(mean_a);
        T(k) = T(k) + imu_data(4:6,l)'*imu_data(4:6,l)/covw + temp'*temp/cova;
    end
end
T = T./W;
for k = 1:size(T,2)
    if T(k) < gamma
        iszv(k:k+W-1) = ones(1,W);
    end
end


%%
%=========================================================================%
%==                             Initialization                           =%
%=========================================================================%
% We require that the IMU to be at static for first one second. So we can
% estimate the initial roll/pitch angle, as well as the biases.
init_a = mean(imu_data(1:3,1:20),2);
init_a = init_a / norm(init_a);

init_psi =  0;
init_theta = -asin(init_a(1));
init_phi = atan2(init_a(2), init_a(3));

init_quat = angle2quat(init_psi, init_theta, init_phi);

% Estimate sensor bias.
Rsw = quat2dcm(init_quat);
as  = Rsw * [0;0;g];
bias_a = mean(imu_data(1:3,1:500),2) - as;
bias_w = mean(imu_data(4:6,1:500),2);

% set the initial state vector
x = zeros(10,1);
x(7:10,1) = init_quat';

% set the initial covariance
P = diag([1e-10*ones(1,6), 1e-6*ones(1,4)]);

%
x_r = zeros(10,N);
x_r(:,1) = x;

% measurement matrix
H = [zeros(3), eye(3), zeros(3,4)];
R =  sigma_vel^2 * eye(3);

%%
%=========================================================================%
%==                             Main  Loop                               =%
%=========================================================================%
for k = 2:N
    %% compute state transition matrix F and covariance Q
    w = imu_data(4:6, k-1); % - bias_w;
    quat = x(7:10,1);
    a = imu_data(1:3, k-1); % - bias_a;
    
    % continuous state transition matrix
    Ow = [0     -w(1)   -w(2)    -w(3);...
          w(1)   0       w(3)    -w(2);...
          w(2)  -w(3)    0        w(1);...
          w(3)   w(2)   -w(1)     0  ];
    Vq = compVq(quat, a);
    
    Fc = zeros(10);
    Fc(1:3, 4:6) = eye(3);
    Fc(4:10,7:10)= [Vq; 0.5*Ow];
    
    % continuous process covariance
    Gq = 0.5* [-quat(2)  -quat(3)   -quat(4); ...
                quat(1)  -quat(4)    quat(3); ...
                quat(4)   quat(1)   -quat(2); ...
               -quat(3)   quat(2)    quat(1)];       
    Qc = zeros(10);
    Qc(4:6, 4:6)  =  sigma_acc^2*eye(3);
    Qc(7:10,7:10) =  sigma_gyro^2*(Gq*Gq');
    
    % discretilization
    F = eye(10) + Fc* dt;
    Q = Qc* dt;
    
    %% state propagation
    R_S_n = quat2dcm(quat');
    acc = R_S_n' * a - [0; 0;  g];
    
    x(1:3) = x(1:3) + x(4:6)* dt + 0.5*acc* dt^2;
    x(4:6) = x(4:6) + acc* dt;
    
    quat = (eye(4) + 0.5*Ow* dt)*quat;
    quat = quat/norm(quat);
    x(7:10) = quat;
    
    %% covariance propagation
    P = F*P*F' + Q;

    %% zero-velocity update
    if iszv(k) == 1
        K = (P*H')/(H*P*H'+R);
        y = -x(4:6);
        
        x = x + K*y;
        x(7:10) = x(7:10)/norm(x(7:10));
        
        P = (eye(10)-K*H)*P;
    end
    
    P = (P+P')/2;
    
    x_r(:,k) = x;
      
end


%% View the results
%% Plot the trajectory
figure(1), hold on
plot(x_r(1,:),x_r(2,:));
plot(x_r(1,1),x_r(2,1),'ro');
legend('Trajectory','Start point')
axis equal
grid on

figure(2),
plot(1:N,x_r(3,:));
title('Height')
grid on



%% Animation
%% The animation code was stolen from xioTechnologies.
%% https://github.com/xioTechnologies/Gait-Tracking-With-x-IMU
L = size(x_r,2);
SamplePlotFreq = 4;
Spin = 120;
SixDofAnimation(x_r(1:3,:)', quat2dcm(quatinv(x_r(7:10,:)')), ...
                'SamplePlotFreq', SamplePlotFreq, 'Trail', 'All',...
                'Position', [9 39 1280 768],...
                'View', [(100:(Spin/(L-1)):(100+Spin))', 10*ones(L, 1)],...
                'AxisLength', 0.1, 'ShowArrowHead', false, ...
                'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)',... 
                'ShowLegend', false,...
                'CreateVideo', false);


