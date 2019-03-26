clc; clear all; close all;

g = 9.81;
dt = 0.01;

vx0 = 10;
vy0= 30;
vx = vx0;
vy = vy0;

x0 = [0; 0; vx; vy];        % state vector
A = [ 1 0 dt 0;
      0 1 0 0.5*dt;
      0 0 1 0;
      0 0 0 1]   ;
B = [ 0; 0; 0; -g*dt];

x = x0;
real = [];
last = 2*vx0*vy0/g;
for i=1:610
    x = A*x + B;
    if round(x(1),2) == round(last,2)
        break;
    end
    
    real = [real x];
end

figure; plot(real(1,:),real(2,:)); hold on

sigma_meas = 0.2;

z = real + randn(4, length(real))*sigma_meas;
plot(real(1,:), z(2,:));

%% Kalman

szum_sys = rand(1000,4)-0.5;
szum_pom = 10 * rand(1000,4)-5;

C = eye(4);
Q = diag(var(szum_sys,1));
R = diag(var(szum_pom,1));
P = 1 * eye(4);

x_hat = x0;
x1 = [];

for i=1:610
    % % predykcja / propagation / projection to k+1
    % state propagation
    x_hat = A * x_hat + B;
    
    % covariance propagation
    P = A * P * A' + Q;
    
    % % update
    % innovation or measurement residual
    y_hat = z(:,i) - C * x_hat;
    
    % innovation or residual covariance
    S = C * P * C' + R;
    
    % Kalman gain
    K = P * C' * inv(S);
    
    % state estimation update
    x_hat = x_hat + K * y_hat;  
    
    x1 = [x1 x_hat];
    
end

figure; plot(real(1,:), x1(2,:), real(1,:), real(2,:))
