function [x_hat] = Tilt_and_Bias_Estimation(a_body, tiltRate, L)
%% Kalman Filter Angle Estimation
T = 0.01;
g = 9.81;
ge = sqrt( a_body(1,1)^2 +  a_body(1,2)^2);
%initial mounting angle in degs
tilt0 = sign(a_body(1,1))*acos( a_body(1,2)/ge);

%time
t = 0:T:T*(length(tiltRate)-1);

%% Filter data
%low pass filter 1Hz and correct scale and direction
a_body = lowpass(a_body, 0.001, 100);
a_body = a_body';
%% Observer
x_hat = zeros(4,length(t));
x_hat(1,1) = tilt0;
% x_hat(:,1) = [1;1;1;1];
A = [0 0 0 1;0 0 0 0;0 0 0 0;0 0 0 0];
B = [1;0;0;0];
C = [0 1 0 0;0 0 1 0];
for i=2:1:length(t)
    h = [g*sin(x_hat(1,i-1));g*cos(x_hat(1,i-1))];
    x_hat(:,i) = x_hat(:,i-1) + (A*x_hat(:,i-1) + B*tiltRate(i-1) +...
        L * (a_body(:,i-1) - C*x_hat(:,i-1) - h)) * T;
end
