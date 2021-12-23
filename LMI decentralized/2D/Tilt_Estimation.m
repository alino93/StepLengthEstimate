function [tiltE, rollE] = Tilt_Estimation(a_body, tiltRate, rollRate, K)
%% Kalman Filter Angle Estimation
T = 0.01;
g = sqrt( a_body(1,1)^2 +  a_body(1,2)^2 +  a_body(1,3)^2);

%initial mounting angle in degs
tilt0 = sign(a_body(1,1))*acos( a_body(1,3)/g);
roll0 = atan2(a_body(1,2),a_body(1,3));

%time
t = 0:T:T*(length(tiltRate)-1);

%% Filter data
%low pass filter 4Hz and correct scale and direction
a_body  = lowpass(a_body, 4, 100);

%% Without KF
tilt = cumtrapz(t,tiltRate);
tilt = tilt + tilt0;

%% Kalman Filter
tiltE = zeros(1,length(t));
tiltE(1) = tilt0;
rollE = zeros(1,length(t));
rollE(1) = roll0;
for i=2:1:length(t)
    ge = sqrt(a_body(i,1)^2 + a_body(i,2)^2 + a_body(i,3)^2);
    tilt_acc = atan2(a_body(i,1),sqrt(a_body(i,2)^2+a_body(i,3)^2));%sign(a_body(i,1))*acos(a_body(i,2)/ge);
    tiltE(i) = tiltE(i-1) + (tiltRate(i) + K * (tilt_acc - tiltE(i-1))) * T;
    roll_acc = atan2(a_body(i,2),a_body(i,3));
    rollE(i) = rollE(i-1) + (rollRate(i) + K * (roll_acc - rollE(i-1))) * T;
end
