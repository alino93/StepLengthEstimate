function [tiltE,rollE] = Tilt_Estimation3D(a_body, tiltRate, rollRate, K)
%% Kalman Filter Angle Estimation
T = 0.01;
g = sqrt( a_body(1,1)^2 +  a_body(1,2)^2 + a_body(1,3)^2 );

%initial mounting angle in degs
tilt0 = sign(-1*a_body(1,1))*acos( sqrt( a_body(1,2)^2 + a_body(1,3)^2 )/g );
roll0 = sign(a_body(1,2))*acos( a_body(1,3)/cos(tilt0)/g );

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
rollE = zeros(1,length(t));
tiltE(1) = tilt0;
rollE(1) = roll0;
for i=2:1:length(t)
    ge = sqrt( a_body(i-1,1)^2 +  a_body(i-1,2)^2 + a_body(i-1,3)^2);
    tilt_acc = sign(-1*a_body(i-1,1))*acos( sqrt( a_body(i-1,2)^2 + a_body(i-1,3)^2 )/ge );
    roll_acc = sign(a_body(i-1,2))*acos( a_body(i-1,3)/cos(tilt_acc)/ge );
    
    tiltE(i) = tiltE(i-1) + (tiltRate(i) + K * (tilt_acc - tiltE(i-1))) * T;
    rollE(i) = rollE(i-1) + (rollRate(i) + K * (roll_acc - rollE(i-1))) * T;
end
