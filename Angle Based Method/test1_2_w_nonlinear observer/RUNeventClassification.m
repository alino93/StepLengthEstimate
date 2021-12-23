%% Run eventClassification v5 Integrate Step by Step, find climbing, Compare mountings
% Update: find delta y of more than 15 cm as climbing or stairs

% Integrate the acceleration of each foot to find instantaneous velocity
% with turning, standing, falling detection 
% Using euler angles we find the orientaion of coordinates with respect to 
% the intertial axes

g=9.8;
T=0.01; %period

%% Load and sync IMU data
nameR1='Right_2020-06-23T12.10.30.229_C361585F0624_Accelerometer.csv';
nameL1='Left_2020-06-23T12.10.30.229_F951B6EDAAE8_Accelerometer.csv';
nameC1='Chest_2020-06-23T12.10.30.229_DAB7F828D40E_Accelerometer.csv';
nameR2='Right_2020-06-23T12.10.30.229_C361585F0624_Gyroscope.csv';
nameL2='Left_2020-06-23T12.10.30.229_F951B6EDAAE8_Gyroscope.csv';
nameC2='Chest_2020-06-23T12.10.30.229_DAB7F828D40E_Gyroscope.csv';

[dataR, dataL, dataC] = dataProcess (nameR1,nameR2,nameL1,nameL2,nameC1,nameC2);

%remove initial drift (bias)
%dataR  =  dataR - dataR(1,:);
%dataL  =  dataL - dataL(1,:);
%dataC  =  dataC - dataC(1,:); %chest acc used for trunk direction detection

%time
t = (0:T:0.01*(length(dataR(:,1))-1))';

%accelerations
aRight = dataR(:,4:5)*g;
aLeft = dataL(:,4:5)*g;
aChest = dataC(:,4:6)*g;

aRight(:,1) = aRight(:,1);
aLeft(:,1) = aLeft(:,1);
aChest(:,3) = aChest(:,3);
aRight(:,2) = aRight(:,2);
aLeft(:,2) = aLeft(:,2)*-1;


%gyros
tiltRRate = dataR(:,12)* pi/180;
%tiltRRate = tiltRRate - mean(tiltRRate(1:1));
%tiltRRate = highpass(tiltRRate,0.1,100);
%tiltRRate = tiltRRate - mean(tiltRRate(1));
tiltLRate = dataL(:,12)*-1* pi/180;
%tiltLRate = tiltLRate - mean(tiltLRate(1:1));
%tiltLRate = highpass(tiltLRate,0.1,100);
%tiltLRate = tiltLRate - mean(tiltLRate(1));
pitchCRate = dataC(:,11)*pi/180;
yawCRate =  dataC(:,10)*-1;

for i = 1:1:length(t)-100
   yawCRate(i) = mean(yawCRate(i:i+100));
end
yawCRate = yawCRate - yawCRate(1);
yawC = cumtrapz(t,yawCRate);

for i = 1:1:length(t)-100
   pitchCRate(i) = mean(pitchCRate(i:i+100));
end
pitchCRate = pitchCRate - pitchCRate(1);
pitchC = cumtrapz(t,pitchCRate);
%% Finding events
eventClassification();

%% Bias calculation
lbound = 10; %region lower bound (deg)
ubound = 30; %region upper bound (deg)
sigma = 0.55; %convergance rate sigma/2
offset = 1; %switching offset (deg)
swbound = 5; %switching bound (deg)
SolveLMI2();
x_hat_R = Tilt_and_Bias_Estimation(aRight, tiltRRate,offset,swbound, L1,L2,L3);
x_hat_L = Tilt_and_Bias_Estimation(aLeft, tiltLRate,offset,swbound, L1,L2,L3);

BiasCal();

biasR = x_hat_R(2:3,:);%[0 0]';
biasL = x_hat_L(2:3,:);%[0 0]';
biasGR = x_hat_R(4,:);%0;
biasGL = x_hat_L(4,:);%0;
disp(['Right Bias (m/s^2):',num2str(mean(biasR'))]);
disp(['Left Bias  (m/s^2):',num2str(mean(biasL'))]);
disp(['Gyro Bias  (rad/s):',num2str(mean([biasGR,biasGL]))]);
% aRight = aRight - biasR';
% aLeft = aLeft - biasL';
aRight(1:end-115,:) = aRight(1:end-115,:) - biasR(:,116:end)';
aLeft(1:end-115,:) = aLeft(1:end-115,:) - biasL(:,116:end)';

tiltRRate = tiltRRate - biasGR';
tiltLRate = tiltLRate - biasGL';
%% Measured displacements
XR=[20 34 34 34 20 34 34 34 20 34 34 34 20 34 34 34];
XL=[34 34 34 20 34 34 34 20 35 34 34 20 34 34 34 20];
XR=XR*2.5/100;
XL=XL*2.5/100;
tR=ones(1,length(XR));
tL=ones(1,length(XL));

%% Tilt Angle EStimation 
K = 1;
% Right ankle tilt estimation
[tiltR, tiltRE] = Tilt_Estimation(aRight, tiltRRate, K);
tiltRE=x_hat_R(1,:);
% Left ankle tilt estimation
[tiltL, tiltLE] = Tilt_Estimation(aLeft, tiltLRate, K);
tiltLE=x_hat_L(1,:);
%% Transform acceleration w/o calibration
% Right ankle acceleration transform
biasR = [0 0];
aR = Transform_acc(aRight, tiltRE, biasR);

% Left ankle acceleration transform
biasL = [0 0];
aL = Transform_acc(aLeft, tiltLE, biasL);

%% Integrate w/o calibration
% find velocity and displacement w/o calibration
[vR, pR, tR, biasR, vL, pL, tL, biasL] = Integrate (aR, aL, tiltRE, tiltLE, XR, XL, events, t); % Use tiltE on all steps

%% Calibration
ERROR_R = zeros(1,30);
ERROR_L = zeros(1,30);
for j = 1:1:30
    K = j/10;
    % Right ankle tilt estimation
    [tiltR, tiltRE] = Tilt_Estimation(aRight, tiltRRate, K);
    tiltRE=x_hat_R(1,:);
    %tiltRE(6:end) = tiltRE(1:end-5)*1.3;
    % Left ankle tilt estimation
    [tiltL, tiltLE] = Tilt_Estimation(aLeft, tiltLRate, K);
    tiltLE=x_hat_L(1,:);
    % Right ankle acceleration transform
    biasR = [0 0];
    aR = Transform_acc(aRight, tiltRE, biasR);

    % Left ankle acceleration transform
    biasL = [0 0];
    aL = Transform_acc(aLeft, tiltLE, biasL);
    
    % find velocity and displacement w/o calibration
    [vR, pR, tR, biasR, vL, pL, tL, biasL] = Integrate(aR, aL, tiltRE, tiltLE, XR, XL, events, t);
    
    
    for n = 1:1:length(tR)
        pRmax = max(pR(1,floor(tR(n)*100-40):floor(tR(n)*100)+40));
        ERROR_R(j) = ERROR_R(j) + abs( (pRmax - XR(n)) ./ pRmax );
    end
    ERROR_R(j) = ERROR_R(j) / length(tR); 
    for n = 1:1:length(tL)
        pLmax = max(pL(1,floor(tL(n)*100-40):floor(tL(n)*100)+40));
        ERROR_L(j) = ERROR_L(j) + abs( (pLmax - XL(n)) ./ pLmax );
    end
    ERROR_L(j) = ERROR_L(j) / length(tL); 
end
[RE , KR] = min (ERROR_R);
[LE , KL] = min (ERROR_L);
KR = KR / 10;
KL = KL / 10;
%disp(['Mean Error Rate:',num2str(mean([RE,LE])*100)]);
% Right ankle tilt estimation
[tiltR, tiltRE] = Tilt_Estimation(aRight, tiltRRate, KR);
tiltRE=x_hat_R(1,:);
%tiltRE(6:end) = tiltRE(1:end-5)*1.3;
% Left ankle tilt estimation
[tiltL, tiltLE] = Tilt_Estimation(aLeft, tiltLRate, KL);
tiltLE=x_hat_L(1,:);
% Right ankle acceleration transform
biasR = [0 0];
aR = Transform_acc(aRight, tiltRE, biasR);

% Left ankle acceleration transform
biasL = [0 0];
aL = Transform_acc(aLeft, tiltLE, biasL);

% find velocity and displacement w calibration
[vR, pR, tR, biasR, vL, pL, tL, biasL] = Integrate(aR, aL, tiltRE, tiltLE, XR, XL, events, t);


%% Integrate w calibration
% %find velocity and displacement w calibration
% [vR, pR, tR, biasR, vL, pL, tL, biasL, XR, XL] = Integrate(aR, aL, tiltRE, tiltLE, XR, XL, events, t);

%% cal error
ERROR_R2 = zeros(1,1);
ERROR_L2 = zeros(1,1);
for n = 1:1:length(tR)
    pRmax = max(abs(pR(1,floor(tR(n)*100-40):floor(tR(n)*100)+40)));
    ERROR_R2 = ERROR_R2 + abs( (pRmax - XR(n)) ./ pRmax );
end
ERROR_R2 = ERROR_R2 / length(tR); 
for n = 1:1:length(tL)
    pLmax = max(abs(pL(1,floor(tL(n)*100-40):floor(tL(n)*100)+40)));
    ERROR_L2 = ERROR_L2 + abs( (pLmax - XL(n)) ./ pLmax );
end
ERROR_L2 = ERROR_L2 / length(tL);
disp(['Right Error Rate:',num2str(ERROR_R2(1)*100)]);
disp(['Left Error Rate: ',num2str(ERROR_L2(1)*100)]);
disp(['Mean Error Rate: ',num2str(mean([ERROR_R2(1),ERROR_L2(1)])*100)]);
%% Detect climbing events
%[Rsteps, Lsteps, events] = findClimbing(aR, aL, events, t);

%% plot results
figure(3)
subplot(2,2,2);
plot(t,aR(1,:));
title('Right foot horizontal acceleration');
xlabel('t (s)');
ylabel('a_x (m/s^2)');

subplot(2,2,4);
plot(t,aR(2,:));
title('Right foot vertical acceleration');
xlabel('t (s)');
ylabel('a_y (m/s^2)');

subplot(2,2,1);
plot(t,aL(1,:));
title('Left foot horizontal acceleration');
xlabel('t (s)');
ylabel('a_x (m/s^2)');

subplot(2,2,3);
plot(t,aL(2,:));
title('Left foot vertical acceleration');
xlabel('t (s)');
ylabel('a_y (m/s^2)');


figure(4)
subplot(1,2,2);
plot(t,pR1(1,:)*100,t,pR(1,:)*100,tR,XR*100,'ro','LineWidth',2);
title('Right foot horizontal displacement');
legend('Integration','Nonlinear observer','measured');
xlabel('t (s)');
ylabel('x (cm)');

% subplot(2,2,4);
% plot(t,pR(2,:)*100,'LineWidth',2);
% title('Right foot vertical displacement');
% xlabel('t (s)');
% ylabel('y (cm)');

subplot(1,2,1);
plot(t,pL1(1,:)*100,t,pL(1,:)*100,tL,XL*100,'ro','LineWidth',2);
title('Left foot horizontal displacement');
legend('Integration','Nonlinear observer','measured');
xlabel('t (s)');
ylabel('x (cm)');

% subplot(2,2,3);
% plot(t,pL(2,:)*100,'LineWidth',2);
% title('Left foot vertical displacement');
% xlabel('t (s)');
% ylabel('y (cm)');
[tiltR, tiltRE] = Tilt_Estimation(aRight, tiltRRate, 1);
[tiltL, tiltLE] = Tilt_Estimation(aLeft, tiltLRate, 1);
figure(10)
subplot(1,2,2);
plot(t,tiltR*180/pi,t,tiltRE*180/pi,'-.',t,x_hat_R(1,:)*180/pi,'--','LineWidth',2);
title('Right foot tilt angle(deg)');
legend('integrated value','Kalman Filter','Nonlinear Observer');
xlabel('t (s)');
ylabel('\theta (deg)');

subplot(1,2,1);
plot(t,tiltL*180/pi,t,tiltLE*180/pi,'-.',t,x_hat_L(1,:)*180/pi,'--','LineWidth',2);
title('Left foot tilt angle(deg)');
legend('integrated value','Kalman Filter','Nonlinear Observer');
xlabel('t (s)');
ylabel('\theta (deg)');

figure(5)
plot(t,aR(1,:),t,aL(1,:),'LineWidth',2);
title('foot horizontal acceleration');
legend('Right foot','Left foot');
xlabel('t (s)');
ylabel('a (m/s^2)');

figure(9)
subplot(1,3,2)
plot(0.1:0.1:3,ERROR_R);
title('Right foot Error vs K');
xlabel('K');
ylabel('Horiz Displacement Error (%)');
subplot(1,3,1)
plot(0.1:0.1:3,ERROR_L);
title('Left foot Error vs K');
xlabel('K');
ylabel('Horiz Displacement Error (%)');
subplot(1,3,3)
plot(0.1:0.1:3,mean([ERROR_L;ERROR_R],1));
title('Mean Error vs K');
xlabel('K');
ylabel('Horiz Displacement Error (%)');

figure(6)
plot(t,tiltLRate1*180/pi,t,tiltLRate2*180/pi,'-.',t,tiltLRate*180/pi,'--','LineWidth',2);
title('tilt rates');
legend('biased','bias from 0.1 sec mean','bias from 1 sec mean');
xlabel('t (s)');
ylabel('\theta^. (deg/s)');

figure(7)
plot(t,tiltLE1*180/pi,t,tiltLE2*180/pi,'-.',t,tiltLE*180/pi,'--','LineWidth',2);
title('tilt angle');
legend('biased','bias from 0.1 sec mean','bias from 1 sec mean');
xlabel('t (s)');
ylabel('\theta (deg)');

figure(8)
plot(t,tiltLRate1*180/pi,t,tiltLRate2*180/pi,'-.',t,tiltLRate*180/pi,'--','LineWidth',2);
title('tilt rates');
legend('Not filtered and biased','Not filtered and bias from 0.1s mean','Filtered and bias from 0.1s mean');
xlabel('t (s)');
ylabel('\theta^. (deg/s)');


% %% Posture Estimate Animation
% animationV2();