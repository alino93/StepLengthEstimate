%% Run eventClassification v6 Find Steps and Events (Integrate Step by Step, find climbing)
% same as posture estimation but with  Nonlinear Observer instead of KF

g=9.8;
T=0.01; %period

%% Load and sync IMU and IR data
date='2020-08-10T07.59.14.787';
nameRS1=strcat('Right Shank_',date,'_F77B017786FA_Accelerometer.csv');
nameRS2=strcat('Right Shank_',date,'_F77B017786FA_Gyroscope.csv');
nameLS1=strcat('Left Shank_',date,'_EBEF7988970B_Accelerometer.csv');
nameLS2=strcat('Left Shank_',date,'_EBEF7988970B_Gyroscope.csv');
nameRT1=strcat('Right Thigh_',date,'_C361585F0624_Accelerometer.csv');
nameRT2=strcat('Right Thigh_',date,'_C361585F0624_Gyroscope.csv');
nameLT1=strcat('Left Thigh_',date,'_F951B6EDAAE8_Accelerometer.csv');
nameLT2=strcat('Left Thigh_',date,'_F951B6EDAAE8_Gyroscope.csv');
nameC1=strcat('Chest_',date,'_EF5CCAED2A08_Accelerometer.csv');
nameC2=strcat('Chest_',date,'_EF5CCAED2A08_Gyroscope.csv');
[dataR, dataL, dataRT, dataLT, dataC] = dataProcess3 (nameRS1,nameRS2,nameLS1,nameLS2,nameRT1,nameRT2,nameLT1,nameLT2,nameC1,nameC2);
%[dataR, dataRT, dataC] = dataProcess (nameRS1,nameRS2,nameLS1,nameLS2,nameRT1,nameRT2,nameLT1,nameLT2,nameC1,nameC2);

%% IR data extraction
optitrackExtract();
%time
t = (0:T:0.01*(length(dataR(:,1))-1))';
%position
XArIR = interp1(tO,XArIR,t);
XAlIR = interp1(tO,XAlIR,t);
YArIR = interp1(tO,YArIR,t);
YAlIR = interp1(tO,YAlIR,t);
%velocity
VXArIR = [0;diff(XArIR)/T];
VXAlIR = [0;diff(XAlIR)/T];
VYArIR = [0;diff(YArIR)/T];
VYAlIR = [0;diff(YAlIR)/T];
%acceleration
aXArIR = [0;diff(VXArIR)/T];
aXAlIR = [0;diff(VXAlIR)/T];
aYArIR = [0;diff(VYArIR)/T];
aYAlIR = [0;diff(VYAlIR)/T];
%angle
tiltIRr = interp1(tO,tiltIRr,t);
tiltIRl = interp1(tO,tiltIRl,t);

%filtering
aXArIR = lowpass(aXArIR,4,100);
aXAlIR = lowpass(aXAlIR,4,100);
aYArIR = lowpass(aYArIR,4,100);
aYAlIR = lowpass(aYAlIR,4,100);

%smoothing
% aXArIR = smooth(aXArIR,100);
%% IMU data
%accelerations
aRight = dataR(:,4:5)*g;
aLeft = dataL(:,4:5)*g;
aThighR = dataRT(:,4:5)*g;
aThighL = dataLT(:,4:5)*g;
aChest = dataC(:,4:6)*g;

aRight(:,1) = dataR(:,4)*g;
aRight(:,2) = dataR(:,5)*g;
aLeft(:,1) = dataL(:,4)*g;
aLeft(:,2) = dataL(:,5)*g;
aThighR(:,1) = dataRT(:,4)*g;
aThighR(:,2) = dataRT(:,5)*g;
aThighL(:,1) = dataLT(:,4)*g;
aThighL(:,2) = dataLT(:,5)*g;
aChest(:,3) = aChest(:,3);

%gyros
tiltRRate = dataR(:,12)*  pi/180;
tiltRRate = tiltRRate - mean(tiltRRate(1:1));
tiltLRate = dataL(:,12)* pi/180;
tiltLRate = tiltLRate - mean(tiltLRate(1:1));
tiltRTRate = dataRT(:,12)* pi/180;
tiltRTRate = tiltRTRate - mean(tiltRTRate(1:1));
tiltLTRate = dataLT(:,12)* pi/180;
tiltLTRate = tiltLTRate - mean(tiltLTRate(1:1));
pitchCRate = dataC(:,11)*pi/180;
pitchCRate = pitchCRate - mean(pitchCRate(1:1)); 
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

%% Bias calculation
lbound = 5; %region lower bound (deg)
ubound = 30; %region upper bound (deg)
sigma = 0.45; %convergance rate sigma/2
offset = 1; %switching offset (deg)
swbound = 2; %switching bound (deg)
SolveLMI2();

x_hat_R = Tilt_and_Bias_Estimation(aRight, tiltRRate,offset,swbound, L1,L2,L3);
x_hat_L = Tilt_and_Bias_Estimation(aLeft, tiltLRate,offset,swbound, L1,L2,L3);
x_hat_RT = Tilt_and_Bias_Estimation(aThighR, tiltRTRate,offset,swbound, L1,L2,L3);
x_hat_LT = Tilt_and_Bias_Estimation(aThighL, tiltLTRate,offset,swbound, L1,L2,L3);

tiltRE = x_hat_R(1,:);
tiltLE = x_hat_L(1,:);
tiltRTE = x_hat_RT(1,:);
tiltLTE = x_hat_LT(1,:);

%bias from observer
biasR = x_hat_R(2:3,:);%[0 0]';
biasL = x_hat_L(2:3,:);%[0 0]';
biasRT = x_hat_RT(2:3,:);%[0 0]';
biasLT = x_hat_LT(2:3,:);%[0 0]';
biasGR = x_hat_R(4,:);%0;
biasGL = x_hat_L(4,:);%0;

%bias from IR
biasRx = [-.0; -0];
aRIR = Transform_acc(aRight, tiltIRr*pi/180, biasRx');
aLIR = Transform_acc(aLeft, tiltIRl*pi/180, biasRx');
biasRIR = aRIR'-[aXArIR,aYArIR];
biasLIR = aLIR'-[aXAlIR,aYAlIR];

disp(['Right Shank Bias (m/s^2):',num2str(mean(biasR'))]);
disp(['Left Shank Bias  (m/s^2):',num2str(mean(biasL'))]);
disp(['Right Thigh Bias (m/s^2):',num2str(mean(biasRT'))]);
disp(['Left Thigh Bias  (m/s^2):',num2str(mean(biasLT'))]);
disp(['Gyro Bias  (rad/s):',num2str(mean([biasGR,biasGL]))]);
% aRight = aRight - biasR';
% aLeft = aLeft - biasL';
dt=115;
aRight(1:end-dt,:) = aRight(1:end-dt,:) - biasR(:,dt+1:end)';
aLeft(1:end-115,:) = aLeft(1:end-115,:) - biasL(:,116:end)';

tiltRRate = tiltRRate - biasGR';
tiltLRate = tiltLRate - biasGL';
%% Tilt Angle EStimation 
KR = 0.8;
% Right ankle tilt estimation
tiltRKE = Tilt_Estimation(aRight, tiltRRate, KR);

KL = 0.8;
% Left ankle tilt estimation
tiltLKE = Tilt_Estimation(aLeft, tiltLRate, KL);

KT = 1;
% Right thigh tilt estimation
tiltRTKE = Tilt_Estimation(aThighR, tiltRTRate, KT);

KT = 1;
% Left thigh tilt estimation
tiltLTKE = Tilt_Estimation(aThighL, tiltLTRate, KT);

KC = 2;
% Left thigh tilt estimation
tiltCE = Tilt_Estimation([aChest(:,3) aChest(:,1)], pitchCRate, KC);

%% Transform acceleration w calibration
% Right ankle acceleration transform
biasRx = [-.0; -0];
%biasR = mean(biasRy,2);
aR = Transform_acc(aRight, tiltRE, biasRx');

% Left ankle acceleration transform
biasLx = [-0; 0];
%biasL = mean(biasLy,2);
aL = Transform_acc(aLeft, tiltLE, biasLx');

%% Finding events
eventClassification();
%% Integrate w calibration
% find velocity and displacement w/o calibration
[tR, vR, pR, tL, vL, pL, biasRy, biasLy] = Integrate(aR, aL, tiltRE, tiltLE, events, t); % Use tiltE on all steps

%% Find position from angles
Len = length(t);
i = 1;

Ll=0.9;
Ls=0.43;
Lt=Ll-Ls;
Lb=0.85;

tiltR = tiltRE - mean(tiltRE(1:100));
tiltL = tiltLE - mean(tiltLE(1:100));
tiltTr = tiltRTE - mean(tiltRTE(1:100));
tiltTl = tiltLTE - mean(tiltLTE(1:100));
tiltC = tiltCE - mean(tiltCE(1:100));

str=find(events(:,2)==11,2,'first');
edr=find(events(:,2)==12,2,'first');
% tiltTl = tiltL;
XAr = zeros(Len,1);
YAr = XAr;
XAl = XAr;
YAl = XAr;
XKr = XAr;
YKr = XAr;
XKl = XAr;
YKl = XAr;
XH = XAr;
YH = XAr;
XC = XAr;
YC = XAr;
XAR = XAr;
XAL = XAr;
XARIR = XAr;
XALIR = XAr;

% %f=fit(t(str(2):edr(2))',tiltTr(str(2):edr(2))','poly3','Normalize','on','Robust','Bisquare');
%  ss=880;%str(2);
%  ee=988;
%  f=polyfit(ss:ee,tiltTr(ss:ee),3);
% % plot(t(str(2):edr(2)),polyval(f,t(str(2):edr(2))))
% % tiltTl(1:970) = interp1(t(str:edr),tiltTr(str:edr),tiltL(1:970),'spline','extrap');
str = find(events(i:end,2)==11,1,'first') + i-1;        
stl = find(events(i:end,3)==21,1,'first') + i-1;
edr = find(events(i:end,2)==12,1,'first') + i-1;        
edl = find(events(i:end,3)==22,1,'first') + i-1;
while i<Len
    i = i + 1;
    if i==edr
        str = find(events(i:end,2)==11,1,'first') + i-1;
        edr = find(events(i+1:end,2)==12,1,'first') + i;
        if isempty(str) || isempty(edr)
            str=inf;
            edr=inf;
        end
    end
    if i==edl
        stl = find(events(i:end,3)==21,1,'first') + i-1;
        edl = find(events(i+1:end,3)==22,1,'first') + i;
        if isempty(stl) || isempty(edl)
            stl=inf;
            edl=inf;
        end
    end
    if i>str && edr<edl
        XKl(i:end) = XAl(i) - Ls*sin(tiltL(i));
        YKl(i:end) = YAl(i) + Ls*cos(tiltL(i));
%         tiltTl(i) = tiltL(i);
        XH(i:end) = XKl(i) - Lt*sin(tiltTl(i));
        YH(i:end) = YKl(i) + Lt*cos(tiltTl(i));
        XC(i:end) = XH(i) - Lb*sin(tiltC(i));
        YC(i:end) = YH(i) + Lb*cos(tiltC(i));
        XKr(i:end) = XH(i) + Lt*sin(tiltTr(i));
        YKr(i:end) = YH(i) - Lt*cos(tiltTr(i));
        %cumultive right ankle
        XAr(i:end) = XKr(i) + Ls*sin(tiltR(i));
        YAr(i) = YKr(i) - Ls*cos(tiltR(i));
        %step by step right ankle
        XAR(i) = XAr(i) - XAr(str+1);
        XARIR(i) = XArIR(i) - XArIR(str);
    elseif i>stl && edl<edr
        XKr(i:end) = XAr(i) - Ls*sin(tiltR(i));
        YKr(i:end) = YAr(i) + Ls*cos(tiltR(i));
        XH(i:end) = XKr(i) - Lt*sin(tiltTr(i));
        YH(i:end) = YKr(i) + Lt*cos(tiltTr(i));
        XC(i:end) = XH(i) - Lb*sin(tiltC(i));
        YC(i:end) = YH(i) + Lb*cos(tiltC(i));
%         tiltTl(i) = polyval(f,i+ss-stl);
        XKl(i:end) = XH(i) + Lt*sin(tiltTl(i));
        YKl(i:end) = YH(i) - Lt*cos(tiltTl(i));
        %cumultive left ankle
        XAl(i:end) = XKl(i) + Ls*sin(tiltL(i));
        YAl(i) = YKl(i) - Ls*cos(tiltL(i));
        %step by step left ankle
        XAL(i) = XAl(i) - XAl(stl+1);
        XALIR(i) = XAlIR(i) - XAlIR(stl);
    else
        XKr(i:end) = XAr(i) - Ls*sin(tiltR(i));
        YKr(i:end) = YAr(i) + Ls*cos(tiltR(i));
        XH(i:end) = XKr(i) - Lt*sin(tiltTr(i));
        YH(i:end) = YKr(i) + Lt*cos(tiltTr(i));
        XC(i:end) = XH(i) - Lb*sin(tiltC(i));
        YC(i:end) = YH(i) + Lb*cos(tiltC(i));
        XKl(i:end) = XAl(i) - Ls*sin(tiltTl(i));
        YKl(i:end) = YAl(i) + Ls*cos(tiltTl(i));
    end
end
%% Comparison of measurements

% cal step length error
ERROR_R1 = zeros(1,1);
ERROR_R2 = zeros(1,1);
ERROR_L2 = zeros(1,1);
ERROR_L1 = zeros(1,1);
for n = 1:1:length(tR)
    if tR(n) > 0
        IRmax = abs(max(XARIR(floor(tR(n)-20):floor(tR(n))+20)));
        pRmax = abs(max(pR(1,floor(tR(n)-20):floor(tR(n))+20)));
        XARmax = abs(max(XAR(floor(tR(n)-20):floor(tR(n))+20)));
    else
        IRmax = abs(min(XARIR(floor(-tR(n)-20):floor(-tR(n))+20)));
        pRmax = abs(min(pR(1,floor(-tR(n)-20):floor(-tR(n))+20)));
        XARmax = abs(min(XAR(floor(-tR(n)-20):floor(-tR(n))+20)));
    end
    if IRmax~=0
        ERROR_R1 = ERROR_R1 + abs( (pRmax - IRmax) ./ IRmax );
        ERROR_R2 = ERROR_R2 + abs( (XARmax - IRmax) ./ IRmax );
    end
end
ERROR_R1 = ERROR_R1 / length(tR);
ERROR_R2 = ERROR_R2 / length(tR);
for n = 1:1:length(tL)
    if tL(n) > 0
        IRmax = abs(max(XALIR(floor(tL(n)-20):floor(tL(n))+20)));
        pLmax = abs(max(pL(1,floor(tL(n)-20):floor(tL(n))+20)));
        XALmax = abs(max(XAL(floor(tL(n)-20):floor(tL(n))+20)));
    else
        IRmax = abs(min(XALIR(floor(-tL(n)-20):floor(-tL(n))+20)));
        pLmax = abs(min(pL(1,floor(-tL(n)-20):floor(-tL(n))+20)));
        XALmax = abs(min(XAL(floor(-tL(n)-20):floor(-tL(n))+20)));
    end
    if IRmax~=0
        ERROR_L1 = ERROR_L1 + abs( (pLmax - IRmax) ./ IRmax );
        ERROR_L2 = ERROR_L2 + abs( (XALmax - IRmax) ./ IRmax );
    end
end
ERROR_L1 = ERROR_L1 / length(tL);
ERROR_L2 = ERROR_L2 / length(tL);
disp('******* Posture Using Nonlinear Observer Error Rate:');
disp(['Right Error Rate:',num2str(ERROR_R2*100)]);
disp(['Left Error Rate: ',num2str(ERROR_L2*100)]);
disp(['Mean Error Rate: ',num2str(mean([ERROR_R2,ERROR_L2])*100)]);
disp('*** Integration Using  Nonlinear Observer Error Rate:');
disp(['Right Error Rate:',num2str(ERROR_R1*100)]);
disp(['Left Error Rate: ',num2str(ERROR_L1*100)]);
disp(['Mean Error Rate: ',num2str(mean([ERROR_R1,ERROR_L1])*100)]);

%% Comparison of measurements for first half of the test (walking)
 
% cal step length error
ERROR_R1_1 = zeros(1,1);
ERROR_R2_1 = zeros(1,1);
ERROR_L2_1 = zeros(1,1);
ERROR_L1_1 = zeros(1,1);
for n = 1:1:12
    if tR(n) > 0
        IRmax = abs(max(XARIR(floor(tR(n)-20):floor(tR(n))+20)));
        pRmax = abs(max(pR(1,floor(tR(n)-20):floor(tR(n))+20)));
        XARmax = abs(max(XAR(floor(tR(n)-20):floor(tR(n))+20)));
    else
        IRmax = abs(min(XARIR(floor(-tR(n)-20):floor(-tR(n))+20)));
        pRmax = abs(min(pR(1,floor(-tR(n)-20):floor(-tR(n))+20)));
        XARmax = abs(min(XAR(floor(-tR(n)-20):floor(-tR(n))+20)));
    end
    if IRmax~=0
        ERROR_R1_1 = ERROR_R1_1 + abs( (pRmax - IRmax) ./ IRmax );
        ERROR_R2_1 = ERROR_R2_1 + abs( (XARmax - IRmax) ./ IRmax );
    end
end
ERROR_R1_1 = ERROR_R1_1 / 12;
ERROR_R2_1 = ERROR_R2_1 / 12;
ERROR_L1_all = [];
ERROR_L2_all = [];
for n = floor(length(tL)/2):1:length(tL)
    if tL(n) > 0
        IRmax = abs(max(XALIR(floor(tL(n)-20):floor(tL(n))+20)));
        pLmax = abs(max(pL(1,floor(tL(n)-20):floor(tL(n))+20)));
        XALmax = abs(max(XAL(floor(tL(n)-20):floor(tL(n))+20)));
    else
        IRmax = abs(min(XALIR(floor(-tL(n)-20):floor(-tL(n))+20)));
        pLmax = abs(min(pL(1,floor(-tL(n)-20):floor(-tL(n))+20)));
        XALmax = abs(min(XAL(floor(-tL(n)-20):floor(-tL(n))+20)));
    end
    if IRmax~=0
        ERROR_L1_1 = ERROR_L1_1 + abs( (pLmax - IRmax) ./ IRmax );
        ERROR_L2_1 = ERROR_L2_1 + abs( (XALmax - IRmax) ./ IRmax );
        ERROR_L1_all = [ERROR_L1_all;abs( (pLmax - IRmax) ./ IRmax )];
        ERROR_L2_all = [ERROR_L2_all;abs( (XALmax - IRmax) ./ IRmax )];
    end
end
ERROR_L1_1 = ERROR_L1_1 / 12;
ERROR_L2_1 = ERROR_L2_1 / 12;
disp('******* First half (walking) Error Rate:');
disp('*** Posture Using Nonlinear Observer Error Rate:');
disp(['Right Error Rate:',num2str(ERROR_R2_1*100)]);
disp(['Left Error Rate: ',num2str(ERROR_L2_1*100)]);
disp(['Mean Error Rate: ',num2str(mean([ERROR_R2_1,ERROR_L2_1])*100)]);
disp('*** Integration Using  Nonlinear Observer Error Rate:');
disp(['Right Error Rate:',num2str(ERROR_R1_1*100)]);
disp(['Left Error Rate: ',num2str(ERROR_L1_1*100)]);
disp(['Mean Error Rate: ',num2str(mean([ERROR_R1_1,ERROR_L1_1])*100)]);

ERROR_R2_2 = (ERROR_R2*22-ERROR_R2_1*12)/10;
ERROR_L2_2 = (ERROR_L2*22-ERROR_L2_1*12)/10;
ERROR_R1_2 = (ERROR_R1*22-ERROR_R1_1*12)/10;
ERROR_L1_2 = (ERROR_L1*22-ERROR_L1_1*12)/10;
disp('******* Second half (stumbling) Error Rate:');
disp('*** Posture Using Nonlinear Observer Error Rate:');
disp(['Right Error Rate:',num2str(ERROR_R2_2*100)]);
disp(['Left Error Rate: ',num2str(ERROR_L2_2*100)]);
disp(['Mean Error Rate: ',num2str(mean([ERROR_R2_2,ERROR_L2_2])*100)]);
disp('*** Integration Using  Nonlinear Observer Error Rate:');
disp(['Right Error Rate:',num2str(ERROR_R1_2*100)]);
disp(['Left Error Rate: ',num2str(ERROR_L1_2*100)]);
disp(['Mean Error Rate: ',num2str(mean([ERROR_R1_2,ERROR_L1_2])*100)]);

%% Detect climbing events and Step lengths
%[Rsteps, Lsteps, events] = findClimbing(aR, aL, events, t);
%fallSteps();
%% Total Distance and Steps
% disp(['Number of steps: ',num2str(length(Rsteps(1,:))+length(Lsteps(1,:)))]);
% disp(['Estimated Distance Traveled: ',num2str(int16(mean([sum(Rsteps(2,:)),sum(Lsteps(2,:))])))]);

%% plot results
figure(200)
plot(t,aR(1,:),t,x_hat_R(2,:),'LineWidth',2)
title('Right foot horizontal acceleration bias');
legend('Horizontal acceleration','Estimated Bias');
xlabel('t (s)');
ylabel('acceleration (m/s^2)');

figure(4)
subplot(1,2,2);
plot(t,pR(1,:)*100,t,XAR*100,t,XARIR*100,'LineWidth',2);
title('Right foot horizontal displacement');
legend('Integrated','Posture estimation','IR measured');
xlabel('t (s)');
ylabel('x (cm)');
% 
% subplot(1,2,1);
% plot(t,pR(2,:)*100,t,YAr*100,t,YArIR*100,'LineWidth',2);
% title('Right foot vertical displacement');
% xlabel('t (s)');
% ylabel('y (cm)');

subplot(1,2,1);
plot(t,pL(1,:)*100,t,XAL*100,t,XALIR*100,'LineWidth',2);
title('Left foot horizontal displacement');
legend('Integrated','Posture estimation','IR measured');
xlabel('t (s)');
ylabel('x (cm)');
% 
% subplot(2,2,3);
% plot(t,pL(2,:)*100,t,YAl*100,t,YAlIR*100,'LineWidth',2);
% title('Left foot vertical displacement');
% xlabel('t (s)');
% ylabel('y (cm)');

figure(3)
subplot(3,2,1);
plot(t,aR(1,:));
title('Right foot horizontal acceleration');
xlabel('t (s)');
ylabel('a_x (m/s^2)');

subplot(3,2,2);
plot(t,aR(2,:));
title('Right foot vertical acceleration');
xlabel('t (s)');
ylabel('a_y (m/s^2)');

subplot(3,2,3);
plot(t,aL(1,:));
title('Left foot horizontal acceleration');
xlabel('t (s)');
ylabel('a_x (m/s^2)');

subplot(3,2,4);
plot(t,aL(2,:));
title('Left foot vertical acceleration');
xlabel('t (s)');
ylabel('a_y (m/s^2)');

subplot(3,2,5);
plot(t,aC(:,1));
title('Chest horizontal acceleration');
xlabel('t (s)');
ylabel('a_x (m/s^2)');

subplot(3,2,6);
plot(t,aC(:,2));
title('Chest vertical acceleration');
xlabel('t (s)');
ylabel('a_y (m/s^2)');



% figure(10)
% subplot(1,2,1);
% plot(t,tiltR*180/pi,t,tiltTr*180/pi,'LineWidth',2);
% title('Right leg tilt angle(deg)');
% legend('Shank','Thigh');
% xlabel('t (s)');
% ylabel('\theta (deg)');
% 
% subplot(1,2,2);
% plot(t,tiltL*180/pi,t,tiltTl*180/pi,'LineWidth',2);
% title('Left leg tilt angle(deg)');
% legend('Shank','Thigh');
% xlabel('t (s)');
% ylabel('\theta (deg)');
% 
% figure(21)
% subplot(1,2,2)
% plot(t,tiltIRr,t,tiltR*180/pi,'LineWidth',2);
% title('Right shank tilt angle(deg)');
% legend('IR','IMU');
% xlabel('t (s)');
% ylabel('\theta (deg)');
% subplot(1,2,1)
% plot(t,tiltIRl,t,tiltL*180/pi,'LineWidth',2);
% title('Left shank tilt angle(deg)');
% legend('IR','IMU');
% xlabel('t (s)');
% ylabel('\theta (deg)');
% 
% figure(22)
% plot(t,tiltR*180/pi,t,tiltL*180/pi,'LineWidth',2);
% title('Right leg tilt angle(deg)');
% legend('Right Shank','Left Shank');
% xlabel('t (s)');
% ylabel('\theta (deg)');
% subplot(1,4,3);
% plot(t,tiltTr*180/pi,'LineWidth',2);
% title('Right Thigh tilt angle(deg)');
% legend('estimated value');
% xlabel('t (s)');
% ylabel('\theta (deg)');
% 
% subplot(1,4,4);
% plot(t,tiltTl*180/pi,'LineWidth',2);
% title('Left Thigh tilt angle(deg)');
% legend('estimated value');
% xlabel('t (s)');
% ylabel('\theta (deg)');
% figure(40)
% plot(t,pR(1,:)*100,t,XAR*100,t,XARIR*100,'LineWidth',2);
% title('Right foot horizontal displacement');
% legend('Integrate mode','Angle mode','IR camera');
% xlabel('time (s)');
% ylabel('x (cm)');

figure(20)
plot(t,events(:,1),'LineWidth',1.5)
title('Events');
xlabel('time (s)');
dim = [0.78 0.675 0.20 0.25];
strng = {'0 : Standing','1 : Walking','2 : Sitting','3 : Turning','4 : Lying down','5 : Bending','6 : Climbing/stairs','7 : Near fall FWD','8 : Near fall LAT','9 : Near fall BWD','10 : Free Fall' };
annotation('textbox',dim,'String',strng,'FitBoxToText','on');



% figure(41)
% plot(t,(pR(1,:)'-XARIR)*15,t,(XAR-XARIR)*15,'LineWidth',2);
% title('Right foot horizontal displacement error');
% legend('Integrate mode','Angle mode');
% xlabel('t (s)');
% ylabel('Ex (cm)');


% figure(42)
% plot(t,x_hat_R(2:3,:),'LineWidth',2);
% title('Right foot  acceleration bias');
% legend('Horizontal','Vertical');
% xlabel('t (s)');
% ylabel('Accelerometer bias (m/s^2)');

figure(42)
plot(t,biasRIR(:,1),t,x_hat_R(2,:),'LineWidth',2);
title('Right foot horizontal acceleration bias');
legend('IR camera','nonlinear observer');
xlabel('t (s)');
ylabel('Accelerometer bias (m/s^2)');
 
figure(43)
plot(t,biasRIR(:,2),t,x_hat_R(3,:),'LineWidth',2);
title('Right foot vertical acceleration bias');
legend('IR camera','nonlinear observer');
xlabel('t (s)');
ylabel('Accelerometer bias (m/s^2)');
% 
% figure(44)
% plot(t,aXArIR,t,aR(1,:),'LineWidth',2);
% title('Numerically found Right foot horizontal acceleration from IR cam position');
% legend('IR cam', 'IMU acceleration');
% xlabel('t (s)');
% ylabel('Acceleration (m/s^2)');

% figure(44)
% plot(t,tiltR*180/pi-tiltIRr','LineWidth',2);
% title('Right shank tilt angle error(deg)');
% xlabel('time (sec)');
% ylabel('E_\theta_{sr} (deg)');
