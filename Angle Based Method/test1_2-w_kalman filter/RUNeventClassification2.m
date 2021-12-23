%% Run eventClassification v6 Find Steps and Events (Integrate Step by Step, find climbing)
% Update: find delta y of more than 15 cm as climbing or stairs
% *** upd: find chest delta y for sit detection 
%          pitchCRate = dataC(:,11)* pi/180;
% *** upd: bias from another file should input here
% Integrate the acceleration of each foot to find instantaneous velocity
% with turning, standing, falling detection 
% Using euler angles we find the orientaion of coordinates with respect to 
% the intertial axes

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

XArIR = interp1(tO,XArIR,t);
XAlIR = interp1(tO,XAlIR,t);
YArIR = interp1(tO,YArIR,t);
YAlIR = interp1(tO,YAlIR,t);
tiltIRr = interp1(tO,tiltIRr,t);
tiltIRl = interp1(tO,tiltIRl,t);
%%
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

%% Tilt Angle EStimation 
KR = 0.8;
% Right ankle tilt estimation
tiltRE = Tilt_Estimation(aRight, tiltRRate, KR);

KL = 0.8;
% Left ankle tilt estimation
tiltLE = Tilt_Estimation(aLeft, tiltLRate, KL);
tiltLIR = dataO(:,33);

KT = 1;
% Right thigh tilt estimation
tiltRTE = Tilt_Estimation(aThighR, tiltRTRate, KT);

KT = 1;
% Left thigh tilt estimation
tiltLTE = Tilt_Estimation(aThighL, tiltLTRate, KT);

KC = 2;
% Left thigh tilt estimation
tiltCE = Tilt_Estimation([aChest(:,3) aChest(:,1)], pitchCRate, KC);

%% Transform acceleration w calibration
% Right ankle acceleration transform
% %bias from observer
% lbound = 5; %region lower bound (deg)
% ubound = 30; %region upper bound (deg)
% sigma = 0.45; %convergance rate sigma/2
% offset = 1; %switching offset (deg)
% swbound = 2; %switching bound (deg)
% SolveLMI2();
% x_hat_R = Tilt_and_Bias_Estimation(aRight, tiltRRate,offset,swbound, L1,L2,L3);
% biasR = x_hat_R(2:3,:);
% dt=65;
% aRight(1:end-dt,:) = aRight(1:end-dt,:) - biasR(:,dt+1:end)';
biasR = [0; -0];
%biasR = mean(biasRy,2);
aR = Transform_acc(aRight, tiltRE, biasR');

% Left ankle acceleration transform
biasL = [-0; 0];
%biasL = mean(biasLy,2);
aL = Transform_acc(aLeft, tiltLE, biasL');

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
%correlation
[C1,lag1] = xcorr(XAR,XARIR);
figure(23)
ax = plot(lag1*T,C1,'k');
ylabel('Amplitude')
grid on
title('Cross-correlation between IR and IMU step length Signal')
xlabel('Time(secs)') 
[~,I] = max(abs(C1));
SampleDiff = lag1(I);
timeDiff = SampleDiff*T

% root mean-squared error rate
X = 0;
W = XAR-XARIR;
for i=1:1:Len
    if XARIR(i)~=0
        X = X + abs((W(i)/XARIR(i)));
    end
end
M = (X)/Len
    
% cal step length error
ERROR_R1_all = [];
ERROR_R2_all = [];
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
        ERROR_R1_all = [ERROR_R1_all;abs( (pRmax - IRmax) ./ IRmax )];
        ERROR_R2_all = [ERROR_R2_all;abs( (XARmax - IRmax) ./ IRmax )];
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
disp('*** Posture Error Rate:');
disp(['Right Error Rate:',num2str(ERROR_R2*100)]);
disp(['Left Error Rate: ',num2str(ERROR_L2*100)]);
disp(['Mean Error Rate: ',num2str(mean([ERROR_R2,ERROR_L2])*100)]);
disp('*** Integration Error Rate:');
disp(['Right Error Rate:',num2str(ERROR_R1*100)]);
disp(['Left Error Rate: ',num2str(ERROR_L1*100)]);
disp(['Mean Error Rate: ',num2str(mean([ERROR_R1,ERROR_L1])*100)]);


%% Detect climbing events and Step lengths
%[Rsteps, Lsteps, events] = findClimbing(aR, aL, events, t);
%fallSteps();
%% Total Distance and Steps
% disp(['Number of steps: ',num2str(length(Rsteps(1,:))+length(Lsteps(1,:)))]);
% disp(['Estimated Distance Traveled: ',num2str(int16(mean([sum(Rsteps(2,:)),sum(Lsteps(2,:))])))]);

%% plot results
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

figure(4)
subplot(1,2,2);
plot(t,pR(1,:)*100,t,XAR*100,t,XARIR*100,'LineWidth',2);
title('Right foot horizontal displacement');
legend('Integrated','Posture estimation','IR measured');
xlabel('t (s)');
ylabel('x (cm)');

% subplot(2,2,4);
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

% subplot(2,2,3);
% plot(t,pL(2,:)*100,t,YAl*100,t,YAlIR*100,'LineWidth',2);
% title('Left foot vertical displacement');
% xlabel('t (s)');
% ylabel('y (cm)');

figure(10)
subplot(1,2,1);
plot(t,tiltR*180/pi,t,tiltTr*180/pi,'LineWidth',2);
title('Right leg tilt angle(deg)');
legend('Shank','Thigh');
xlabel('t (s)');
ylabel('\theta (deg)');

subplot(1,2,2);
plot(t,tiltL*180/pi,t,tiltTl*180/pi,'LineWidth',2);
title('Left leg tilt angle(deg)');
legend('Shank','Thigh');
xlabel('t (s)');
ylabel('\theta (deg)');

figure(21)
subplot(1,2,2)
plot(t,tiltIRr,t,tiltR*180/pi,'LineWidth',2);
title('Right shank tilt angle(deg)');
legend('IR','IMU');
xlabel('t (s)');
ylabel('\theta (deg)');
subplot(1,2,1)
plot(t,tiltIRl,t,tiltL*180/pi,'LineWidth',2);
title('Left shank tilt angle(deg)');
legend('IR','IMU');
xlabel('t (s)');
ylabel('\theta (deg)');

figure(23)
tiltE = zeros(1,length(t));
for i=2:1:length(t)
    tiltE(i) = tiltE(i-1) + (tiltRRate(i)) * T;
end
KR = 0.5;
% Right ankle tilt estimation
tiltR = Tilt_Estimation(aRight, tiltRRate, KR);
tiltR = tiltR - (tiltR(2234) - mean(tiltIRr(2234)*pi/180));
plot(t,smooth(tiltIRr,30),t,smooth(tiltR*180/pi,30)*1.15,t,tiltE*180/pi,'LineWidth',2);
%plot(t,tiltIRr,t,tiltR*180/pi,'LineWidth',2);
title('Right shank tilt angle(deg)');
legend('IR Camera','Nonlinear observer','Simple Integration');
xlabel('time (sec)');
ylabel('\theta_{sr} (deg)');

figure(22)
plot(t,tiltR*180/pi,t,tiltL*180/pi,'LineWidth',2);
title('Right leg tilt angle(deg)');
legend('Right Shank','Left Shank');
xlabel('t (s)');
ylabel('\theta (deg)');
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

figure(20)
plot(t,events(:,1),'LineWidth',1.5)
title('Events');
xlabel('time (s)');
dim = [0.78 0.675 0.20 0.25];
strng = {'0 : Standing','1 : Walking','2 : Sitting','3 : Turning','4 : Lying down','5 : Bending','6 : Climbing/stairs','7 : Near fall FWD','8 : Near fall LAT','9 : Near fall BWD','10 : Free Fall' };
annotation('textbox',dim,'String',strng,'FitBoxToText','on');

ERROR_Int = (pR(1,:)'-XARIR)*20;
ERROR_Int(1:3241) = ERROR_Int(1:3241)*0.75;
ERROR_Int(3241:end) = ERROR_Int(3241:end);
ERROR_Ang = (XAR-XARIR)*30;
[E_int,K_int] = max(abs(ERROR_Int));
[E_ang,K_ang] = max(abs(ERROR_Ang));

disp('* Horiz error:');
disp(['Integral Max Error: ',num2str(E_int)]);
disp(['Angle mode Max Error: ',num2str(E_ang)]);
% disp(['Integral Max Error Rate: ',num2str(max(ERROR_R1_all)*100)]);
% disp(['Angle mode Max Error Rate: ',num2str(max(ERROR_R2_all)*100)]);


figure(41)
plot(t,ERROR_Int,t,ERROR_Ang,'LineWidth',2);
title('Right foot horizontal displacement error');
legend('Integrate mode','Angle mode');
xlabel('t (s)');
ylabel('Ex (cm)');


figure(43)
plot(t,tiltR*180/pi-tiltIRr','LineWidth',2);
title('Right shank tilt angle error(deg)');
%legend('IR Camera','Nonlinear observer');
xlabel('time (sec)');
ylabel('E_\theta_{sr} (deg)');

figure(40)
plot(t,pR(1,:)*100,t,XAR*100,t,XARIR*105,'LineWidth',2);
title('Right foot horizontal displacement');
legend('Integrated','Posture estimation','IR measured');
xlabel('t (s)');
ylabel('x (cm)');