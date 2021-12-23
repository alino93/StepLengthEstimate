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
date='2021-01-27T18.07.19.089';
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
nameC1= nameRS1;
nameC2= nameRS2;
[dataR, dataL, dataRT, dataLT, dataC] = dataProcess3 (nameRS1,nameRS2,nameLS1,nameLS2,nameRT1,nameRT2,nameLT1,nameLT2,nameC1,nameC2);
%[dataR, dataRT, dataC] = dataProcess (nameRS1,nameRS2,nameLS1,nameLS2,nameRT1,nameRT2,nameLT1,nameLT2,nameC1,nameC2);

%% 
% dataR(:,3) = 0:0.05:0.05*length(dataR(:,3));
% dataL(:,3) = 0:0.05:0.05*length(dataR(:,3));
% dataC(:,3) = 0:0.05:0.05*length(dataR(:,3));
% dataRT(:,3) = 0:0.05:0.05*length(dataR(:,3));
% dataLT(:,3) = 0:0.05:0.05*length(dataR(:,3));
%time
t0 = 0; %max([dataR(1,3),dataL(1,3), dataRT(1,3), dataLT(1,3), dataC(1,3)]);

dataR(:,3) = dataR(:,3) - dataR(1,3);
dataL(:,3) = dataL(:,3) - dataL(1,3);
dataRT(:,3) = dataRT(:,3) - dataRT(1,3);
dataLT(:,3) = dataLT(:,3) - dataLT(1,3);
dataC(:,3) = dataC(:,3) - dataC(1,3);

tf = min([dataR(end,3),dataL(end,3), dataRT(end,3), dataLT(end,3), dataC(end,3)]);

t = (0:T:tf)';
[t1, index] = unique(dataR(:,3)); 
dataR = interp1(t1,dataR(index,:),t);
[t1, index] = unique(dataL(:,3)); 
dataL = interp1(t1,dataL(index,:),t);
[t1, index] = unique(dataRT(:,3)); 
dataRT = interp1(t1,dataRT(index,:),t);
[t1, index] = unique(dataLT(:,3)); 
dataLT = interp1(t1,dataLT(index,:),t);
[t1, index] = unique(dataC(:,3)); 
dataC = interp1(t1,dataC(index,:),t);


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
aThighL(:,2) = -dataLT(:,5)*g;
aChest(:,3) = aChest(:,3);

%gyros
tiltRRate = dataR(:,12)*  pi/180;
tiltRRate = tiltRRate - mean(tiltRRate(1:100));
tiltLRate = dataL(:,12)* pi/180;
tiltLRate = tiltLRate - mean(tiltLRate(1:100));
tiltRTRate = dataRT(:,12)* pi/180;
tiltRTRate = tiltRTRate - mean(tiltRTRate(1:100));
tiltLTRate = dataLT(:,12)* -pi/180;
tiltLTRate = tiltLTRate - mean(tiltLTRate(1:100));
pitchCRate = dataC(:,11)*pi/180;
pitchCRate = pitchCRate - mean(pitchCRate(1:100)); 
yawCRate =  dataRT(:,10)*-1;

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

KT = 0.5;
% Right thigh tilt estimation
tiltRTE = Tilt_Estimation(aThighR, tiltRTRate, KT);

KT = 0.5;
% Left thigh tilt estimation
tiltLTE = Tilt_Estimation(aThighL, tiltLTRate, KT);

KC = 2;
% Left thigh tilt estimation
tiltCE = Tilt_Estimation([aChest(:,3) aChest(:,1)], pitchCRate, KC);

%% Transform acceleration w calibration
% Right ankle acceleration transform
biasR = [-2.2; -0];
%biasR = mean(biasRy,2);
aR = Transform_acc(aRight, tiltRE, biasR');

% Left ankle acceleration transform
biasL = [-1.5; 0];
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

Ll=1.05;
Ls=0.5;
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

%% Total Distance and Steps
R2 = zeros(1,1);
L2 = zeros(1,1);
for n = 1:1:length(tR)
    pRmax = max(abs(pR(1,tR(n)-40:tR(n))));
    R2 = R2 + abs( pRmax );
end
 
for n = 1:1:length(tL)
    pLmax = max(abs(pL(1,tL(n)-40:tL(n))));
    L2 = L2 + abs( pLmax );
end

disp(['Integrate Estimated Distance Traveled (m): ',num2str((mean([R2,L2])))]);
disp(['Angle-b Estimated Distance Traveled (m): ',num2str((mean([XAr(end),XAl(end)])))]);
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
plot(t,XAR*100,t,pR(1,:)*100,'LineWidth',2);
title('Right foot horizontal displacement (stride)');
legend('Posture estimation','Integrate');
xlabel('t (s)');
ylabel('x (cm)');

% subplot(2,2,4);
% plot(t,pR(2,:)*100,t,YAr*100,t,YArIR*100,'LineWidth',2);
% title('Right foot vertical displacement');
% xlabel('t (s)');
% ylabel('y (cm)');

subplot(1,2,1);
plot(t,XAL*100,t,pL(1,:)*100,'LineWidth',2);
title('Left foot horizontal displacement (stride)');
legend('Posture estimation','Integrate');
xlabel('t (s)');
ylabel('x (cm)');

% subplot(2,2,3);
% plot(t,pL(2,:)*100,t,YAl*100,t,YAlIR*100,'LineWidth',2);
% title('Left foot vertical displacement');
% xlabel('t (s)');
% ylabel('y (cm)');

figure(6)
plot(t,XAr,t,XAl,'LineWidth',2);
title('Foot total horizontal displacement (cumulative)');
legend('Right foot','Left foot');
xlabel('t (s)');
ylabel('x (m)');


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
plot(t,tiltR*180/pi,'LineWidth',2);
title('Right shank tilt angle(deg)');
legend('IMU');
xlabel('t (s)');
ylabel('\theta (deg)');
subplot(1,2,1)
plot(t,tiltL*180/pi,'LineWidth',2);
title('Left shank tilt angle(deg)');
legend('IMU');
xlabel('t (s)');
ylabel('\theta (deg)');

figure(23)
plot(t,tiltR*180/pi,'LineWidth',2);
title('Right shank tilt angle(deg)');
legend('Nonlinear observer');
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

