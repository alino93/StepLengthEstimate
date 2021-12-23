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

nameRS1=strcat('./1turn/Right Shank_A.csv');
nameRS2=strcat('./1turn/Right Shank_G.csv');
nameLS1=strcat('./1turn/Left Shank_A.csv');
nameLS2=strcat('./1turn/Left Shank_G.csv');

[dataR, dataL] = dataProcess3 (nameRS1,nameRS2,nameLS1,nameLS2);
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

tf = min([dataR(end,3),dataL(end,3)]);

t = (0:T:tf)';
[t1, index] = unique(dataR(:,3)); 
dataR = interp1(t1,dataR(index,:),t);
[t1, index] = unique(dataL(:,3)); 
dataL = interp1(t1,dataL(index,:),t);


%%
%accelerations
aRight = dataR(:,4:5)*g;
aLeft = dataL(:,4:5)*g;

aRight(:,1) = dataR(:,4)*g;
aRight(:,2) = dataR(:,5)*g;
aLeft(:,1) = dataL(:,4)*g;
aLeft(:,2) = dataL(:,5)*g;

%gyros
tiltRRate = dataR(:,12)*  pi/180;
tiltRRate = tiltRRate - mean(tiltRRate(1:1));
tiltLRate = dataL(:,12)* pi/180;
tiltLRate = tiltLRate - mean(tiltLRate(1:1));

yawCRate =  dataR(:,11);

for i = 1:1:length(t)-100
   yawCRate(i) = mean(yawCRate(i:i+100));
end
yawCRate = yawCRate - yawCRate(1);
yawC = cumtrapz(t,yawCRate);

pitchCRate = dataR(:,11);
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
figure(22)
plot(t,tiltRE*180/pi,t,tiltLE*180/pi,'LineWidth',2);
title('Right leg tilt angle(deg)');
legend('Right Shank','Left Shank');
xlabel('t (s)');
ylabel('\theta (deg)');
%% Transform acceleration w calibration
% Right ankle acceleration transform
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
%% Measured displacements
XR=[20 23+17 17 20+23 20 23+17];
XL=[20+23 17 23+17 20 20+23 17];
XR=XR*2.54/100;
XL=XL*2.54/100;
tR=ones(1,length(XR));
tL=ones(1,length(XL));
iR=1;
iL=1;
for i=1:1:length(events(:,1))
   if events(i,2) == 12 && events(i,1) == 1
       tR(iR) = i/100;
       iR = iR + 1;
   end
   if events(i,3) == 22 && events(i,1) == 1
       tL(iL) = i/100;
       iL = iL + 1;
   end
end
%% cal posture error
ERROR_R2 = zeros(1,1);
ERROR_L2 = zeros(1,1);
for n = 1:1:length(tR)
    pRmax = max(abs(XAR(floor(tR(n)*100-40):floor(tR(n)*100)+40)));
    ERROR_R2 = ERROR_R2 + abs( (pRmax - XR(n)) ./ pRmax );
end
ERROR_R2 = ERROR_R2 / length(tR); 
for n = 1:1:length(tL)
    pLmax = max(abs(XAL(floor(tL(n)*100-40):floor(tL(n)*100)+40)));
    ERROR_L2 = ERROR_L2 + abs( (pLmax - XL(n)) ./ pLmax );
end
ERROR_L2 = ERROR_L2 / length(tL);
disp('*** Posture Error Rate:');
disp(['Right Error Rate:',num2str(ERROR_R2(1)*100)]);
disp(['Left Error Rate: ',num2str(ERROR_L2(1)*100)]);
disp(['Mean Error Rate: ',num2str(mean([ERROR_R2(1),ERROR_L2(1)])*100)]);



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
plot(t,XAR*100,tR,XR*100,'ro','LineWidth',2);
title('Right foot horizontal displacement');
legend('Posture estimation','Measured');
xlabel('t (s)');
ylabel('x (cm)');

% subplot(2,2,4);
% plot(t,pR(2,:)*100,t,YAr*100,t,YArIR*100,'LineWidth',2);
% title('Right foot vertical displacement');
% xlabel('t (s)');
% ylabel('y (cm)');

subplot(1,2,1);
plot(t,XAL*100,tL,XL*100,'ro','LineWidth',2);
title('Left foot horizontal displacement');
legend('Posture estimation','Measured');
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

