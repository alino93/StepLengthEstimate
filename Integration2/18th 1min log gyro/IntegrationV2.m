%% Integration v2.0.0 
% Integrate the acceleration of each foot to find instantaneous velocity
% with turning, standing, falling detection 
% repeat with a new set of data: data0

%Using euler angles we find the orientaion of coordinates with respect to 
%the intertial axes

l=0.7;
d=0.40;

g=9.8;
num=100; %max number of steps
T=0.01; %period
theta_bias = 0; %45*pi/180; %initial mounting angle in degs 

%% REAL (MEASURED) DATA
% XR=[14 14+14+14 14+14+14+18+18 14+14+14+18*2+22*2];
% XL=[14+14 14+14+14+18 14+14+14+18*2+22 14+14*2+18*2+22*2];
% XR = [-18 -18*2];
% XL = [-18 -18*2];
% XR=XR*2.5;
% XL=XL*2.5;
% t=ones(1,length(tR));
% XR=XR'.*t;
% XL=XL'.*t;

%% Load and sync IMU data
nameR1='7_2020-02-05T20.29.05.425_C361585F0624_Accelerometer.csv';
nameL1='MetaWear_2020-02-05T20.29.05.425_F951B6EDAAE8_Accelerometer.csv';
nameC1='10_2020-02-05T20.29.05.425_DAB7F828D40E_Accelerometer.csv';
nameR2='7_2020-02-05T20.29.05.425_C361585F0624_Gyroscope.csv';
nameL2='MetaWear_2020-02-05T20.29.05.425_F951B6EDAAE8_Gyroscope.csv';
nameC2='10_2020-02-05T20.29.05.425_DAB7F828D40E_Gyroscope.xlsx';
[dataR, dataL, dataC] = dataProcess (nameR1,nameR2,nameL1,nameL2,nameC1,nameC2);

%remove initial drift (bias)
dataR  =  dataR - dataR(1,:);
dataL  =  dataL - dataL(1,:);
%dataC  =  dataC - dataC(1,:); %chest acc used for trunk direction detection

%time
t = (0:T:0.01*(length(dataR(:,1))-1))';

%accelerations
aRight = dataR(:,4:5)*g;
aLeft = dataL(:,4:5)*g;
aChest = dataC(:,4:6)*g;

%low pass filter 4Hz and correct scale and direction
% aRight  = lowpass(aRight, 4, 100);
% aLeft  = lowpass(aLeft, 4, 100);
% aChest  = lowpass(aChest, 4, 100);

aRight(:,1) = aRight(:,1)*-1;
aLeft(:,1) = aLeft(:,1)*-1;
%aChest(:) = aChest(:)*-1;
%aRight(:,2) = aRight(:,2)*-1;
aLeft(:,2) = aLeft(:,2)*-1;

%gyros
psiR = dataR(:,12);
psiL = dataL(:,12);
pitchCRate = dataC(:,11);
yawCRate =  dataC(:,10);

% psiR = psiR*pi/180+theta_bias;
% psiL = psiL*pi/180+theta_bias;
% pitchC = pitchC*pi/180;
% yawC =  yawC*pi/180;
% aRt = aRight(:,1)*-g;
% aRn = aRight(:,2)*g;
% 
% aLt = aLeft(:,1)*-g;
% aLn = aLeft(:,2)*g;

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
% figure
% plot(0:0.01:0.01*(length(aRight(:,1))-2),diff(aRight(:,1))./diff(tR));
% figure
% plot(0:0.01:0.01*(length(aLeft(:,1))-2),diff(aLeft(:,1))./diff(tL));

driveR=diff(aRight(:,1))./diff(tR);
driveL=diff(aLeft(:,1))./diff(tL);
nR_h=1;

tsw=zeros(1,500);
    tsw(1:499)=tswi(2:500);
    tsw(500)=v;
    s=sqrt(var(tsw));
    rg=rgi;
    if v>25
        rg=[1;1;1];
    elseif v<(25-2*s)
        rg(3)=0;
    end


heel=zeros(2,num);
heel(:,1)=[1;1];
i=0;
while (i<length(tR)-1)
    i=i+1;
    if (abs(driveR(i))>=50)
        nR_h=nR_h+1;
        heel(1,nR_h)=i;
        i=i+30;
    end
end
nL_h=1;
i=0;
while (i<length(tL)-1)
    i=i+1;
    if (abs(driveL(i))>=80)
        nL_h=nL_h+1;
        heel(2,nL_h)=i-10;
        i=i+30;
    end
end


%% 
%image acc to the gound coordinates
%Right Foot
aR=zeros(2,length(tR));
for i=1:1:length(tR)
    transMx = [cos(psiR(i)) -sin(psiR(i)); sin(psiR(i)) cos(psiR(i))]; %transfer matrix
    aR(:,i) = transMx*[aRt(i); aRn(i)];
    if abs(aR(1,i)) < 0.2 
        aR(1,i)=0;
    end
    if abs(aR(2,i)) < 0.2 
        aR(2,i)=0;
    end
end

%Left Foot
aL=zeros(2,length(tL));
for i=1:1:length(tL)
    transMx = [cos(psiL(i)) -sin(psiL(i)); sin(psiL(i)) cos(psiL(i))]; %transfer matrix
    aL(:,i) = transMx*[aLt(i); aLn(i)];
    if abs(aL(1,i)) < 0.2
        
        aL(1,i)=0;
    end
    if abs(aL(2,i)) < 0.2
        aL(2,i)=0;
    end
end

vR = zeros(2,length(tR));
vL = zeros(2,length(tL));
%integrate acc for each step
for i=2:1:min(nR_h,nL_h)
    if (heel(1,i)<heel(2,i))
        vR (:,heel(2,i-1):heel(1,i)) = cumtrapz(tR(heel(2,i-1):heel(1,i)),aR(:,heel(2,i-1):heel(1,i)),2);
        aR (:,heel(1,i):heel(2,i)) = zeros(2,heel(2,i)-heel(1,i)+1);
        aL (:,heel(2,i-1):heel(1,i)) = zeros(2,heel(1,i)-heel(2,i-1)+1);
        vL (:,heel(1,i):heel(2,i)) = cumtrapz(tL(heel(1,i):heel(2,i)),aL(:,heel(1,i):heel(2,i)),2);
        if (i<nR_h) && (i==min(nR_h,nL_h))
            vR (:,heel(2,i):heel(1,i+1)) = cumtrapz(tR(heel(2,i):heel(1,i+1)),aR(:,heel(2,i):heel(1,i+1)),2);
            aR (:,heel(1,i+1):length(tR)) = zeros(2,length(tR)-heel(1,i+1)+1);
        end
    else
        vL (:,heel(1,i-1):heel(2,i)) = cumtrapz(tL(heel(1,i-1):heel(2,i)),aL(:,heel(1,i-1):heel(2,i)),2);
        aL (:,heel(2,i):heel(1,i)) = zeros(2,heel(1,i)-heel(2,i)+1);
        aR (:,heel(1,i-1):heel(2,i)) = zeros(2,heel(2,i)-heel(1,i-1)+1);
        vR (:,heel(2,i):heel(1,i)) = cumtrapz(tR(heel(2,i):heel(1,i)),aR(:,heel(2,i):heel(1,i)),2);
        if (i<nL_h) && (i==min(nR_h,nL_h))
            vL (:,heel(1,i):heel(2,i+1)) = cumtrapz(tL(heel(1,i):heel(2,i+1)),aL(:,heel(1,i):heel(2,i+1)),2);
            aL (:,heel(2,i+1):length(tL)) = zeros(2,length(tL)-heel(2,i+1)+1);
        end
    end
end


%set final velocity to zero
% vR(:,end) = 0;
% vL(:,end) = 0;
%compensate vel drift after each step
% for i=1:1:min(n_h,m_h)
%     if (heelR(i)<heelL(i))
%         vR (:,heelR(i):heelL(i)) = zeros(2,heelL(i)-heelR(i)+1);
%         if i<n_h
%             vL (:,heelL(i):heelR(i+1)) = zeros(2,heelR(i+1)-heelL(i)+1);
%         end
%     else
%         vL (:,heelL(i):heelR(i)) = zeros(2,heelR(i)-heelL(i)+1);
%         if i<m_h
%             vL (:,heelR(i):heelL(i+1)) = zeros(2,heelL(i+1)-heelR(i)+1);
%         end
%     end
% end
 vR = vR*1.09;
 vL = vL*1.45;
%integration find position
pR = cumtrapz(tR,vR,2);
pL = cumtrapz(tL,vL,2);

%% plot results

figure(1)
subplot(1,2,2);
plot(pR(1,:)*100,pR(2,:)*100,'LineWidth',2);
title('Right foot position');
xlabel('x (cm)');
ylabel('y (cm)');

subplot(1,2,1);
plot(pL(1,:)*100,pL(2,:)*100,'LineWidth',2);
title('Left foot position');
xlabel('x (cm)');
ylabel('y (cm)');

figure(2)
subplot(2,1,1);
plot(tR,vR(1,:),'LineWidth',2);
title('Right foot velocity');
xlabel('t (s)');
ylabel('V_x (m/s)');

subplot(2,1,2);
plot(tR,vR(2,:),'LineWidth',2);
xlabel('t (s)');
ylabel('V_y (m/s)');

figure(3)
subplot(2,1,1);
plot(tL,vL(1,:),'LineWidth',2);
title('Left foot velocity');
xlabel('t (s)');
ylabel('V_x (m/s)');

subplot(2,1,2);
plot(tL,vL(2,:),'LineWidth',2);
xlabel('t (s)');
ylabel('V_y (m/s)');

figure(4)
subplot(1,2,2);
plot(tR,pR(1,:)*100,tR,XR,'r--','LineWidth',2);
title('Right foot horizontal displacement');
xlabel('t (s)');
ylabel('x (cm)');

subplot(1,2,1);
plot(tL,pL(1,:)*100,tR,XL,'r--','LineWidth',2);
title('Left foot horizontal displacement');
legend('estimated value','measured value');
xlabel('t (s)');
ylabel('x (cm)');