%% Run eventClassification v2.0.0 
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

for i = 1:1:length(t)-100
   yawCRate(i) = mean(yawCRate(i:i+100));
end
yawCRate = yawCRate - yawCRate(1);
yawC = cumtrapz(t,yawCRate);

%% Finding events
eventClassification2();