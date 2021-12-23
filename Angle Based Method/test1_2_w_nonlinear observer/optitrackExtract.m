%extract optitrack data
nameO='Take 2020-08-10 07.59.03 AM.csv';
dataO = xlsread(nameO);
dataO = dataO(8:end,:);
[ht,wd] = size(dataO);
% approximate time sync
deltaT = int16(120*(8.05));
if deltaT>0
    dataO = dataO(deltaT+1:end,:);
else
    dataO = [zeros(deltaT,wd);dataO];
end
tO = 0:1/120:(length(dataO(:,1))-1)/120;

%tilt right shank
tiltIRr = -1*dataO(:,39);
tiltIRr = tiltIRr - tiltIRr(1);

%displacement right shank
XArIR = dataO(:,47);
XArIR = XArIR - XArIR(1);
YArIR = dataO(:,46);
YArIR = YArIR - YArIR(1);

%tilt right shank
tiltIRl = -1*dataO(:,57);
tiltIRl = tiltIRl - tiltIRl(1);

%displacement right shank
XAlIR = dataO(:,65);
XAlIR = XAlIR - XAlIR(1);
YAlIR = dataO(:,64);
YAlIR = YAlIR - YAlIR(1);