%extract optitrack data
nameO='Take 2020-08-10 07.59.03 AM.csv';
dataO = xlsread(nameO);
dataO = dataO(8:end,:);
[ht,wd] = size(dataO);
Frq=119.55;
% approximate time sync
deltaT = int16(119.3*(8.23));
if deltaT>0
    dataO = dataO(deltaT+1:end,:);
else
    dataO = [zeros(deltaT,wd);dataO];
end
tO = 0:1/Frq:(length(dataO(:,1))-1)/Frq;

%tilt right shank
tiltIRr = -1*dataO(:,39);
tiltIRr = tiltIRr - mean(tiltIRr(1));

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