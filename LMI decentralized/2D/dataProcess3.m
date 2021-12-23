function [dataR, dataL] = dataProcess3(nameR1,nameR2,nameL1,nameL2)


dataR1 = xlsread(nameR1);
dataR2 = xlsread(nameR2);

if length(dataR1(:,1))<length(dataR2(:,1))
    [dataR1, dataR2]= timeSync(dataR1,dataR2);
end
if length(dataR2(:,1))<length(dataR1(:,1))
    [dataR2, dataR1]= timeSync(dataR2,dataR1);
end

dataL1 = xlsread(nameL1);
dataL2 = xlsread(nameL2);

if length(dataL1(:,1))<length(dataL2(:,1))
    [dataL1, dataL2]= timeSync(dataL1,dataL2);
end
if length(dataL2(:,1))<length(dataL1(:,1))
    [dataL2, dataL1]= timeSync(dataL2,dataL1);
end


dataR  =  [dataR1 dataR2];
dataL  =  [dataL1 dataL2];

[dataR, dataL]= timeSync(dataR,dataL);
[dataL, dataR]= timeSync(dataL,dataR);

% [dataR, dataL]= timeSync(dataR,dataL);
% [dataL, dataR]= timeSync(dataL,dataR);
% [dataC, dataR]= timeSync(dataC,dataR);
% [dataR, dataC]= timeSync(dataR,dataC);
% [dataC, dataL]= timeSync(dataC,dataL);
% [dataL, dataC]= timeSync(dataL,dataC);
% [dataR, dataL]= timeSync(dataR,dataL);
% [dataL, dataR]= timeSync(dataL,dataR);

len = min([length(dataR(:,1)),length(dataL(:,1))]);
dataR = dataR(1:len,:);
dataL = dataL(1:len,:);




