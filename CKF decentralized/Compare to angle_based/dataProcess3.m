function [dataR, dataL, dataTR, dataTL, dataC] = dataProcess3(nameR1,nameR2,nameL1,nameL2,nameTR1,nameTR2,nameTL1,nameTL2,nameC1,nameC2)

dataC1 = xlsread(nameC1);
dataC2 = xlsread(nameC2);

if length(dataC1(:,1))<length(dataC2(:,1))
    [dataC1, dataC2]= timeSync(dataC1,dataC2);
end
if length(dataC2(:,1))<length(dataC1(:,1))
    [dataC2, dataC1]= timeSync(dataC2,dataC1);
end

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

dataTR1 = xlsread(nameTR1);
dataTR2 = xlsread(nameTR2);

if length(dataTR1(:,1))<length(dataTR2(:,1))
    [dataTR1, dataTR2]= timeSync(dataTR1,dataTR2);
end
if length(dataTR2(:,1))<length(dataTR1(:,1))
    [dataTR2, dataTR1]= timeSync(dataTR2,dataTR1);
end

dataTL1 = xlsread(nameTL1);
dataTL2 = xlsread(nameTL2);

if length(dataTL1(:,1))<length(dataTL2(:,1))
    [dataTL1, dataTL2]= timeSync(dataTL1,dataTL2);
end
if length(dataTL2(:,1))<length(dataTL1(:,1))
    [dataTL2, dataTL1]= timeSync(dataTL2,dataTL1);
end

dataR  =  [dataR1 dataR2];
dataL  =  [dataL1 dataL2];
dataC  =  [dataC1 dataC2];
dataTR  =  [dataTR1 dataTR2];
dataTL  =  [dataTL1 dataTL2];
% if length(dataR(:,1))<length(dataL(:,1))
%     [dataR, dataL]= timeSync(dataR,dataL);
% end
% if length(dataR(:,1))>length(dataL(:,1))
%     [dataL, dataR]= timeSync(dataL,dataR);
% end
% if length(dataR(:,1))>length(dataC(:,1))
%     [dataC, dataR]= timeSync(dataC,dataR);
% end
% if length(dataR(:,1))<length(dataC(:,1))
%     [dataR, dataC]= timeSync(dataR,dataC);
% end
% if length(dataL(:,1))>length(dataC(:,1))
%     [dataC, dataL]= timeSync(dataC,dataL);
% end
% if length(dataL(:,1))<length(dataC(:,1))
%     [dataL, dataC]= timeSync(dataL,dataC);
% end
[dataR, dataTR]= timeSync(dataR,dataTR);
[dataTR, dataR]= timeSync(dataTR,dataR);
[dataL, dataTL]= timeSync(dataL,dataTL);
[dataTL, dataL]= timeSync(dataTL,dataL);
[dataL, dataR, dataTR]= timeSync1(dataL,dataR,dataTR);
[dataR, dataL, dataTL]= timeSync1(dataR,dataL,dataTL);
[dataL, dataR, dataTR]= timeSync1(dataL,dataR,dataTR);
% [dataR, dataL]= timeSync(dataR,dataL);
% [dataL, dataR]= timeSync(dataL,dataR);
% [dataC, dataR]= timeSync(dataC,dataR);
% [dataR, dataC]= timeSync(dataR,dataC);
% [dataC, dataL]= timeSync(dataC,dataL);
% [dataL, dataC]= timeSync(dataL,dataC);
% [dataR, dataL]= timeSync(dataR,dataL);
% [dataL, dataR]= timeSync(dataL,dataR);

len = min([length(dataR(:,1)),length(dataL(:,1)),length(dataC(:,1))]);
dataR = dataR(1:len,:);
dataL = dataL(1:len,:);
dataC = dataC(1:len,:);
dataTL = dataTL(1:len,:);
dataTR = dataTR(1:len,:);



