function [dataR, dataL, dataC] = dataProcess(nameR1,nameR2,nameL1,nameL2,nameC1,nameC2)

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

dataR  =  [dataR1 dataR2];
dataL  =  [dataL1 dataL2];
dataC  =  [dataC1 dataC2];

if length(dataR(:,1))<length(dataL(:,1))
    [dataR, dataL]= timeSync(dataR,dataL);
end
if length(dataR(:,1))>length(dataL(:,1))
    [dataL, dataR]= timeSync(dataL,dataR);
end
if length(dataR(:,1))>length(dataC(:,1))
    [dataC, dataR]= timeSync(dataC,dataR);
end
if length(dataR(:,1))<length(dataC(:,1))
    [dataR, dataC]= timeSync(dataR,dataC);
end
if length(dataL(:,1))>length(dataC(:,1))
    [dataC, dataL]= timeSync(dataC,dataL);
end
if length(dataL(:,1))<length(dataC(:,1))
    [dataL, dataC]= timeSync(dataL,dataC);
end


