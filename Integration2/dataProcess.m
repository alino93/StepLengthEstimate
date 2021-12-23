function [dataR, dataL, dataC] = dataProcess(nameR1,nameR2,nameL1,nameL2,nameC1,nameC2)

dataCa = xlsread(nameC1);
CaL = length(dataCa(:,1));
dataCg = xlsread(nameC2);
CgL = length(dataCg(:,1));

dataR  =  [xlsread(nameR1) xlsread(nameR2)];
dataL  =  [xlsread(nameL1) xlsread(nameL2)];
dataC  =  [dataCa(1:min(CaL,CgL),:) dataCg(1:min(CaL,CgL),:)];

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


