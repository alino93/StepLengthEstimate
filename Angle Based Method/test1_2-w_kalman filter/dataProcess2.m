function [dataR, dataL, dataTR, dataTL, dataC] = dataProcess2(nameR1,nameR2,nameL1,nameL2,nameTR1,nameTR2,nameTL1,nameTL2,nameC1,nameC2)

dataC1 = xlsread(nameC1);
dataC2 = xlsread(nameC2);
len = min([length(dataC1(:,1)),length(dataC2(:,1))]);
%match sizes 
dataC1(len+1:end,:) = [];
dataC2(len+1:end,:) = [];
%remove timer bias
dataC1(:,3) = dataC1(:,3) + (dataC1(:,1) - dataC2(:,1))/1000;

dataR1 = xlsread(nameR1);
dataR2 = xlsread(nameR2);
len = min([length(dataR1(:,1)),length(dataR2(:,1))]);
%match sizes 
dataR1(len+1:end,:) = [];
dataR2(len+1:end,:) = [];
%remove timer bias
dataR1(:,3) = dataR1(:,3) + (dataR1(:,1) - dataR2(:,1))/1000;

dataL1 = xlsread(nameL1);
dataL2 = xlsread(nameL2);
len = min([length(dataL1(:,1)),length(dataL2(:,1))]);
%match sizes 
dataL1(len+1:end,:) = [];
dataL2(len+1:end,:) = [];
%remove timer bias
dataL1(:,3) = dataL1(:,3) + (dataL1(:,1) - dataL2(:,1))/1000;

dataTR1 = xlsread(nameTR1);
dataTR2 = xlsread(nameTR2);
len = min([length(dataTR1(:,1)),length(dataTR2(:,1))]);
%match sizes 
dataTR1(len+1:end,:) = [];
dataTR2(len+1:end,:) = [];
%remove timer bias
dataTR1(:,3) = dataTR1(:,3) + (dataTR1(:,1) - dataTR2(:,1))/1000;

dataTL1 = xlsread(nameTL1);
dataTL2 = xlsread(nameTL2);
len = min([length(dataTL1(:,1)),length(dataTL2(:,1))]);
%match sizes 
dataTL1(len+1:end,:) = [];
dataTL2(len+1:end,:) = [];
%remove timer bias
dataTL1(:,3) = dataTL1(:,3) + (dataTL1(:,1) - dataTL2(:,1))/1000;

dataR  =  [dataR1 dataR2];
dataL  =  [dataL1 dataL2];
dataTR  =  [dataTR1 dataTR2];
dataTL  =  [dataTL1 dataTL2];
dataC  =  [dataC1 dataC2];

%sync timers relative to each other using epochs
len = min([length(dataR(:,1)),length(dataTR(:,1)),length(dataL(:,1)),length(dataTL(:,1)),length(dataC(:,1))]);
%match sizes 
dataL(len+1:end,:) = [];
dataR(len+1:end,:) = [];
dataTL(len+1:end,:) = [];
dataTR(len+1:end,:) = [];
dataC(len+1:end,:) = [];
%remove timer bias
dataL(:,3) = dataL(:,3) + (dataL(:,1) - dataR(:,1))/1000;
dataC(:,3) = dataC(:,3) + (dataC(:,1) - dataR(:,1))/1000;
dataTL(:,3) = dataTL(:,3) + (dataTL(:,1) - dataR(:,1))/1000;
dataTR(:,3) = dataTR(:,3) + (dataTR(:,1) - dataR(:,1))/1000;
%sync timers
% [dataR, dataL]= timeSync2(dataR,dataL);
% [dataL, dataR]= timeSync2(dataL,dataR);
% [dataTR, dataTL]= timeSync2(dataTR,dataTL);
% [dataTL, dataTR]= timeSync2(dataTL,dataTR);
% [dataR, dataTR]= timeSync2(dataR,dataTR);
% [dataTR, dataR]= timeSync2(dataTR,dataR);
% [dataR, dataTL]= timeSync2(dataR,dataTL);
% [dataTL, dataR]= timeSync2(dataTL,dataR);
% [dataC, dataR]= timeSync2(dataC,dataR);
% [dataR, dataC]= timeSync2(dataR,dataC);
% [dataTR, dataC]= timeSync2(dataTR,dataC);
% [dataC, dataTR]= timeSync2(dataC,dataTR);
% [dataC, dataTL]= timeSync2(dataC,dataTL);
% [dataTL, dataC]= timeSync2(dataTL,dataC);
% [dataC, dataL]= timeSync2(dataC,dataL);
% [dataL, dataC]= timeSync2(dataL,dataC);
% [dataTL, dataL]= timeSync2(dataTL,dataL);
% [dataL, dataTR]= timeSync2(dataL,dataTR);
% [dataL, dataTL]= timeSync2(dataL,dataTL);
%[dataTL, dataL]= timeSync2(dataTL,dataL);
% [dataR, dataL]= timeSync2(dataR,dataL);
% [dataL, dataR]= timeSync2(dataL,dataR);
% [dataL, dataR, dataTR]= timeSync3(dataL,dataR,dataTR);
% [dataR, dataL, dataTL]= timeSync3(dataR,dataL,dataTL);

len = min([length(dataR(:,1)),length(dataTR(:,1)),length(dataL(:,1)),length(dataTL(:,1)),length(dataC(:,1))]);
dataR = dataR(1:len,:);
dataL = dataL(1:len,:);
dataC = dataC(1:len,:);
dataTL = dataTL(1:len,:);
dataTR = dataTR(1:len,:);



