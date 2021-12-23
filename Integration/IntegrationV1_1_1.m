%% Integration v1.1.1 
% Integrate the acceleration of each foot 
% with drift compensation
% without transformation for zero biased coordinates


l=0.7;
d=0.40;
g=9.8;
num=100; %max number of steps
T=0.01; %period

%load acc data
data  =  xlsread('data0.xlsx');
data  =  data - data(1,:);
aRight = data(:,2:3);
aLeft = data(:,4:5);

tR = (0:T:0.01*(length(aRight(:,1))-1))';
tL = (0:T:0.01*(length(aLeft(:,1))-1))';

aRt = aRight(:,1)*-g;
aRn = aRight(:,2)*g;

aLt = aLeft(:,1)*-g;
aLn = aLeft(:,2)*g;

%finding heel strikes
% figure
% plot(0:0.01:0.01*(length(aRight(:,1))-2),diff(aRight(:,1))./diff(tR));
% figure
% plot(0:0.01:0.01*(length(aLeft(:,1))-2),diff(aLeft(:,1))./diff(tL));

driveR=diff(aRight(:,1))./diff(tR);
driveL=diff(aLeft(:,1))./diff(tL);
nR_h=1;
heel=zeros(2,num);
heel(:,1)=[1;1];
i=0;
while (i<length(tR)-1)
    i=i+1;
    if (abs(driveR(i))>=100)
        nR_h=nR_h+1;
        heel(1,nR_h)=i;
        i=i+30;
    end
end
nL_h=1;
i=0;
while (i<length(tL)-1)
    i=i+1;
    if (abs(driveL(i))>=100)
        nL_h=nL_h+1;
        heel(2,nL_h)=i;
        i=i+30;
    end
end

%image acc to the ground coordinates
%Right Foot
aR=zeros(2,length(tR));
for i=1:1:length(tR)
    aR(:,i) = [aRt(i); aRn(i)];
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
    aL(:,i) = [aLt(i); aLn(i)];
    if abs(aL(1,i)) < 0.2 
        aL(1,i)=0;
    end
    if abs(aL(2,i)) < 0.2
        aL(2,i)=0;
    end
end
vR = zeros(2,length(tR));
vL = zeros(2,length(tL));
%integrate acc for each step and compensate drift
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


%integration find position
pR = cumtrapz(tR,vR,2);
pL = cumtrapz(tL,vL,2);

%% plot results
figure
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

figure
subplot(2,1,1);
plot(tR,vR(1,:),'LineWidth',2);
title('Right foot velocity');
xlabel('t (s)');
ylabel('V_x (m/s)');

subplot(2,1,2);
plot(tR,vR(2,:),'LineWidth',2);
xlabel('t (s)');
ylabel('V_y (m/s)');

figure
subplot(2,1,1);
plot(tL,vL(1,:),'LineWidth',2);
title('Left foot velocity');
xlabel('t (s)');
ylabel('V_x (m/s)');

subplot(2,1,2);
plot(tL,vL(2,:),'LineWidth',2);
xlabel('t (s)');
ylabel('V_y (m/s)');

figure
subplot(1,2,2);
plot(tR,pR(1,:)*100,6.5,55.88,'r--o',8.5,101.6,'r--o','LineWidth',2);
title('Right foot horizontal displacement');
xlabel('t (s)');
ylabel('x (cm)');

subplot(1,2,1);
plot(tL,pL(1,:)*100,7.5,101.6,'r--o','LineWidth',2);
title('Left foot horizontal displacement');
xlabel('t (s)');
ylabel('x (cm)');