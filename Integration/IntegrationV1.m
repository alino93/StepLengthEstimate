%% Integration
% Integrate the acceleration of each foot to find instantaneous velocity
% without drift compensation
% with transformation

%from gyro we find the orientaion of coordinates

l=0.7;
d=0.40;
g=9.8;
num=100; %max number of steps
T=0.01; %period
XR = [22 18+22]; %real x right positions in inches
XL = [18+22];       %real x left positions in inches

%load acc data
aRight  =  xlsread('aRight3.xlsx');
aLeft  =  xlsread('aLeft3.xlsx');
% aRight = aRight - aRight(1,:);
aLeft = aLeft - aLeft(1,:);

tR = (0:T:0.01*(length(aRight(:,1))-1))';
tL = (0:T:0.01*(length(aLeft(:,1))-1))';

XR=XR*2.5;
XL=XL*2.5;
t=ones(1,length(tR));
XR=XR'.*t;
XL=XL'.*t;
%finding heel strikes
% figure
% plot(0:0.01:0.01*(length(aRight(:,1))-2),diff(aRight(:,2))./diff(tR));
% figure
% plot(0:0.01:0.01*(length(aLeft(:,1))-2),diff(aLeft(:,2))./diff(tL));

driveR=diff(aRight(:,2))./diff(tR);
driveL=diff(aLeft(:,2))./diff(tL);
n_h=0;
heelR=zeros(1,num);
heelL=zeros(1,num);
i=0;
while (i<length(tR)-1)
    i=i+1;
    if (abs(driveR(i))>=30)
        n_h=n_h+1;
        heelR(n_h)=i;
        i=i+30;
    end
end
m_h=0;
i=0;
while (i<length(tL)-1)
    i=i+1;
    if (abs(driveL(i))>=30)
        m_h=m_h+1;
        heelL(m_h)=i;
        i=i+30;
    end
end

%finding toe-offs

n_t=0;
toeR=zeros(1,num);
toeL=zeros(1,num);
i=0;
while (i<length(tR)-1)
    i=i+1;
    if (abs(driveR(i))>=3) && (abs(driveR(i))<10)
        n_t=n_t+1;
        toeR(n_t)=i;
        i=i+30;
    end
end
m_t=0;
i=0;
while (i<length(tL)-1)
    i=i+1;
    if (abs(driveL(i))>=30)
        m_t=m_t+1;
        toeL(m_t)=i;
        i=i+30;
    end
end


aRt = aRight(:,2)*-g;
aRn = aRight(:,3)*g;

aLt = aLeft(:,2)*-g;
aLn = aLeft(:,3)*g;

%compensate acc drift after each step
% for i=1:1:min(n_h,m_h)
%     if (heelR(i)<heelL(i))
%         aRt (heelR(i):heelL(i)) = zeros(1,heelL(i)-heelR(i)+1);
%         aRn (heelR(i):heelL(i)) = zeros(1,heelL(i)-heelR(i)+1);
%         if i<n_h
%             aLt (heelL(i):heelR(i+1)) = zeros(1,heelR(i+1)-heelL(i)+1);
%             aLn (heelL(i):heelR(i+1)) = zeros(1,heelR(i+1)-heelL(i)+1);
%         end
%     else
%         aLt (heelL(i):heelR(i)) = zeros(1,heelR(i)-heelL(i)+1);
%         aLn (heelL(i):heelR(i)) = zeros(1,heelR(i)-heelL(i)+1);
%         if i<m_h
%             aLt (heelR(i):heelL(i+1)) = zeros(1,heelL(i+1)-heelR(i)+1);
%             aLn (heelR(i):heelL(i+1)) = zeros(1,heelL(i+1)-heelR(i)+1);
%         end
%     end
% end


%Loda gyro data
eulerRight  =  xlsread('EulerRight2.xlsx');
eulerLeft  =  xlsread('EulerLeft2.xlsx');
psiR = eulerRight(:,2) - eulerRight(1,2);
psiL = eulerLeft(:,2) - eulerLeft(1,2);
psiR = psiR*-pi/180;
psiL = psiL*-pi/180;

%image acc to the gound coordinates
%Right Foot
aR=zeros(2,length(tR));
for i=1:1:length(tR)
    transMx = [cos(psiR(i)) -sin(psiR(i)); sin(psiR(i)) cos(psiR(i))]; %transfer matrix
    aR(:,i) = transMx*[aRt(i); aRn(i)];
    if abs(aR(1,i)) < 0.1 
        aR(1,i)=0;
    end
    if abs(aR(2,i)) < 0.1 
        aR(2,i)=0;
    end
end

%Left Foot
aL=zeros(2,length(tL));
for i=1:1:length(tL)
    transMx = [cos(psiL(i)) -sin(psiL(i)); sin(psiL(i)) cos(psiL(i))]; %transfer matrix
    aL(:,i) = transMx*[aLt(i); aLn(i)];
    if abs(aL(1,i)) < 0.1 
        aL(1,i)=0;
    end
    if abs(aL(2,i)) < 0.1 
        aL(2,i)=0;
    end
end

%integration find velocity
vR = cumtrapz(tR,aR,2);
vL = cumtrapz(tL,aL,2);

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

% figure
% subplot(2,1,1);
% plot(pR(1,1200:1400)*100,pR(2,1200:1400)*100);
% xlabel('x (cm)');
% ylabel('y (cm)');
% 
% subplot(2,1,2);
% plot(pL(1,:)*100,pL(2,:)*100);
% xlabel('x (cm)');
% ylabel('y (cm)');
% 
% figure
% plot(tR,vR(1,:));
% title('Vx Right foot(m/s)');
% 
% figure
% plot(tR,vR(2,:));
% title('Vy Right foot(m/s)');

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