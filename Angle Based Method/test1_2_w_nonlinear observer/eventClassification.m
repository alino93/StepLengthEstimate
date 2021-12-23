%classification with real free fall and step rms estimation from y direction
%upd: walk st/ed is in two columns for right and left
%upd: turn is 100T and near fall is n>=1 
%*** upd: sit detection
%       aC = [azChest -axChest]
%       DeltaY function
flag = 0; %start from sit
azRight = dataR(:,6)*g;
azLeft = dataL(:,6)*g;
axRight=aRight(:,1);
ayRight=aRight(:,2);
axLeft=aLeft(:,1);
ayLeft=aLeft(:,2);
axChest=aChest(:,1);
ayChest=aChest(:,2);
azChest=aChest(:,3);
Yaw=yawC;
aC = [azChest -axChest];
pitchCE = Tilt_Estimation(aC, pitchCRate, 1);
%events column 1: 0-unknown 0-stand 1-walking 2-sitting 3-turning 4-near
%fall forward 5-lie down 6-bending 7-near fall backward(slip) 8-fall imapact 
%events column 2: walk:1-right foot toe off 2-right foot swing 
%3-left foot toe off 4-left foot swing 7-walk and sit

%note sitting in rack chairs may be different and need new event definition
t = (0:T:T*(length(axRight)-1));
events = zeros(length(t),4);

axCM = zeros(length(t),1);
ayCM = zeros(length(t),1); 
azCM = zeros(length(t),1);

aRV = zeros(length(t),1);
aLV = zeros(length(t),1);
armsR = zeros(length(t),1);
armsL = zeros(length(t),1);
aRVar = zeros(length(t),1);
azCV = zeros(length(t)-11,1);
Yawdif = zeros(length(t),1);
aCT = zeros(length(t),1);
for i = 1:1:length(t)-86

   axCM(i) = mean(axChest(i:i+10));
   ayCM(i) = mean(ayChest(i:i+10));
   azCM(i) = mean(azChest(i:i+10));
   
   
   aRV(i) = var(axRight(i:i+10)+ayRight(i:i+10), 1);
   aLV(i) = var(axLeft(i:i+10)+ayLeft(i:i+10), 1);
 
   armsR(i) = rms( [var(axRight(i:i+5)) , var(ayRight(i:i+5))] );
   armsL(i) = rms( [var(axLeft(i:i+5)) , var(ayLeft(i:i+5))] );
   azCV(i) = var(azChest(i:i+20), 1);
   aCT(i) = sqrt(axCM(i)^2 + ayCM(i)^2 + azCM(i)^2);    
end
aC = [azCM -axCM];
aRmax = islocalmax(armsR,'MinSeparation',70);
aLmax = islocalmax(armsL,'MinSeparation',80);

for i = 1:1:length(t)-301
       Yawdif(i) = var(Yaw(i:i+200)); %- asin(sin(pi/180*Yaw(i)));
       % for gyro just integrate the gyro and check the above value
       aRVar(i) = var(ayRight(i:i+300), 1);
end
% % FIND DRIVEING
% i = 0;
% while i < (length(t)-301)
%     i = i + 1;
%     if (aRVar(i) > 29)          
%         while (aRVar(i) > 25) && i < (length(t)-301)
%             events(i,1) = 11;
%             i = i + 1;
%         end
%         events(i:i+299,1) = ones(300,1)*11;
%         i = i + 300;
%     end
% end
%FIND WALKING
i = 0;
Dr = 5;%10%5
Dl = 0;
DSr=90;%90%60
DSl=75;%75%75
while i < (length(t)-200)
    i = i + 1;
    if i>3200
        Dr = 5;%5%10
        DSr=50;%50%70
    end
    if armsR(i)*aRmax(i)>5
        events(i+Dr,2) =12;
        events(i+Dr-DSr,2)=11;
        events(i+Dr-DSr:i+Dr+24,1) = ones(25+DSr,1)*1;
        events(i+Dr-DSr:i+Dr+24,4) = ones(25+DSr,1)*1;
        i = i + 30;
    end
end
i = 0;
while i < (length(t)-200)
    i = i + 1;
    if aLV(i)*aLmax(i)>5
        events(i+Dl,3) =22;
        events(i+Dl-DSl,3)=21;
        events(i+Dl-DSl:i+Dl+24,1) = ones(25+DSl,1)*1;
        events(i+Dl-DSl:i+Dl+24,4) = ones(25+DSl,1)*1;
        i = i + 30;
    end
end
i = 0;
while i < (length(t)-200)
    i = i + 1;
    if Yawdif(i)>300                         %turning
        if Yawdif(i)<50           % compensate for almost 1 sec delay
            i = i + 100;
        end
        while (Yawdif(i)>270) %&&  events(i,1) ~= 11
            events(i,1) = 3;
            i = i + 1;
        end
        events(i:i+299,1) = ones(300,1)*3;
        i = i + 299;
    end
    %---------------------------------------look for lie on back
    j = i;
    while abs(axCM(j))<3.5 && azCM(j) > 9 && j < length(t)
        j = j + 1;
    end
    if j-i > 200                 
        while abs(axCM(i)) < 3.5 && i < length(t)
            events(i,1) = 4;
            i = i + 1;
        end
        events(i:i+49,1) = ones(50,1)*4;
        i = i + 50;
        DyC = -1;
        while DyC < 1 && i < length(t)-200
            while azCM(i) > -6 && i < length(t)-200             %look for sit after lie 
                events(i,1) = 2;
                i = i + 1;
            end
            j = i;
            while (azCM(j) <= -5) && j < length(t)-200
                j = j + 1;
            end
            DyC = DeltaY(T, aC(i-100:j+100,:), pitchCE(i-100:j+100));
            events(i:j+49,1) = ones(50+j-i,1)*2;
            i = j + 49;
        end
    end
    if azCM(i) < -5  || (flag==1)        %Bend detect: sit start or bend start and cont until stands again
        %----------------------------------------look for sit
        j = i;
        while (azCM(j) < -5) && j < length(t)-200
            j = j + 1;
        end
        DyC = 0;
        DyR = 0;
        DyL = 0;
        if j-i < 201 && j-i > 80 
              i;
              j;
              DyC = DeltaY(T, aC(i-50:j+100,:), pitchCE(i-50:j+100));   % checj Dy of chest (before bend til after bend)
              DyR = DeltaYF(T, aR(:,i+130:j-50));                     % check Dy of feet during sit(maybe stairs!)
              DyL = DeltaYF(T, aL(:,i+130:j-50));                       
        end
        if j-i < 301 && DyC < -0.8 && (DyR<-0.01 || DyL<-0.01)%(DyR<-0.15 || DyL<-0.15) % should actually be -0.15 to be step!
            events(i:j+49,1) = ones(50+j-i,1)*6;
            i = j + 50;
        elseif (j-i < 301 && DyC < -0.8 ) || (flag==1)
            events(i:i+199,1) = ones(200,1)*2;
            i = i + 200;
            while (DyC < 1 && i < length(t)-200 ) || (flag==1)     % check if stands (Dy Positive to stop)
                while azCM(i) > -5 && i < length(t)-200
                    events(i,1) = 2;
                    if abs(axCM(i)) < 3.5 && azCM(i) > 9        %look for lie after sit
                       while abs(axCM(i)) < 3.5 && i < length(t)-200
                            events(i,1) = 4;
                            i = i + 1;
                       end
                       events(i:i+49,1) = ones(50,1)*4;
                       i = i + 49;
                       while azCM(i) > -5 && i < length(t)-200             %look for sit after lie 
                            events(i,1) = 2;
                            i = i + 1;
                       end
                    end
                    i = i + 1;
                end
                j = i;
                while (azCM(j) <= -5) && j < length(t)-200
                    j = j + 1;
                end
                DyC = DeltaY(T, aC(i-100:j+100,:), pitchCE(i-100:j+100));   % check if Dy becomes positive to stand
                if DyC>1 
                    flag = 0;
                end
                events(i:j,1) = ones(1+j-i,1)*2;
                i = j + 1;
            end
        end
    end
        %----------------------------------------look for bend
    if azCM(i) < -6  
        j = i;
        while (azCM(j) < -5 || axCM(j) < -5 )&& j < length(t)-200
            j = j + 1;
        end
        if j-i > 200 && j-i < 6000
            events(i:i+199,1) = ones(200,1)*5;
            i = i + 200;
            while (azCM(i) < -5 || axCM(i) > 5 )&& i < length(t)-200
                events(i,1) = 5;
                i = i + 1;
            end
            events(i:i+49,1) = ones(50,1)*5;
            i = i + 49; 
        end
        % if bending took so long (1 min) it's probably lie on stomach
        if j-i >= 6000        
            while abs(axCM(i)) < 3.5 && i < length(t)-200
                events(i,1) = 4;
                i = i + 1;
            end
            events(i:i+49,1) = ones(50,1)*4;
            i = i + 50;
            while azCM(i) > -6 && i < length(t)-200             %look for sit after lie 
                events(i,1) = 2;
                i = i + 1;
            end
            events(i:i+49,1) = ones(50,1)*2;
            i = i + 49; 
        end
    end
        %---------------------------------------look for near fall from forward bend
    if azCM(i) < -5  && events(i-1,1) ~= 2  %&&  events(i,1) ~= 11    % tune this parameter!
        n = 0;
        for  j = i-50:1:i+150                      % tune this parameter!
            if events(j,2) == 11 || events(j,3) == 21
                n = n + 1;
            end   
        end
        if n >= 1                               % tune this parameter!
            events(i:i+99,1) = ones(100,1)*7;
            %events(i,2) = n; % number of steps after near fall event in the next 1 sec
            i = i + 100;
        end
    end
    %-----------------------------------lateral bend ----look for near fall
    if abs(ayCM(i)) > 6             % tune this parameter!
        n = 0;
        for  j = i:1:i+70               % tune this parameter!
            if events(j,2) == 11 || events(j,3) == 21
                n = n + 1;
            end   
        end
        if n > 1                        % tune this parameter!
            events(i:i+99,1) = ones(100,1)*8;
            %events(i,2) = n; % number of steps after near fall event in the next 1 sec
            i = i + 100;
        end
    end
    %-----------------------------------backward bend --- look for slip ----look for near fall
    if azCM(i) > 5                    % tune this parameter!
        n = 0;
        for  j = i-50:1:i+80               % tune this parameter!
            if events(j,2) == 11 || events(j,3) == 21
                n = n + 1;
            end   
        end
        if n >= 1                        % tune this parameter!
            events(i:i+99,1) = ones(100,1)*9;
            %events(i,2) = n; % number of steps after near fall event in the next 1 sec
            i = i + 100;
        end
    end
end
%----------------------------- real fall
i = 200;
while  i <length(t)-200
    i = i + 1;
    ok = 0;
    if abs(aCT(i)) < 2            %fall event from free fall and lie down or bend for 1 sec from any direction
        for  j = i:1:i+200
            if abs(azCM(j)) > 5 || abs(ayCM(j)) > 5 || events(j,1) == 5
                ok = ok + 1;
            end   
        end
        if ok > 100
            events(i-10:i+50,1) = ones(61,1)*10;
        end
        i = i + 200;
    end
end

% FIND DRIVEING
i = 0;
while i < (length(t)-301)
    i = i + 1;
    if (aRVar(i) > 100)          
        while (aRVar(i) > 25) && i < (length(t)-301)
            events(i,1) = 11;
            i = i + 1;
        end
        events(i:i+299,1) = ones(300,1)*11;
        i = i + 300;
    end
end
% figure(1)
% plot(t(1:end-11),axRM);
% title('Mean of Right ankle horizaontal acceleration');
% xlabel('time (s)');
% ylabel('axRM (m/s^2)');
% 
% figure(2)
% plot(t(1:end-11),axLM);
% title('Mean of Left ankle horizaontal acceleration');
% xlabel('time (s)');
% ylabel('axLM (m/s^2)');
% 
figure(19)
plot(t,azCM);
title('Mean of Chest horizaontal acceleration');
xlabel('time (s)');
ylabel('azCM (m/s^2)')
% 
figure(18)
plot(t,ayCM);
title('Mean of Chest lateral acceleration');
xlabel('time (s)');
ylabel('ayCM (m/s^2)')
% 
figure(5)
plot(t,axCM);
title('Mean of Chest vertical acceleration');
xlabel('time (s)');
ylabel('axCM (m/s^2)')

figure(100)
plot(t,aRVar);
title('RMS of foot acceleration');
xlabel('time (s)');
ylabel('aRMS (m/s^2)')
% figure(11)
% plot(t(1:end-11),aRV);
% title('Variance of Right ankle horizaontal acceleration');
% xlabel('time (s)');
% ylabel('axRV (m^2/s^4)')
% 
% figure(12)
% plot(t(1:end-11),aLV);
% title('Variance of Left ankle horizaontal acceleration');
% xlabel('time (s)');
% ylabel('axLV (m^2/s^4)')
% 
% figure(13)
% plot(t(1:end-11),azCV);
% title('Variance of Chest horizaontal acceleration');
% xlabel('time (s)');
% ylabel('axCV (m^2/s^4)')
% 
figure(16)
plot(t,Yaw);
title('Chest yaw angle');
xlabel('time (s)');
ylabel('Yaw (deg)')

figure(17)
plot(t,Yawdif);
title('RMS of Chest yaw angle');
xlabel('time (s)');
ylabel('YawV (deg^2)')

% figure(18)
% plot(t,pitch);
% title('Chest pitch angle');
% xlabel('time (s)');
% ylabel('Pitch (deg)')
figure(1)
plot(t,ayRight);
title('Right ankle vertical acceleration');
xlabel('time (s)');
ylabel('ayR (m/s^2)');

figure(2)
plot(t,ayLeft);
title('Left ankle vertical acceleration');
xlabel('time (s)');
ylabel('ayL (m/s^2)');

figure(11)
plot(t,armsR,t(aRmax),armsR(aRmax),'r*');
title('RMS of Right ankle acceleration');
xlabel('time (s)');
ylabel('aRrms (m^2/s^4)')

figure(12)
plot(t,armsL,t(aLmax),armsL(aLmax),'r*');
title('RMS of Left ankle acceleration');
xlabel('time (s)');
ylabel('aLrms (m^2/s^4)')


figure(13)
plot(t,floor(armsR*100));
title('Timewise RMS of right ankle acceleration');
xlabel('time (s)');
ylabel('armsR (m/s^2)')

figure(14)
plot(t,armsL);
title('Timewise RMS of left ankle acceleration');
xlabel('time (s)');
ylabel('armsL (m/s^2)')

figure(20)
plot(t,events(:,1),'LineWidth',1.5)
title('Events');
xlabel('time (s)');
dim = [0.91 0.6 0.3 0.3];
str = {'0 : Standing','1 : Walking','2 : Sitting','3 : Turning','4 : Lying down','5 : Bending','6 : climbing/stairs','7 : Near fall forward','8 : Near fall lateral','9 : Near fall backward/slip','10 : Free Fall' };
annotation('textbox',dim,'String',str,'FitBoxToText','on');
% theNumber = 5;
% myText = sprintf('%.2f Event ', theNumber);
% text(t, events(:,1), myText);