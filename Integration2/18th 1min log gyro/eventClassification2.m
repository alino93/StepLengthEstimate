%classification
axRight=aRight(:,1);
axLeft=aLeft(:,1);
axChest=aChest(:,1);
ayChest=aChest(:,2);
azChest=aChest(:,3);
Yaw=yawC;
%events column 1: 0-unknown 0-stand 1-walking 2-sitting 3-turning 4-near
%fall forward 5-lie down 6-bending 7-near fall backward(slip) 8-fall imapact 
%events column 2: walk:1-right foot toe off 2-right foot swing 
%3-left foot toe off 4-left foot swing 7-walk and sit

%note sitting in rack chairs may be different and need new event definition

events = zeros(length(t),2);
axRM = zeros(length(t)-11,1);
axLM = zeros(length(t)-11,1);
axCM = zeros(length(t)-11,1);
ayCM = zeros(length(t)-11,1); 
azCM = zeros(length(t)-11,1);
YawM = zeros(length(t)-51,1);
dYawM = zeros(length(t)-51,1);

axRV = zeros(length(t)-11,1);
axLV = zeros(length(t)-11,1);
azCV = zeros(length(t)-11,1);
YawV = zeros(length(t)-51,1);
dYawV = zeros(length(t)-51,1);
Yawdif = zeros(length(t)-51,1);
dYaw=diff(cos(Yaw))./diff(t);
for i = 1:1:length(t)-50
   axRM(i) = mean(axRight(i:i+10));
   axLM(i) = mean(axLeft(i:i+10));
   axCM(i) = mean(axChest(i:i+10));
   ayCM(i) = mean(ayChest(i:i+10));
   azCM(i) = mean(azChest(i:i+10));
   
   
   axRV(i) = var(axRight(i:i+50));
   axLV(i) = var(axLeft(i:i+50));
   azCV(i) = var(azChest(i:i+20));
       
end
for i = 1:1:length(t)-201
       Yawdif(i) = var(Yaw(i:i+200)); %- asin(sin(pi/180*Yaw(i)));
       % for gyro just integrate the gyro and check the above value
end
i = 0;
while i < (length(t)-200)
    i = i + 1;
    if (axRV(i) > 1)          %walking --- events col 2: Right Start:11 Right End:12
        i = i + 15;
        events(i,2) = 11;
        while (axRV(i) > 1)
            events(i,1) = 1;
            i = i + 1;
        end
        events(i:i+24,1) = ones(25,1)*1;
        events(i,2) = 12;
    end
    if (axLV(i) > 1)          %walking --- events col 2: Left Start:21 Left End:22
        i = i + 15;
        events(i,2) = 21;
        while (axLV(i) > 1)
            events(i,1) = 1;
            i = i + 1;
        end
        events(i:i+24,1) = ones(25,1)*1;
        events(i,2) = 22;
    end
end
i = 0;
while i < (length(t)-200)
    i = i + 1;
    if Yawdif(i)>25                          %turning
        if Yawdif(i)<30
            i = i + 120;
        end
        while (Yawdif(i)>25)
            events(i,1) = 3;
            i = i + 1;
        end
        events(i:i+59,1) = ones(60,1)*3;
        i = i + 60;
    end
    %---------------------------------------look for lie on back
    j = i;
    while abs(axCM(j)) < 3.5 && azCM(j) > 9 && j < length(t)
        j = j + 1;
    end
    if j-i > 200                 
        while abs(axCM(i)) < 3.5 && i < length(t)
            events(i,1) = 5;
            i = i + 1;
        end
        events(i:i+49,1) = ones(50,1)*5;
        i = i + 50;
        while azCM(i) > -6 && i < length(t)             %look for sit after lie 
            events(i,1) = 2;
            i = i + 1;
        end
        events(i:i+19,1) = ones(20,1)*2;
        i = i + 20;
    end
    if azCM(i) < -5                              %Bend detect: sit start or bend start and cont until stands again
        %----------------------------------------look for sit
        j = i;
        while (azCM(j) < -5) && j < length(t)
            j = j + 1;
        end
        if j-i < 201
            events(i:i+99,1) = ones(100,1)*2;
            i = i + 100;
            while azCM(i) > -5 && i < length(t)-200
                events(i,1) = 2;
                if abs(axCM(i)) < 3.5 && azCM(i) > 9        %look for lie after sit
                   while abs(axCM(i)) < 3.5 && i < length(t)
                        events(i,1) = 5;
                        i = i + 1;
                   end
                   events(i:i+49,1) = ones(50,1)*5;
                   while azCM(i) > -5 && i < length(t)             %look for sit after lie 
                        events(i,1) = 2;
                        i = i + 1;
                   end
                end
                i = i + 1;
            end
            events(i:i+49,1) = ones(50,1)*2;
            i = i + 49;                
        end
        
        %----------------------------------------look for bend
        j = i;
        while (azCM(j) < -5 || axCM(j) > 5 )&& j < length(t)
            j = j + 1;
        end
        if j-i > 200 && j-i < 6000
            events(i:i+199,1) = ones(200,1)*6;
            i = i + 200;
            while (azCM(i) < -5 || axCM(i) > 5 )&& i < length(t)
                events(i,1) = 6;
                i = i + 1;
            end
            events(i:i+49,1) = ones(50,1)*6;
            i = i + 49; 
        end
        % if bending took so long (1 min) it's probably lie on stomach
        if j-i >= 6000        
            while abs(axCM(i)) < 3.5 && i < length(t)
                events(i,1) = 5;
                i = i + 1;
            end
            events(i:i+49,1) = ones(50,1)*5;
            i = i + 50;
            while azCM(i) > -6 && i < length(t)             %look for sit after lie 
                events(i,1) = 2;
                i = i + 1;
            end
            events(i:i+49,1) = ones(50,1)*2;
            i = i + 49; 
        end
        %---------------------------------------look for near fall from forward bend
        n = 0;
        for  j = i:1:i+100
            if events(j,2) == 11 || events(j,2) == 21
                n = n + 1;
            end   
        end
        if n > 1
            events(i:i+99,1) = ones(100,1)*4;
            events(i,2) = n; % number of steps after near fall event in the next 1 sec
            i = i + 99;
        end
    end
    %-----------------------------------lateral bend ----look for near fall
    if abs(ayCM(i)) > 5
        n = 0;
        for  j = i:1:i+100
            if events(j,2) == 11 || events(j,2) == 21
                n = n + 1;
            end   
        end
        if n > 1
            events(i:i+99,1) = ones(100,1)*4;
            events(i,2) = n; % number of steps after near fall event in the next 1 sec
            i = i + 99;
        end
    end
    %-----------------------------------backward bend --- look for slip ----look for near fall
    if azCM(i) > 5
        n = 0;
        for  j = i:1:i+100
            if events(j,2) == 11 || events(j,2) == 21
                n = n + 1;
            end   
        end
        if n > 1
            events(i:i+99,1) = ones(100,1)*7;
            events(i,2) = n; % number of steps after near fall event in the next 1 sec
            i = i + 99;
        end
    end
end
%----------------------------- Climbing/stairs
for  i = 200:1:length(t)-200
    %calculate delta_y of feet
end
%----------------------------- fall impacts
for  i = 200:1:length(t)-200
    if abs(axRM(i)) > 15 || abs(axLM(i)) > 15 || abs(azCM(i)) > 15 || abs(ayCM(i)) > 15            %fall event from impact
        events(i-10:i+50,1) = ones(61,1)*8;
    end
end
figure(1)
plot(t(1:end-11),axRM);

% figure(2)
% plot(t(1:end-11),axLM);
% 
figure(3)
plot(t(1:end-11),azCM);

figure(11)
plot(t(1:end-11),axRV);

figure(12)
plot(t(1:end-11),axLV);

figure(13)
plot(t(1:end-11),azCV);

figure(16)
plot(t(1:end-51),Yawdif);

figure(17)
plot(t,Yaw);

figure(20)
plot(t,events(:,1))
dim = [0.91 0.6 0.3 0.3];
str = {'0 : Standing','1 : Walking','2 : Sitting','3 : Turning','4 : Falling','5 : Lying down'};
annotation('textbox',dim,'String',str,'FitBoxToText','on');
% theNumber = 5;
% myText = sprintf('%.2f Event ', theNumber);
% text(t, events(:,1), myText);


