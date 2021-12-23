%classification
axRight=aRight(:,1);
axLeft=aLeft(:,1);
azChest=aChest;
Yaw=yawC*180/pi;
%events column 1: 0-stand 1-walking 2-sitting 3-turning 4-fall 5-lie down
%events column 2: walk:1-right foot toe off 2-right foot swing 
%3-left foot toe off 4-left foot swing 7-walk and sit

%note sitting in rack chairs may be different and need new event definition

events = zeros(length(t),2);
axRM = zeros(length(t)-11,1);
axLM = zeros(length(t)-11,1);
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
   azCM(i) = mean(azChest(i:i+10));
   
   
   axRV(i) = var(axRight(i:i+50));
   axLV(i) = var(axLeft(i:i+50));
   azCV(i) = var(azChest(i:i+20));

%    if dyawM > 10 
%        event(i) = 6;
%    end
%    if axRSD > 0.2 && event(i-1) ~= 6
%        event = 2;
       
end
for i = 1:1:length(t)-101

end
Yawnorm=asin(sin(pi/180*Yaw));
for i = 1:1:length(t)-201
       Yawdif(i) = var(Yawnorm(i:i+200)); %- asin(sin(pi/180*Yaw(i)));
       % for gyro just integrate the gyro and check the above value
end
i = 0;
while i < (length(t)-200)
    i = i + 1;
    if Yawdif(i)>0.025                          %turning
        events(i:i+100,1) = ones(101,1)*3;
        i = i + 100;
    end
    if (axRV(i) > 1 || axLV(i) > 1) && events(i-1) ~=3         %walking
        events(i:i+20,1) = ones(21,1)*1;
        i = i + 20;
    end
    if azCV(i) > 2                              %sit starts and cont until stands again
        events(i:i+49,1) = ones(50,1)*2;
        i = i + 50;
        while azCV(i) < 2
            events(i,1) = 2;
            i = i + 1;
            if azCM(i) < -10                   % look for lying after sit
                events(i:i+99,1) = ones(100,1)*5;
                i = i + 100;
                while azCM(i) < -10
                    events(i,1) = 5;
                    i = i + 1;
                end
                events(i:i+9,1) = ones(10,1)*2;
                i = i + 10;
            end
        end
        i = i + 15;
    end
    if axRM(i) > 10 || axLM(i) > 10             %fall event
        events(i-5:i+5,1) = ones(11,1)*4;
    end
    if azCM(i) < -10                           %lie down starts and cont until stands again
        events(i:i+99,1) = ones(100,1)*5;
        i = i + 100;
        while azCM(i) < -10
            events(i,1) = 5;
            i = i + 1;
        end
        while azCV(i) < 2                       %suppose sit after lie 
            events(i,1) = 2;
            i = i + 1;
        end
        i = i + 15;
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

figure(20)
plot(t,events(:,1))
dim = [0.91 0.6 0.3 0.3];
str = {'0 : Standing','1 : Walking','2 : Sitting','3 : Turning','4 : Falling','5 : Lying down'};
annotation('textbox',dim,'String',str,'FitBoxToText','on');
% theNumber = 5;
% myText = sprintf('%.2f Event ', theNumber);
% text(t, events(:,1), myText);


