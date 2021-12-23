%% fallsteps
i=1;
rSF = zeros(2,1);%1:time 2:Rstep length 3:Rstep num 4:Lstep length 5:Lstep num
n = 0;
Rn = 0;
Ln = 0;
biasC = [0 0 0];

%data sign correction 
tiltCRate = dataC(:,11)* -pi/180;
tiltCRate = tiltCRate - mean(tiltCRate(1:20));
rollCRate = dataC(:,12)* pi/180;
rollCRate = rollCRate - mean(rollCRate(1:20));
aC = [dataC(:,6) -dataC(:,5) dataC(:,4)]*g;

%estimate roll and tilt angles
[tiltE, rollE] = Tilt_Estimation3D(aC, tiltCRate, rollCRate, 1);

%transfer to intertial coordinates
aB = Transform_acc3D(aC, tiltE, rollE, biasC);
aBTotal = sqrt(aB(:,1).^2+aB(:,2).^2+aB(:,3).^2);
aBrms = sqrt(aB(:,1).^2+aB(:,2).^2);%rms(aChest(:,2:3),2);
Len = length(events(:,1))-100;
while i<Len
    i = i + 30;
    if events(i,1) >= 7 && events(i,1)<10
        n = n + 1;
        rSF(n,1) = n;
        rSF(n,2) = i;
        rSF(n,3) = max(aBTotal(i - 500:i + 500));
        
        TR = find(events(i:end,2)==11,1,'first')+i-1; %find time of first R stp
        LR = events(TR,4);                    %find length of first R stp
        TL = find(events(i:end,3)==21,1,'first')+i-1; %find time of first L stp
        LL = events(TL,4);                    %find length of first L stp
        
        if TR <= TL
            rSF(n,6) = (TR-rSF(n,2));
            rSF(n,5) = LR;
            rSF(n,4) = mean(aBrms(rSF(n,2):TR));
        else
            rSF(n,6) = (TL-rSF(n,2));
            rSF(n,5) = LL;
            rSF(n,4) = mean(aBrms(rSF(n,2):TL));
        end
        %plot velocity
        st = rSF(n,2);
        ed = st + 150;
        time = 0:0.01:1.5;
        vB = cumtrapz(time,aB(st:ed,:));
        VBtotal = sqrt(vB(:,1).^2+vB(:,2).^2+vB(:,3).^2);

        figure(6)
        title('Chest horizontal velocity');
        xlabel('t (s)');
        ylabel('v_x (cm/s^2)');
        plot(time,abs(vB(:,1))*100);
        hold on

        i = i + 500;
        
    end
end

col_header = {'Num', 'Fall_time(s)', 'Peak_acc(m/s^2)', 'Mean_acc(m/s^2)', 'First_stp_len(m)','Rxn_time(s)'};
VarNames = {'Num', 'Fall_time', 'Peak_acc', 'Mean_acc', 'First_stp_len','Rxn_time'};
T = table(rSF(:,1),rSF(:,2)/100,rSF(:,3),rSF(:,4),rSF(:,5),rSF(:,6)/100, 'VariableNames',VarNames);
T
writetable(T,'fallStats.xlsx')
xlswrite('fallStats.xlsx',col_header,'Sheet1','A1');

