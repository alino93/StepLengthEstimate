%% Plot error vs K vs bias

%RUNeventClassification();
lbound = 10; %region lower bound (deg)
ubound = 30; %region upper bound (deg)
offset = 1; %switching (hysteresis) offset (deg)
swbound = 5; %switching bound (deg)
ERROR_M = zeros(1,20);
for i=1:1:20
    sigma = (i+0)*0.05;
    
    %time
    t = (0:T:0.01*(length(dataR(:,1))-1))';

    %accelerations
    aRight = dataR(:,4:5)*g;
    aLeft = dataL(:,4:5)*g;
    aChest = dataC(:,4:6)*g;

    aRight(:,1) = aRight(:,1);
    aLeft(:,1) = aLeft(:,1);
    aChest(:,3) = aChest(:,3);
    aRight(:,2) = aRight(:,2);
    aLeft(:,2) = aLeft(:,2)*-1;
    %gyros
    tiltRRate = dataR(:,12)* pi/180;
    tiltLRate = dataL(:,12)*-1* pi/180;
    
    % Bias calculation
    SolveLMI3();
    x_hat_R = Tilt_and_Bias_Estimation(aRight, tiltRRate, offset, swbound, L1,L2,L3);
    x_hat_L = Tilt_and_Bias_Estimation(aLeft, tiltLRate, offset, swbound, L1,L2,L3);

    biasR = x_hat_R(2:3,:);%[0 0]';
    biasL = x_hat_L(2:3,:);%[0 0]';
    biasGR = x_hat_R(4,:);%0;
    biasGL = x_hat_L(4,:);%0;
    
    aRight(1:end-115,:) = aRight(1:end-115,:) - biasR(:,116:end)';
    aLeft(1:end-115,:) = aLeft(1:end-115,:) - biasL(:,116:end)';
%     aRight = aRight - biasR';
%     aLeft = aLeft - biasL';
    
    K = 1;
    % Right ankle tilt estimation
    tiltRE=x_hat_R(1,:);

    % Left ankle tilt estimation
    tiltLE=x_hat_L(1,:);

    % Right ankle acceleration transform
    biasR3 = [0 0];
    aR = Transform_acc(aRight, tiltRE, biasR3);

    % Left ankle acceleration transform
    biasL3 = [0 0];
    aL = Transform_acc(aLeft, tiltLE, biasL3);

    [vR, pR, tR, biasR2, vL, pL, tL, biasL2] = Integrate (aR, aL, tiltRE, tiltLE, XR, XL, events, t); % Use tiltE on all steps

    % cal error
    ERROR_R2 = zeros(1,1);
    ERROR_L2 = zeros(1,1);
    for n = 1:1:length(tR)
        pRmax = max(abs(pR(1,floor(tR(n)*100-40):floor(tR(n)*100)+40)));
        ERROR_R2 = ERROR_R2 + abs( (pRmax - XR(n)) ./ pRmax );
    end
    ERROR_R2 = ERROR_R2 / length(tR); 
    for n = 1:1:length(tL)
        pLmax = max(abs(pL(1,floor(tL(n)*100-40):floor(tL(n)*100)+40)));
        ERROR_L2 = ERROR_L2 + abs( (pLmax - XL(n)) ./ pLmax );
    end
    ERROR_L2 = ERROR_L2 / length(tL);


    ERROR_M(i) = mean([ERROR_R2(1),ERROR_L2(1)]);
    
end
figure(21)
hold on
plot(0.05:0.05:1,ERROR_M*100,'Linewidth',2);
title('Mean Error vs \sigma');
xlabel('\sigma');
ylabel('Horiz Displacement Error (%)');