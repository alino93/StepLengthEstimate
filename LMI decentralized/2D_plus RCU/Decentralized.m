Ts = 0.01;
g = 9.8;
g_v = g*[0;0;1];
events = events;

% input from IMU
f1 = dataR(:,2:4);
f1(:,1) = dataR(:,2); % N = x
f1(:,2) = dataR(:,4); % E = y
f1(:,3) = -dataR(:,3); % D = z
f1 = f1' * g/1000;

w1 = dataR(:,5:7);
w1(:,1) = dataR(:,5); % N = x
w1(:,2) = dataR(:,7); % E = y
w1(:,3) = -dataR(:,6); % D = z
w1 = w1' * pi/180;

f2 = dataL(:,2:4);
f2(:,1) = dataL(:,2); % N = x
f2(:,2) = dataL(:,4); % E = y
f2(:,3) = -dataL(:,3); % D = z
f2 = f2' * g/1000;

w2 = dataL(:,5:7);
w2(:,1) = dataL(:,5); % N = x
w2(:,2) = dataL(:,7); % E = y
w2(:,3) = -dataL(:,6); % D = z
w2 = w2' * pi/180;

n = length(f1(1,:));

% bias and noise statistics
tue_a = 500;%correleation time constant
tue_g = 300;
sig_bad = 1.8e-3*g;%stdv markov bias
sig_bgd = 0.07*pi/180;
Sa = 180e-6*g;%root power spectral density noise
Sg = 0.007*pi/180;
Sbad = 2*sig_bad^2/tue_a;%power spectral density for stdv markov bias
Sbgd = 2*sig_bgd^2/tue_g;
sigma_initial_acc_bias = 0.01;
sigma_initial_gyro_bias = 0.1*pi/180;

% design and tuning
gamma = 2;%step max length
sig_zupt = 0.001;%Tuning parameter
tue = 5; % min time period for RCU execution (ensure lagrange mul convergance)
sig_rcu = 1;
m0 = -gamma^2;

% some needed variables
x = zeros(30,n);
x1 = zeros(15,n);
x2 = zeros(15,n);
I30 = eye(30);
I15 = eye(15);
I3 = eye(3); 
z3_15 = zeros(3,15);
z3_12 = zeros(3,12);
z3_9 = zeros(3,9);
z3 = zeros(3,3);
z15 = zeros(15,15);
z15_12 = zeros(15,12);
H_zupt = [z3 I3 z3 z3 z3];
R_zupt = sig_zupt*I3;

% MeM models

H_rcu = [I3 z3 z3 z3 z3];

R_rcu = I3*sig_rcu^2;



% zupt detector
zupt = zeros(2,n);
edr = 0;
while edr<n
    i = edr + 1;
    str = find(events(i:end,2)==11,1,'first') + i-1;
    edr = find(events(i+1:end,2)==12,1,'first') + i;
    zupt(1,str:edr)=ones(1,edr-str+1);
end
edl = 0;
while edl<n
    i = edl + 1;
    stl = find(events(i:end,3)==21,1,'first') + i-1;
    edl = find(events(i+1:end,3)==22,1,'first') + i;
    zupt(2,stl:edl)=ones(1,edl-stl+1);
end


% initialize
P = I30;
P1 = zeros(15,15);
%  Initial state covariance matrix
P1(10:12,10:12)=diag(sigma_initial_acc_bias.^2);
P1(13:15,13:15)=diag(sigma_initial_gyro_bias.^2);
P2 = P1;
% Process noise covariance matrix
Q1 = [Sa*I3 z3 z3 z3;
     z3 Sg*I3 z3 z3;
     z3 z3 Sbad*I3 z3;
     z3 z3 z3 Sbgd*I3];
Q2 = Q1;
Q = [Q1 zeros(12,12);zeros(12,12) Q2];


r1 = [0;0;0;];
v1 = r1;
%initial mounting angle
ge1 = sqrt( f1(1,1)^2 +  f1(3,1)^2);
pitch0_1 = sign(f1(1,1))*acos( f1(3,1)/ge1);
psi1 = [0;pitch0_1;0];%roll pitch yaw
%initial transfer matrix
Cbr1 = eul2Cbn(psi1);
%inital bias
ba1 = -mean(f1,2); %we cannot do this due to gravity
ba1 = [ba1(1);ba1(2);0];%zeros(3,1);
bg1 = -mean(w1,2);

r2 = [0;0;0;];
v2 = r2;
%initial mounting angle
ge2 = sqrt( f2(1,1)^2 +  f2(3,1)^2);
pitch0_2 = sign(f2(1,1))*acos( f2(3,1)/ge2);
psi2 = [0;pitch0_2;0];%roll pitch yaw
%initial transfer matrix
Cbr2 = eul2Cbn(psi2);
%inital bias
ba2 = -mean(f2,2); %we cannot do this due to gravity
ba2 = [ba2(1);ba2(2);0];%zeros(3,1);
bg2 = -mean(w2,2);

x(:,1) = [r1;v1;psi1;ba1;bg1;r2;v2;psi2;ba2;bg2];

for k = 2:1:n
    %[x(:,k), F, G] = mechanize(x(:,k-1));
    
    %mechanize
    Cbr1 = Cbr1*(I3+skew(w1(:,k)+x(13:15,k-1))*Ts);
    ark1 = Cbr1*(f1(:,k)+x(10:12,k-1)) + g_v;
    vrk1 = x(4:6,k-1);
    wrk1 = Cbr1*(w1(:,k)+x(13:15,k-1));
    bdak1 = -x(10:12,k-1)/tue_a;
    bdgk1 = -x(13:15,k-1)/tue_g;
    Cbr2 = Cbr2*(I3+skew(w2(:,k)+x(28:30,k-1))*Ts);
    ark2 = Cbr2*(f2(:,k)+x(25:27,k-1)) + g_v;
    vrk2 = x(19:21,k-1);
    wrk2 = Cbr2*(w2(:,k)+x(28:30,k-1));
    bdak2 = -x(25:27,k-1)/tue_a;
    bdgk2 = -x(28:30,k-1)/tue_g;
    
    % next step (time update)
    x1(:,k) = x1(:,k-1) + Ts * [vrk1+0.5*ark1*Ts;ark1;wrk1;bdak1;bdgk1;];
    x1(7:9,k) = Cbn2eul(Cbr1);
                             
    x2(:,k) = x2(:,k-1) + Ts * [vrk2+0.5*ark2*Ts;ark2;wrk2;bdak2;bdgk2;];
    x2(7:9,k) = Cbn2eul(Cbr2);
    % find F (linearize)
    %sk1 = skew(w1(:,k))/norm(w1(:,k))*Cbr1;
    
    F1 = [z3 I3 z3 z3 z3;
          z3 z3 skew(Cbr1*f1(:,k)) Cbr1 z3;
          z3 z3 z3 z3 -Cbr1;
          z3 z3 z3 -I3/tue_a z3;
          z3 z3 z3 z3 -I3/tue_g];
    F1 = eye(size(F1)) + Ts*F1;
    %sk2 = skew(w2(:,k))/norm(w2(:,k))*Cbr2;
    F2 = [z3 I3 z3 z3 z3;
          z3 z3 skew(Cbr2*f2(:,k)) Cbr2 z3;
          z3 z3 z3 z3 -Cbr2;
          z3 z3 z3 -I3/tue_a z3;
          z3 z3 z3 z3 -I3/tue_g];
    F2 = eye(size(F2)) + Ts*F2;
    F = [F1 z15;z15 F2];
    
    
    % find L
    L1 = [z3 z3 z3 z3;
          Cbr1 z3 z3 z3;
          z3 -Cbr1 z3 z3;
          z3 z3 I3 z3;
          z3 z3 z3 I3];
    L1 = Ts*L1;
    L2 = [z3 z3 z3 z3;
          Cbr2 z3 z3 z3;
          z3 -Cbr2 z3 z3;
          z3 z3 I3 z3;
          z3 z3 z3 I3];
    L2 = Ts*L2;
    L = [L1 z15_12;z15_12 L2];
    
    % covariance matrix
    P1 = F1*P1*F1' + L1*Q1*L1';
    P1=(P1+P1')/2;
    P2 = F2*P2*F2' + L2*Q2*L2';
    P2=(P2+P2')/2;
    % measurement update
    if zupt(1,k)==0
        H = H_zupt;
        v = x1(4:6,k);
        R = R_zupt;
        K = P1*H'*(H*P1*H' + R)^-1;
        dx = -K*v;
        x1(:,k) = x1(:,k) + dx;
        %x1(:,k) = comp_internal_states(x1(:,k),dx);
        P1 = (I15 - K*H)*P1;
        rho = norm(x1(1:3,k)-x2(1:3,k));
        if rho>gamma && zupt(2,k)==1
            p = 1/rho*(gamma*x2(1:3,k)+(rho-gamma)*x1(1:3,k));
            K_rcu = P2*H_rcu'*(H_rcu*P2*H_rcu' + R_rcu)^-1;
            dx = -K_rcu*(x2(1:3,k)-p);
            x2(:,k) = x2(:,k) + dx;
            %x2(:,k) = comp_internal_states(x2(:,k),dx);
            P2 = (I15 - K_rcu*H_rcu)*P2;
        end
    end
    if zupt(2,k)==0
        H = H_zupt;
        v = x2(4:6,k);
        R = R_zupt;
        K = P2*H'*(H*P2*H' + R)^-1;
        dx = -K*v;
        x2(:,k) = x2(:,k) + dx;
        %x2(:,k) = comp_internal_states(x2(:,k),dx);
        P2 = (I15 - K*H)*P2;
        rho = norm(x1(1:3,k)-x2(1:3,k));
        if rho>gamma && zupt(1,k)==1
            p = 1/rho*(gamma*x1(1:3,k)+(rho-gamma)*x2(1:3,k));
            K_rcu = P1*H_rcu'*(H_rcu*P1*H_rcu' + R_rcu)^-1;
            dx = -K_rcu*(x1(1:3,k)-p);
            x1(:,k) = x1(:,k) + dx;
            %x1(:,k) = comp_internal_states(x1(:,k),dx);
            P1 = (I15 - K_rcu*H_rcu)*P1;
        end
    end
    x = [x1;x2];
end

%% 
t = 0:Ts:(n-1)*Ts;
figure(40)
subplot(2,2,2);
plot(t(1:4000),x1(1,1:4000),'LineWidth',2);
title('Right foot North displacement');
xlabel('t (s)');
ylabel('x (m)');

subplot(2,2,4);
plot(t(1:4000),x1(2,1:4000),'LineWidth',2);
title('Right foot East displacement');
xlabel('t (s)');
ylabel('x (m)');

subplot(2,2,1);
plot(t(1:4000),x2(1,1:4000),'LineWidth',2);
title('Left foot North displacement');
xlabel('t (s)');
ylabel('x (m)');

subplot(2,2,3);
plot(t(1:4000),x2(2,1:4000),'LineWidth',2);
title('Left foot East displacement');
xlabel('t (s)');
ylabel('x (m)');

figure(41)
plot(x1(2,:),x1(1,:),x2(2,:),x2(1,:),'LineWidth',2);
title('Both foot North-East displacement');
legend('Right','Left');
xlabel('East (m)');
ylabel('North (m)');

%%
function [x_out]=comp_internal_states(x,dx)

    % Convert quaternion to a rotation matrix
    R=eul2Cbn(x(7:9));

    % Correct the state vector
    x_out=x+dx;

    % Correct the rotation matrics
    epsilon=dx(7:9);
    R=(eye(3)-skew(epsilon))*R;

    % Get the corrected roll, pitch and heading from the corrected rotation
    % matrix
    PSI_nb = Cbn2eul(R);
    x_out(7:9)=PSI_nb;
    %x_out(8)=PSI_nb;
    %x_out(9)=PSI_nb;

end