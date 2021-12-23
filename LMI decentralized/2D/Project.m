Ts = 0.01;
g = 9.8;
g_v = g*[0;0;1];
events = events;
t_rcu = 0;

% input from IMU
f1 = dataR(:,2:4);
f1(:,1) = dataR(:,2); % N = x
f1(:,2) = dataR(:,4); % E = y
f1(:,3) = -dataR(:,3); % D = y
f1 = f1' * g/1000;

w1 = dataR(:,5:7);
w1(:,1) = dataR(:,5); % N = x
w1(:,2) = dataR(:,7); % E = y
w1(:,3) = -dataR(:,6); % D = y
w1 = w1' * pi/180;

f2 = dataL(:,2:4);
f2(:,1) = dataL(:,2); % N = x
f2(:,2) = dataL(:,4); % E = y
f2(:,3) = -dataL(:,3); % D = y
f2 = f2' * g/1000;

w2 = dataL(:,5:7);
w2(:,1) = dataL(:,5); % N = x
w2(:,2) = dataL(:,7); % E = y
w2(:,3) = -dataL(:,6); % D = y
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

% design and tuning
gamma = 2;%step max length
sig_zupt = 0.01;%Tuning parameter
tue = 10; % min time period for RCU execution (ensure lagrange mul convergance)
sig_MeM1 = 1;
sig_MeM2 = 1;
lambda = 1;
m0 = -gamma^2;

% some needed variables
x = zeros(30,n);
I30 = eye(30);
I3 = eye(3); 
z3_15 = zeros(3,15);
z3_12 = zeros(3,12);
z3_9 = zeros(3,9);
z3 = zeros(3,3);
z15 = zeros(15,15);
z15_12 = zeros(15,12);
H_zupt = [z3 I3 z3 z3 z3];
R_zupt = sig_zupt*I3;
Q1 = [Sa*I3 z3 z3 z3;
     z3 Sg*I3 z3 z3;
     z3 z3 Sbad*I3 z3;
     z3 z3 z3 Sbgd*I3];
Q2 = Q1;
Q = [Q1 zeros(12,12);zeros(12,12) Q2];
% MeM models
D = [I3 z3_12 -I3 z3_12];
H1 = [I3 z3 z3_9;z3 I3 z3_9];
R1 = [I3 z3;z3 I3]*sig_MeM1;
H2 = D;
R2 = I3*sig_MeM2;
Hi = [H1 zeros(6,15);zeros(6,15) H1];
Ri = [R1 zeros(6,6);zeros(6,6) R1];


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


% initialize states
P = I30;
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
ba1 = zeros(3,1);
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
ba2 = zeros(3,1);
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
    
    % next step
    x(:,k) = x(:,k-1) + Ts * [vrk1+0.5*ark1*Ts;ark1;wrk1;bdak1;bdgk1;
                              vrk2+0.5*ark2*Ts;ark2;wrk2;bdak2;bdgk2;];
    % find F (linearize)
    sk1 = skew(w1(:,k))/norm(w1(:,k))*Cbr1;
    
    F1 = [z3 I3 z3 z3 z3;
          z3 z3 Cbr1*skew(f1(:,k)) I3 z3;
          z3 z3 Cbr1*skew(w1(:,k)) z3 I3;
          z3 z3 z3 -I3/tue_a z3;
          z3 z3 z3 z3 -I3/tue_g];
    sk2 = skew(w2(:,k))/norm(w2(:,k))*Cbr2;
    F2 = [z3 I3 z3 z3 z3;
          z3 z3 Cbr2*skew(f2(:,k)) I3 z3;
          z3 z3 Cbr2*skew(w2(:,k)) z3 I3;
          z3 z3 z3 -I3/tue_a z3;
          z3 z3 z3 z3 -I3/tue_g];
    F = [F1 z15;z15 F2];
    
    % find L
    L1 = [z3 z3 z3 z3;
          Cbr1 z3 z3 z3;
          z3 Cbr1 z3 z3;
          z3 z3 I3 z3;
          z3 z3 z3 I3];
    L2 = [z3 z3 z3 z3;
          Cbr2 z3 z3 z3;
          z3 Cbr2 z3 z3;
          z3 z3 I3 z3;
          z3 z3 z3 I3];
    L = [L1 z15_12;z15_12 L2];
    
    % covariance matrix
    P = F*P*F' + L*Q*L';
    
    if zupt(1,k)==1 || zupt(2,k)==1
        H = [H_zupt*zupt(1,k) z3_15;z3_15 H_zupt*zupt(2,k)];
        v = [x(4:6,k)*zupt(1,k); x(19:21,k)*zupt(2,k)];
        R = [R_zupt*zupt(1,k) z3; R_zupt*zupt(2,k) z3];
        K = P*H'*(H*P*H' + R)^-1;
        dx = -K*v;
        x(:,k) = x(:,k) + dx;
        P = (I30 - K*H)*P;
    end
    rho = norm(x(1:3,k)-x(16:18,k));
    if rho>gamma && k-t_rcu>=tue
        W_inv = P^-1 + Hi'*Ri^-1*Hi;
        W = W_inv^-1;
        M = D'*D;
        
        delta_lambda = 1;
        lambda = 0;%Lagrange multiplier parameter
        iter = 0;
        while abs(delta_lambda)>1e-1 && iter<100
            iter = iter + 1;
            G = chol(W);
            LG_1 = D*G^-1;
            [U,s,V] = svd(LG_1);
            e = V'*(G')^-1*W*x(:,k);
            sum_q_p = 0;
            sum_q_L = 0;
            for i=1:1:3
                sum_q_p = sum_q_p + e(i)^2*s(i,i)^4/(1+lambda*s(i,i)^2)^3;
                sum_q_L = sum_q_L + e(i)^2*s(i,i)^2/(1+lambda*s(i,i)^2)^2;
            end
            q_L = sum_q_L + m0;
            dq_L = -2*sum_q_p;
            delta_lambda = -q_L/dq_L;
            lambda = lambda + delta_lambda;
        end
        Akh = (W + lambda*M)^-1;
        px = Akh * W * x(:,k);
        x(:,k) = px;
        q = M*x(:,k);
        gk = x(:,k)'*q + m0;
        
        Dp = (I30 - Akh*(q*q')/(q'*Akh*q))*gk;
        P = Dp*P*Dp';
        t_rcu = k;
    end
end
%% 
t = 0:T:(n-1)*T;
figure(40)
subplot(2,2,2);
plot(t,x(1,:),'LineWidth',2);
title('Right foot North displacement');
xlabel('t (s)');
ylabel('x (m)');

subplot(2,2,4);
plot(t,x(2,:),'LineWidth',2);
title('Right foot East displacement');
xlabel('t (s)');
ylabel('x (m)');

subplot(2,2,1);
plot(t,x(16,:),'LineWidth',2);
title('Left foot North displacement');
xlabel('t (s)');
ylabel('x (m)');

subplot(2,2,3);
plot(t,x(17,:),'LineWidth',2);
title('Left foot East displacement');
xlabel('t (s)');
ylabel('x (m)');


figure(41)
plot(x(2,:),x(1,:),x(17,:),x(16,:),'LineWidth',2);
title('Both foot North-East displacement');
legend('Right','Left');
xlabel('East (m)');
ylabel('North (m)');