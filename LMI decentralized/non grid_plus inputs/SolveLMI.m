%solve LMI for bias estimation
function [L]=SolveLMI(C,var)
%% plot regions
g = 9.81;
% phi = -pi/3:0.1:pi/3;
% figure(1)
% plot(phi*180/pi,g*sin(phi),phi*180/pi,g*cos(phi),'Linewidth',2);
% xlabel('\phi (deg)');
% ylabel('gravity component (m/s^2)');
% % Make patch of transparent color.
% xBox = [-10, -10, 10, 10, -10];
% yBox = [-10, 10, 10, -10, -10];
% patch(xBox, yBox,'','EdgeColor','none', 'FaceColor', 'green', 'FaceAlpha', 0.1);
% str = {'R1','R2','R3'};
% text([-30,-2,30],[0,3,0],str,'FontSize',14,'FontWeight','bold');
% legend('output1','output2')
% hold off;
% syms r p y
% Close all windows
close all

r = -pi/3:0.1:pi/3;
p = -pi/3*ones(1,21);
y = -pi/4*ones(1,21);

cr=cos(r);sr=sin(r);cp=cos(p);
sp=sin(p);cy=cos(y);sy=sin(y);

ba = [1;1;1];
f = [];
for i = 1:21
    
    Rnb=[cy(i)*cp(i) sy(i)*cp(i) -sp(i); 
    -sy(i)*cr(i)+cy(i)*sp(i)*sr(i) cy(i)*cr(i)+sy(i)*sp(i)*sr(i) cp(i)*sr(i); 
    sy(i)*sr(i)+cy(i)*sp(i)*cr(i) -cy(i)*sr(i)+sy(i)*sp(i)*cr(i) cp(i)*cr(i)];
    Rbn = Rnb';
    
    f = [f,Rbn*ba];
end

figure(1)
plot(r*180/pi,f,'Linewidth',2);
xlabel('Roll [deg]');
ylabel('bias inertial coordinates [m/s^2]');
legend('x','y','z');

r = -pi/8*ones(1,21);
p = -pi/3:0.1:pi/3;
y = -pi/4*ones(1,21);

cr=cos(r);cp=cos(p);cy=cos(y);
sp=sin(p);sr=sin(r);sy=sin(y);

f = [];
for i = 1:21
    
    Rnb=[cy(i)*cp(i) sy(i)*cp(i) -sp(i); 
    -sy(i)*cr(i)+cy(i)*sp(i)*sr(i) cy(i)*cr(i)+sy(i)*sp(i)*sr(i) cp(i)*sr(i); 
    sy(i)*sr(i)+cy(i)*sp(i)*cr(i) -cy(i)*sr(i)+sy(i)*sp(i)*cr(i) cp(i)*cr(i)];
    Rbn = Rnb';
    
    f = [f,Rbn*ba];
end

figure(2)
plot(p*180/pi,f,'Linewidth',2);
xlabel('Pitch [deg]');
ylabel('bias inertial coordinates [m/s^2]');
legend('x','y','z');

y = 0:0.1:2*pi;
r = -pi/8*ones(1,length(y));
p = pi/4*ones(1,length(y));


cr=cos(r);sr=sin(r);cp=cos(p);
sp=sin(p);cy=cos(y);sy=sin(y);

f = [];
for i = 1:1:length(y)
    
    Rnb=[cy(i)*cp(i) sy(i)*cp(i) -sp(i); 
    -sy(i)*cr(i)+cy(i)*sp(i)*sr(i) cy(i)*cr(i)+sy(i)*sp(i)*sr(i) cp(i)*sr(i); 
    sy(i)*sr(i)+cy(i)*sp(i)*cr(i) -cy(i)*sr(i)+sy(i)*sp(i)*cr(i) cp(i)*cr(i)];
    Rbn = Rnb';
    
    f = [f,Rbn*ba];
end

figure(3)
plot(y*180/pi,f,'Linewidth',2);
xlabel('Yaw [deg]');
ylabel('bias inertial coordinates [m/s^2]');
legend('x','y','z');
%% convergance rate sigma/2
sigma = 0.5;
lbound = 50;%83;%min(X);%-0.00001; %region lower bound (deg)
ubound = 120;%100;%max(X);%-0.00001; %region upper bound (deg)
offset = 2; %switching (hysteresis) offset (deg)
swbound = 10; %switching bound (deg)
tu_a = 500; %bias time constant (s)
tu_g = 500; %bias time constant (s)
%% Solve region 1
%clear all; clc; close all;
I3 = eye(3);
I23 = [eye(2) zeros(2,1)];
z3 = zeros(3,3);
z23 = zeros(2,3);
F = [eye(9) zeros(9,6);
     zeros(3,9) -1/tu_a*eye(3) zeros(3,3);
     zeros(3,12) -1/tu_g*eye(3)];
C_zupt = [z3 I3 z3 z3 z3];
C_gcu = [z23 z23 I23 z23 z23];
C = [C_zupt;C_gcu];
nout = 5;
E = [z3 I3 z3 z3 z3;
     z3 z3 I3 z3 z3;
     z3 z3 I3 z3 z3;
     z3 z3 z3 I3 z3;
     z3 z3 z3 z3 I3];

I = eye(15);
nsys = 15;
% Define the decision variables P and R
P = sdpvar(nsys,nsys);
R = sdpvar(nsys,nout);

u = lbound*ones(1,nout);
v = ubound*ones(1,nout);
u = [-1.7 -1.7 -0.3 -17 -17 -3];
v = [1.7 1.7 -0.3 17 17 -3];
L = solving(F, C, P, R, E, u, v, sigma);

%roll switch
% roll>0
u = [0 0 -0.3 0 0 -3];
v = [1.7 1.7 -0.3 17 17 -3];
L1 = solving(F, C, P, R, E, u, v, sigma);
u = [-1.7 0 -0.3 -17 0 -3];
v = [0 1.7 -0.3 0 17 -3];
L2 = solving(F, C, P, R, E, u, v, sigma);
u = [-1.7 -1.7 -0.3 -17 -17 -3];
v = [0 0 -0.3 0 0 -3];
L3 = solving(F, C, P, R, E, u, v, sigma);
u = [0 -1.7 -0.3 0 -17 -3];
v = [1.7 0 -0.3 17 0 -3];
L4 = solving(F, C, P, R, E, u, v, sigma);
L = {L1, L2, L3, L4};
end
function L = solving(F, C, P, R, E, u, v, sigma)

    I = eye(15);
    U = eye(15);
    U(4:9,4:9) = diag(u);
    V = eye(15);
    V(4:9,4:9) = diag(v);
    
    % Set LMI constraints
    lmi = [-C'*R'-R*C-0.5*E'*(V'*U + U'*V)*E+sigma*P,...
            P*F+(E'*U'+E'*V')/2; F'*P+(V*E+U*E)/2, -I;];

    % Construct LMIs for solver
    F = [P >= 0];
    F = F + [lmi <= 0];

    % Choose solver
    %ops = sdpsettings('solver','LMILAB'); % MATLAB solver
    ops = sdpsettings('solver','mosek'); % need to install
    ops.verbose = 0;
    % Solve the LMI
    diagnostics = optimize(F, [ ], ops);
    % optimize(Constraints, Objective, options)

    %Check the results of optimization
    if diagnostics.problem == 0
        disp('Feasible from Solver')
    elseif diagnostics.problem == 1
        disp('lnfeasible from Solver')
    else
        disp('Something else happened')
    end

    % Display the results
    P = double(P);
    R = double(R);
    L = P\R;
end
% %% Solve region 2
% %clear all; clc; close all;
% A = [0 0 1;0 0 0;0 0 0];
% C = [0 1 0];
% E = [1 0 0];
% M = cos(lbound*pi/180)*9.8;
% N = cos(0*pi/180)*9.8;
% 
% % Define the decision variables P and R
% P = sdpvar(3,3);
% R = sdpvar(3,1);
% 
% % Set LMI constraints
% lmi = [A'*P-C'*R'+P*A-R*C-(E'*M'*N*E+E'*N'*M*E)/2+sigma*P, (-R+(E'*M'+E'*N')/2);
%         (-R'+(N*E+M*E)/2), -1];
% % lmi = [A'*P+P*A-C'*R'-R*C-(E'*M'*N*E+E'*N'*M*E)/2+0.01*P, -R+(E'*M'+E'*N')/2;
% %         -R'+(N*E+M*E)/2, -I];
% 
% % Construct LMIs for solver
% F = [P >= 0];
% F = F + [lmi <= 0];
% 
% % Choose solver
% ops = sdpsettings('solver','LMILAB'); % MATLAB solver
% %ops = sdpsettings('solver','sedumi'); % need to install
% 
% % Solve the LMI
% diagnostics = optimize(F, [ ], ops);
% % optimize(Constraints, Objective, options)
% 
% %Check the results of optimization
% if diagnostics.problem == 0
%     disp('Feasible from Solver')
% elseif diagnostics.problem == 1
%     disp('lnfeasible from Solver')
% else
%     disp('Something else happened')
% end
%     
% % Display the results
% P = double(P);
% R = double(R);
% L2 = inv(P)*R
% %make L2 same size as L1 (compatible with A and C)
% L2 = [L2(1) 0;L2(2) 0;0 0;L2(3) 0];
% %% Solve region 3
% %clear all; clc; close all;
% A = [0 0 0 1;0 0 0 0;0 0 0 0;0 0 0 0];
% C = [0 1 0 0; 0 0 1 0];
% E = [1 0 0 0;1 0 0 0];
% M = [cos(ubound*pi/180)*9.8 0;0 -sin(ubound*pi/180)*9.8];
% N = [cos(lbound*pi/180)*9.8 0;0 -sin(lbound*pi/180)*9.8];
% 
% % Define the decision variables P and R
% P = sdpvar(nsys,nsys);
% R = sdpvar(4,2);
% 
% % Set LMI constraints
% lmi = [A'*P-C'*R'+P*A-R*C-(E'*M'*N*E+E'*N'*M*E)/2+sigma*P, (-R+(E'*M'+E'*N')/2);
%         (-R'+(N*E+M*E)/2), -I];
% % lmi = [A'*P+P*A-C'*R'-R*C-(E'*M'*N*E+E'*N'*M*E)/2+0.01*P, -R+(E'*M'+E'*N')/2;
% %         -R'+(N*E+M*E)/2, -I];
% 
% % Construct LMIs for solver
% F = [P >= 0];
% F = F + [lmi <= 0];
% 
% % Choose solver
% ops = sdpsettings('solver','LMILAB'); % MATLAB solver
% %ops = sdpsettings('solver','sedumi'); % need to install
% 
% % Solve the LMI
% diagnostics = optimize(F, [ ], ops);
% % optimize(Constraints, Objective, options)
% 
% %Check the results of optimization
% if diagnostics.problem == 0
%     disp('Feasible from Solver')
% elseif diagnostics.problem == 1
%     disp('lnfeasible from Solver')
% else
%     disp('Something else happened')
% end
%     
% % Display the results
% P = double(P);
% R = double(R);
% L3 = inv(P)*R