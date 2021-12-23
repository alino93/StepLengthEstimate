%solve LMI for bias estimation
function [L_zupt, L_nz]=SolveLMI(k1, k2)
%% plot regions
g = 9.81;

% close all
% 
% p = -pi/3:0.1:pi/3;
% 
% cp=cos(p);
% sp=sin(p);
% 
% f=[sp;cp];
% 
% figure(1)
% plot(p*180/pi,f,'Linewidth',2);
% xlabel('Pitch [deg]');
% ylabel('Nonlinear outputs [m/s^2]');
% legend('f1','f2');

%% convergance rate sigma/2
sigma = 0.1;
tu_a = 500; %bias time constant (s)
tu_g = 500; %bias time constant (s)
%% Solve ZUPT LMI
%clear all; clc; close all;
I3 = eye(3);
z3 = zeros(3,3);
% k1 = 1;
% k2 = 1;
F = [k2 k1 0 0 0;
    -k1 k2 0 0 0;
     0 0 0 0 1;
     0 0 0 0 0;
     0 0 0 0 0;
     0 0 0 0 0;];
F = [k2 k1 1 2 0;
    -k1 k2 -2 1 0;
     0 0 0 0 0.1;
     0 0 0 0 0;
     0 0 0 0 0;
     0 0 0 0 0;];

% F = [k2 k1 0 0 0;
%     -k1 k2 0 0 0;
%      0 0 0 0 1;
%      0 0 -1/tu_a 0 0;
%      0 0 0 -1/tu_a 0;
%      0 0 0 0 -1/tu_g;];

C = [I3 z3];

E = [0 0 1 0 0 0;
     0 0 1 0 0 0;
     0 0 0 1 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1];

nsys = 6;
nout = 3;
% Define the decision variables P and R
P = sdpvar(nsys,nsys);
R = sdpvar(nsys,nout);

u = [0.5 0.1 1 1 1];
v = [1 1 1 1 1];
L1 = solvingZ(F, C, P, R, E, u, v, sigma)

u = [0.9 0 1 1 1];
v = [1 0 1 1 1];
L2 = solvingZ(F, C, P, R, E, u, v, sigma);

u = [0.5 -1 1 1 1];
v = [1 -0.1 1 1 1];
L3 = solvingZ(F, C, P, R, E, u, v, sigma);

L_zupt = {L1, L2, L3};

%% Solve non-ZUPT LMI
A = [0 0 0 1;0 0 0 0;0 0 0 0;0 0 0 0];
C = [0 1 0 0; 0 0 1 0];
E = [1 0 0 0;1 0 0 0];
nsys = 4;
nout = 2;
% Define the decision variables P and R
P = sdpvar(nsys,nsys);
R = sdpvar(nsys,nout);


%m = [cos(-v*pi/180) -sin(-u*pi/180)]*9.8;
%n = [cos(-u*pi/180) -sin(-v*pi/180)]*9.8;
m = [0.5 0.1];
n = [1 1];
L1 = solvingNZ(A, C, P, R, E, m, n, sigma);
m = [0.9 0];
n = [1 0];
L2 = solvingNZ(A, C, P, R, E, m, n, sigma);
m = [0.5 -1];
n = [1 -0.1];
L3 = solvingNZ(A, C, P, R, E, m, n, sigma);
L_nz = {L1, L2, L3};
end
function L = solvingZ(F, C, P, R, E, u, v, sigma)

    I = eye(5);
    U = diag(u);
    V = diag(v);
    
    % Set LMI constraints
    lmi = [-C'*R'-R*C-0.5*E'*(V'*U + U'*V)*E+sigma*P, P*F+(E'*U'+E'*V')/2; 
            F'*P+(V*E+U*E)/2,                         -I;];

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

%     %Check the results of optimization
%     if diagnostics.problem == 0
%         disp('Feasible from Solver')
%     elseif diagnostics.problem == 1
%         disp('lnfeasible from Solver')
%     else
%         disp('Something else happened')
%     end

    % Display the results
    P = double(P);
    R = double(R);
    L = P\R;
end
function L = solvingNZ(A, C, P, R, E, m, n, sigma)

    M = diag(m);
    N = diag(n);
    I = eye(2);
    

    % Set LMI constraints
    lmi = [A'*P-C'*R'+P*A-R*C-(E'*M'*N*E+E'*N'*M*E)/2+sigma*P, -R+(E'*M'+E'*N')/2;
          -R'+(N*E+M*E)/2, -I];

    % Construct LMIs for solver
    F = [P >= 0];
    F = F + [lmi <= 0];

    % Choose solver
    ops = sdpsettings('solver','mosek');
    ops.verbose = 0;
    %ops = sdpsettings('solver','sedumi');

    % Solve the LMI
    diagnostics = optimize(F, [ ], ops);
    % optimize(Constraints, Objective, options)

%     %Check the results of optimization
%     if diagnostics.problem == 0
%         disp('Feasible from Solver')
%     elseif diagnostics.problem == 1
%         disp('lnfeasible from Solver')
%     else
%         disp('Something else happened')
%     end

    % Display the results
    P = double(P);
    R = double(R);
    L1 = inv(P)*R;
    L = P\R;
end
