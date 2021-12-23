%solve LMI for bias estimation
%% plot regions
% g = 9.81;
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

%% convergance rate sigma/2
sigma = 0.3;
lbound = 10; %region lower bound (deg)
ubound = 20; %region upper bound (deg)
offset = 2; %switching (hysteresis) offset (deg)
swbound = 10; %switching bound (deg)
%% Solve region 1
%clear all; clc; close all;
A = [0 0 0 1;0 0 0 0;0 0 0 0;0 0 0 0];
C = [0 1 0 0; 0 0 1 0];
E = [1 0 0 0;1 0 0 0];
M = [cos(-ubound*pi/180)*9.8 0;0 -sin(-lbound*pi/180)*9.8];
N = [cos(-lbound*pi/180)*9.8 0;0 -sin(-ubound*pi/180)*9.8];
I = eye(2);
nsys = 4;
% Define the decision variables P and R
P = sdpvar(nsys,nsys);
R = sdpvar(4,2);

% Set LMI constraints
lmi = [A'*P-C'*R'+P*A-R*C-(E'*M'*N*E+E'*N'*M*E)/2+sigma*P, (-R+(E'*M'+E'*N')/2);
        (-R'+(N*E+M*E)/2), -I];

% Construct LMIs for solver
F = [P >= 0];
F = F + [lmi <= 0];

% Choose solver
ops = sdpsettings('solver','LMILAB'); % MATLAB solver
ops.verbose = 0;

% Solve the LMI
diagnostics = optimize(F, [ ], ops);
    
% Display the results
P = double(P);
R = double(R);
L1 = inv(P)*R;

%% Solve region 2
%clear all; clc; close all;
A = [0 0 1;0 0 0;0 0 0];
C = [0 1 0];
E = [1 0 0];
M = cos(lbound*pi/180)*9.8;
N = cos(0*pi/180)*9.8;

% Define the decision variables P and R
P = sdpvar(3,3);
R = sdpvar(3,1);

% Set LMI constraints
lmi = [A'*P-C'*R'+P*A-R*C-(E'*M'*N*E+E'*N'*M*E)/2+sigma*P, (-R+(E'*M'+E'*N')/2);
        (-R'+(N*E+M*E)/2), -1];

% Construct LMIs for solver
F = [P >= 0];
F = F + [lmi <= 0];

% Choose solver
ops = sdpsettings('solver','LMILAB'); % MATLAB solver
ops.verbose = 0;

% Solve the LMI
diagnostics = optimize(F, [ ], ops);

% Display the results
P = double(P);
R = double(R);
L2 = inv(P)*R;
%make L2 same size as L1 (compatible with A and C)
L2 = [L2(1) 0;L2(2) 0;0 0;L2(3) 0];
%% Solve region 3
%clear all; clc; close all;
A = [0 0 0 1;0 0 0 0;0 0 0 0;0 0 0 0];
C = [0 1 0 0; 0 0 1 0];
E = [1 0 0 0;1 0 0 0];
M = [cos(ubound*pi/180)*9.8 0;0 -sin(ubound*pi/180)*9.8];
N = [cos(lbound*pi/180)*9.8 0;0 -sin(lbound*pi/180)*9.8];

% Define the decision variables P and R
P = sdpvar(nsys,nsys);
R = sdpvar(4,2);

% Set LMI constraints
lmi = [A'*P-C'*R'+P*A-R*C-(E'*M'*N*E+E'*N'*M*E)/2+sigma*P, (-R+(E'*M'+E'*N')/2);
        (-R'+(N*E+M*E)/2), -I];

% Construct LMIs for solver
F = [P >= 0];
F = F + [lmi <= 0];

% Choose solver
ops = sdpsettings('solver','LMILAB'); % MATLAB solver
% ops = sdpsettings('solver', 'sedumi', 'sedumi.eps', 1e-8, ...
%                 'sedumi.cg.qprec', 1, 'sedumi.cg.maxiter', 49, ...
%                 'sedumi.stepdif', 2);
ops.verbose = 0;
%ops = sdpsettings('solver','sedumi'); % need to install

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
    
% Display the eigenvalues
P = double(P);
R = double(R);
LMI = double(lmi);
L3 = inv(P)*R;
eig(LMI)
eig(P)