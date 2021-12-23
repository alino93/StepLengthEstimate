%solve LMI for bias estimation
function [L]=SolveLMI(C,X)
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

%% convergance rate sigma/2
sigma = 0.3;
lbound = min(X);%-0.00001; %region lower bound (deg)
ubound = max(X);%-0.00001; %region upper bound (deg)
offset = 2; %switching (hysteresis) offset (deg)
swbound = 10; %switching bound (deg)
tu_a = 500; %bias time constant (s)
tu_g = 500; %bias time constant (s)
%% Solve region 1
%clear all; clc; close all;
I3 = eye(3);
z3 = zeros(3,3);
F = [eye(9) zeros(9,6);
     zeros(3,9) -1/tu_a*eye(3) zeros(3,3);
     zeros(3,12) -1/tu_g*eye(3)];
C_zupt = [z3 I3 z3 z3 z3];
C_gcu = [z3 z3 I3 z3 z3];
%C = C_gcu;
E = [z3 I3 z3 z3 z3;
     z3 z3 I3 z3 z3;
     z3 z3 I3 z3 z3;
     z3 z3 z3 I3 z3;
     z3 z3 z3 z3 I3];
M = eye(15);
M(4:9,4:9) = lbound*eye(6);
N = eye(15);
N(4:9,4:9) = ubound*eye(6);
I = eye(15);
nsys = 15;
% Define the decision variables P and R
P = sdpvar(nsys,nsys);
R = sdpvar(nsys,3);

% Set LMI constraints
lmi = [-C'*R'-R*C-(E'*M'*N*E+E'*N'*M*E)/2+sigma*P, (P*F+(E'*M'+E'*N')/2);
        (F'*P+(N*E+M*E)/2), -I];
% lmi = [A'*P+P*A-C'*R'-R*C-(E'*M'*N*E+E'*N'*M*E)/2+0.01*P, -R+(E'*M'+E'*N')/2;
%         -R'+(N*E+M*E)/2, -I];

% Construct LMIs for solver
F = [P >= 0];
F = F + [lmi <= 0];

% Choose solver
%ops = sdpsettings('solver','LMILAB'); % MATLAB solver
ops = sdpsettings('solver','sedumi'); % need to install
ops.verbose = 0;
% Solve the LMI
diagnostics = optimize(F, [ ], ops);
% optimize(Constraints, Objective, options)

% %Check the results of optimization
% if diagnostics.problem == 0
%     disp('Feasible from Solver')
% elseif diagnostics.problem == 1
%     disp('lnfeasible from Solver')
% else
%     disp('Something else happened')
% end
    
% Display the results
P = double(P);
R = double(R);
L = inv(P)*R;

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