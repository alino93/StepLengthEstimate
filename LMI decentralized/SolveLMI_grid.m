%solve LMI for bias estimation
function [L]=SolveLMI_grid(C,var)
% I3 = eye(3);
% z3 = zeros(3,3);
% C_zupt = [z3 I3 z3 z3 z3];
% C_gcu = [z3 z3 I3 z3 z3];
% C = C_zupt;
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
%% grid
% x = -30:5:30;
% 
% x = x * pi/180;
% x1 = 9.81*cos(x);
% x2 = -9.81*sin(x);

% roll pitch yaw angles in radians
syms r p y

cr=cos(r);
sr=sin(r);

cp=cos(p);
sp=sin(p);

cy=cos(y);
sy=sin(y);

Rnb=[cy*cp sy*cp -sp; 
    -sy*cr+cy*sp*sr cy*cr+sy*sp*sr cp*sr; 
    sy*sr+cy*sp*cr -cy*sr+sy*sp*cr cp*cr];
%% convergance rate sigma/2
alpha = 0.5;
gamma = 0.002;
tu_a = 500; %bias time constant (s)
tu_g = 500; %bias time constant (s)
%% Solve for several points on grid
%clear all; clc; close all;

syms rx ry rz vx vy vz bax bay baz bgx bgy bgz

Rbn = Rnb';
v = [vx;vy;vz];
ba = [bax;bay;baz];
bg = [bgx;bgy;bgz];

f = [v;Rbn*ba;Rbn*bg;-ba/tu_a;-bg/tu_g];
x = [rx;ry;rz;v;r;p;y;ba;bg];
A = jacobian(f, x);

L1 = solveLMI(A, C, gamma, alpha, 0,var);
L2 = solveLMI(A, C, gamma, alpha, pi/2,var);
L3 = solveLMI(A, C, gamma, alpha, pi,var);
L4 = solveLMI(A, C, gamma, alpha, 3*pi/2,var);

% [L12,gamma1] = solveBisection(A, C, alpha, 0);
% [L22,gamma2] = solveBisection(A, C, alpha, pi/2);
% [L32,gamma3] = solveBisection(A, C, alpha, pi);
% [L42,gamma4] = solveBisection(A, C, alpha, 3*pi/2);

L = {L1, L2, L3, L4};

% %Check the results of optimization
% if diagnostics.problem == 0
%     disp('Feasible from Solver')
% elseif diagnostics.problem == 1
%     disp('lnfeasible from Solver')
% else
%     disp('Something else happened')
% end
%     
% % Display the eigenvalues
% P = double(P);
% R = double(R);
% LMI = double(lmi);
% eig(LMI)
% eig(P)
end
function L = solveLMI(A, C, gamma, sigma, yaw, var)
    I3 = eye(3);
    I = eye(15);
    nsys = 15;

    % Define the decision variables P and R
    P = sdpvar(nsys,nsys);
    R = sdpvar(nsys,3);
    Bd = [P R; R' gamma*I3]; %gamma * I bound for L 

    % Construct LMIs for solver
    F = [P >= 0];
    F = F + [Bd >=0];

    %jacobian value on grid
    bax=0;
    bay=-0;
    baz=-0;
    bgx=0.0000*pi/180;
    bgy=-0.000*pi/180;
    bgz=-0.1*pi/180;
    
    for bgx=-0.1:0.1:+0.1
        for bgz=-0.05:0.01:0.05
            for r=0.11-0.3:0.3:0.11+0.3
                for p=0.1529-0.3:0.3:0.1529+0.3
                    for y=yaw-pi/3:0.5:yaw+pi/3
                        for bax=-0.3:0.3:+0.3
                            for bay=-0.3:0.3:+0.3
                                for baz=-0.3:0.3:+0.3
                                    for bgy=-0.1:0.1:+0.1
                                
                                        % Set LMI constraints
                                        A_num = eval(A);
                                        lmi = A_num'*P+P*A_num-C'*R'-R*C+sigma*P;
                                        % Construct LMIs for solver
                                        F = F + [lmi <= 0];
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
    end


    % Choose solver
    %ops = sdpsettings('solver','LMILAB'); % MATLAB solver
    %ops = sdpsettings('solver','sedumi'); % need to install
    % 
    ops = sdpsettings('solver', 'sedumi', 'sedumi.eps', 1e-8, ...
                     'sedumi.cg.qprec', 1, 'sedumi.cg.maxiter', 49, ...
                     'sedumi.stepdif', 2, ...
                     'showprogress' , 1);
    ops.verbose = 0;
    % Solve the LMI
    diagnostics = optimize(F, [ ], ops);

    % Display the results
    P = double(P);
    R = double(R);
    L = inv(P)*R;
end
function [L,gamma] = solveBisection(A, C, sigma, yaw)
    I3 = eye(3);
    I = eye(15);
    nsys = 15;
    
    % Define the decision variables P and R
    P = sdpvar(nsys,nsys);
    R = sdpvar(nsys,3);
    sdpvar t
    Bd = [P R; R' t*I3]; %gamma * I bound for L 

    % Construct LMIs for solver
    F = [P >= I];
    F = F + [Bd >=0];

    %jacobian value on grid
    for bax=0.3
        for bgx=0.1   
            for r=-0.2:0.1:+0.2
                for p=-0.4:0.1:+0.4
                    for y=yaw-pi/3:0.3:yaw+pi/3
                        % Set LMI constraints
                        A_num = eval(A);
                        lmi = A_num'*P+P*A_num-C'*R'-R*C+sigma*P;
                        % Construct LMIs for solver
                        F = F + [lmi <= 0];
                    end
                end
            end
        end
    end
    % Objective
    Obj = t;
    ops = sdpsettings('solver','bisection','bisection.solver','sedumi');
    diagnostics =  optimize(F, Obj, ops);
    % Display the results
    P = double(P);
    R = double(R);
    L = inv(P)*R;
    gamma = double(t);
end