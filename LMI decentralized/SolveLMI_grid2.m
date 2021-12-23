%solve LMI for bias estimation
function [L]=SolveLMI_grid2(C,var)
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
alpha = 0.05;
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

L1 = solveLMI(C, gamma, alpha, 0,var);
L2 = solveLMI(C, gamma, alpha, pi/2,var);
L3 = solveLMI(C, gamma, alpha, pi,var);
L4 = solveLMI(C, gamma, alpha, 3*pi/2,var);

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
function L = solveLMI(C, gamma, sigma, yaw, var)
    I3 = eye(3);
    z3 = zeros(3);
    I = eye(15);
    nsys = 15;
    tu_a = 500; %bias time constant (s)
    tu_g = 500; %bias time constant (s)
    % Define the decision variables P and R
    P = sdpvar(nsys,nsys);
    R = sdpvar(nsys,3);
    Bd = [P R; R' gamma*I3]; %gamma * I bound for L 

    % Construct LMIs for solver
    F = [P >= 0];
    F = F + [Bd >=0];

    %jacobian value on grid
    bax=0.15;
    bay=0.15;
    baz=0.15;
    bgx=0.1;
    bgy=0.1;
    bgz=0.1;
    for bgx=-0.5:0.5:0.5
        %for bgz=
            for r=0.11-0.3:0.3:0.11+0.3
                for p=0.1529-0.3:0.3:0.1529+0.3
                    for y=yaw-pi/3:0.5:yaw+pi/3
                        for bax=-0.15:0.15:+0.15
                            %for bay=-0.3:0.3:+0.3
                                %for baz=-0.3:0.3:+0.3
                                    %for bgy=-0.1:0.1:+0.1
                                        
%                                         q1 = sin(r)*sin(y) + cos(r)*cos(y)*sin(p);
%                                         q2 = cos(r)*cos(y) + sin(p)*sin(r)*sin(y);
%                                         q3 = bgz*cos(p)*cos(r) - bgx*sin(p) + bgy*cos(p)*sin(r);
%                                         q4 = baz*cos(p)*cos(r) - bax*sin(p) + bay*cos(p)*sin(r);
%                                         q5 = cos(y)*sin(p)*sin(r);
%                                         q6 = cos(r)*sin(p)*sin(y);
%                                         q7 = cos(p)*cos(y);
%                                         q8 = cos(p)*cos(r);
%                                         q9 = cos(y)*sin(r);
%                                         q10 = cos(r)*sin(y);
%                                         q11 = cos(p)*sin(y);
%                                         q12 = cos(p)*sin(r);
%                                         
%                                         
%                                         A1 = [bay*q1 + baz*(q10 - q5),            cos(y)*q4,   baz *(q9 - q6) - bay*q2 - bax*cos(p)*sin(y)
%                                               -bay*(q9 - q6) - baz*q2,            sin(y)*q4,   baz*q1 - bay*(q10 - q5) + bax*cos(p)*cos(y)
%                                               -cos(p)*(baz*sin(r) - bay*cos(r)), -bax*cos(p) - baz*cos(r)*sin(p) - bay*sin(p)*sin(r),   0];
%                                         A2 = [    q7,   q5 - q10,         q1
%                                                  q11,         q2,    q6 - q9
%                                              -sin(p),        q12,         q8];
%                                         A3 = [bgy*q1 + bgz*(q10 - q5),           cos(y)*q3,                       bgz*(q9 - q6) - bgy*q2 - bgx*cos(p)*sin(y)
%                                             - bgy*(q9 - q6) - bgz*q2,            sin(y)*q3,                      bgz*q1 - bgy*(q10 - q5) + bgx*cos(p)*cos(y)
%                                             -cos(p)*(bgz*sin(r) - bgy*cos(r)), - bgx*cos(p) - bgz*cos(r)*sin(p) - bgy*sin(p)*sin(r),                      0];
%                                         A4 = [q7,       q5 - q10,    q1
%                                              q11,       q2,          q6 - q9
%                                             -sin(p),    q12,         q8];
%                                         A_num = [z3, I3, z3, z3, z3;
%                                                  z3, z3, A1, A2, z3;
%                                                  z3, z3, A3, z3, A4;
%                                                  z3, z3, z3, -I3/tu_a, z3;
%                                                  z3, z3, z3, z3, -I3/tu_g];
                                             
                                        A_num = [ 0, 0, 0, 1, 0, 0,                                                                                                                                            0,                                                                                        0,                                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                                   0
                                         0, 0, 0, 0, 1, 0,                                                                                                                                                     0,                                                                                        0,                                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                                   0
                                         0, 0, 0, 0, 0, 1,                                                                                                                                                     0,                                                                                        0,                                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                                   0
                                         0, 0, 0, 0, 0, 0,   bax*(sin(conj(r))*sin(conj(y)) + cos(conj(r))*cos(conj(y))*sin(conj(p))) + bax*(cos(conj(r))*sin(conj(y)) - cos(conj(y))*sin(conj(p))*sin(conj(r))),  bax*cos(conj(y))*(cos(conj(p))*cos(conj(r)) - sin(conj(p)) + cos(conj(p))*sin(conj(r))), bax*(cos(conj(y))*sin(conj(r)) - cos(conj(r))*sin(conj(p))*sin(conj(y))) - bax*(cos(conj(r))*cos(conj(y)) + sin(conj(p))*sin(conj(r))*sin(conj(y))) - bax*cos(conj(p))*sin(conj(y)), cos(conj(p))*cos(conj(y)) - cos(conj(r))*sin(conj(y)) + sin(conj(r))*sin(conj(y)) + cos(conj(r))*cos(conj(y))*sin(conj(p)) + cos(conj(y))*sin(conj(p))*sin(conj(r)), cos(conj(p))*cos(conj(y)) - cos(conj(r))*sin(conj(y)) + sin(conj(r))*sin(conj(y)) + cos(conj(r))*cos(conj(y))*sin(conj(p)) + cos(conj(y))*sin(conj(p))*sin(conj(r)), cos(conj(p))*cos(conj(y)) - cos(conj(r))*sin(conj(y)) + sin(conj(r))*sin(conj(y)) + cos(conj(r))*cos(conj(y))*sin(conj(p)) + cos(conj(y))*sin(conj(p))*sin(conj(r)),                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                                   0
                                         0, 0, 0, 0, 0, 0, - bax*(cos(conj(r))*cos(conj(y)) + sin(conj(p))*sin(conj(r))*sin(conj(y))) - bax*(cos(conj(y))*sin(conj(r)) - cos(conj(r))*sin(conj(p))*sin(conj(y))),  bax*sin(conj(y))*(cos(conj(p))*cos(conj(r)) - sin(conj(p)) + cos(conj(p))*sin(conj(r))), bax*(sin(conj(r))*sin(conj(y)) + cos(conj(r))*cos(conj(y))*sin(conj(p))) - bax*(cos(conj(r))*sin(conj(y)) - cos(conj(y))*sin(conj(p))*sin(conj(r))) + bax*cos(conj(p))*cos(conj(y)), cos(conj(r))*cos(conj(y)) + cos(conj(p))*sin(conj(y)) - cos(conj(y))*sin(conj(r)) + cos(conj(r))*sin(conj(p))*sin(conj(y)) + sin(conj(p))*sin(conj(r))*sin(conj(y)), cos(conj(r))*cos(conj(y)) + cos(conj(p))*sin(conj(y)) - cos(conj(y))*sin(conj(r)) + cos(conj(r))*sin(conj(p))*sin(conj(y)) + sin(conj(p))*sin(conj(r))*sin(conj(y)), cos(conj(r))*cos(conj(y)) + cos(conj(p))*sin(conj(y)) - cos(conj(y))*sin(conj(r)) + cos(conj(r))*sin(conj(p))*sin(conj(y)) + sin(conj(p))*sin(conj(r))*sin(conj(y)),                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                                   0
                                         0, 0, 0, 0, 0, 0,                                                                                                          2^(1/2)*bax*cos(conj(p))*cos(pi/4 + conj(r)),       - bax*cos(conj(p)) - bax*cos(conj(r))*sin(conj(p)) - bax*sin(conj(p))*sin(conj(r)),                                                                                                                                                                                   0,                                                                                                cos(conj(p))*cos(conj(r)) - sin(conj(p)) + cos(conj(p))*sin(conj(r)),                                                                                                cos(conj(p))*cos(conj(r)) - sin(conj(p)) + cos(conj(p))*sin(conj(r)),                                                                                                cos(conj(p))*cos(conj(r)) - sin(conj(p)) + cos(conj(p))*sin(conj(r)),                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                                   0
                                         0, 0, 0, 0, 0, 0, - bgx*(sin(conj(r))*sin(conj(y)) + cos(conj(r))*cos(conj(y))*sin(conj(p))) - bgx*(cos(conj(r))*sin(conj(y)) - cos(conj(y))*sin(conj(p))*sin(conj(r))), -bgx*cos(conj(y))*(cos(conj(p))*cos(conj(r)) - sin(conj(p)) + cos(conj(p))*sin(conj(r))), bgx*(cos(conj(r))*cos(conj(y)) + sin(conj(p))*sin(conj(r))*sin(conj(y))) - bgx*(cos(conj(y))*sin(conj(r)) - cos(conj(r))*sin(conj(p))*sin(conj(y))) + bgx*cos(conj(p))*sin(conj(y)),                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                                   0, cos(conj(r))*sin(conj(y)) - cos(conj(p))*cos(conj(y)) - sin(conj(r))*sin(conj(y)) - cos(conj(r))*cos(conj(y))*sin(conj(p)) - cos(conj(y))*sin(conj(p))*sin(conj(r)), cos(conj(r))*sin(conj(y)) - cos(conj(p))*cos(conj(y)) - sin(conj(r))*sin(conj(y)) - cos(conj(r))*cos(conj(y))*sin(conj(p)) - cos(conj(y))*sin(conj(p))*sin(conj(r)), cos(conj(r))*sin(conj(y)) - cos(conj(p))*cos(conj(y)) - sin(conj(r))*sin(conj(y)) - cos(conj(r))*cos(conj(y))*sin(conj(p)) - cos(conj(y))*sin(conj(p))*sin(conj(r))
                                         0, 0, 0, 0, 0, 0,   bgx*(cos(conj(r))*cos(conj(y)) + sin(conj(p))*sin(conj(r))*sin(conj(y))) + bgx*(cos(conj(y))*sin(conj(r)) - cos(conj(r))*sin(conj(p))*sin(conj(y))), -bgx*sin(conj(y))*(cos(conj(p))*cos(conj(r)) - sin(conj(p)) + cos(conj(p))*sin(conj(r))), bgx*(cos(conj(r))*sin(conj(y)) - cos(conj(y))*sin(conj(p))*sin(conj(r))) - bgx*(sin(conj(r))*sin(conj(y)) + cos(conj(r))*cos(conj(y))*sin(conj(p))) - bgx*cos(conj(p))*cos(conj(y)),                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                                   0, cos(conj(y))*sin(conj(r)) - cos(conj(p))*sin(conj(y)) - cos(conj(r))*cos(conj(y)) - cos(conj(r))*sin(conj(p))*sin(conj(y)) - sin(conj(p))*sin(conj(r))*sin(conj(y)), cos(conj(y))*sin(conj(r)) - cos(conj(p))*sin(conj(y)) - cos(conj(r))*cos(conj(y)) - cos(conj(r))*sin(conj(p))*sin(conj(y)) - sin(conj(p))*sin(conj(r))*sin(conj(y)), cos(conj(y))*sin(conj(r)) - cos(conj(p))*sin(conj(y)) - cos(conj(r))*cos(conj(y)) - cos(conj(r))*sin(conj(p))*sin(conj(y)) - sin(conj(p))*sin(conj(r))*sin(conj(y))
                                         0, 0, 0, 0, 0, 0,                                                                                                         -2^(1/2)*bgx*cos(conj(p))*cos(pi/4 + conj(r)),         bgx*cos(conj(p)) + bgx*cos(conj(r))*sin(conj(p)) + bgx*sin(conj(p))*sin(conj(r)),                                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                sin(conj(p)) - cos(conj(p))*cos(conj(r)) - cos(conj(p))*sin(conj(r)),                                                                                                sin(conj(p)) - cos(conj(p))*cos(conj(r)) - cos(conj(p))*sin(conj(r)),                                                                                                sin(conj(p)) - cos(conj(p))*cos(conj(r)) - cos(conj(p))*sin(conj(r))
                                         0, 0, 0, 0, 0, 0,                                                                                                                                                     0,                                                                                        0,                                                                                                                                                                                   0,                                                                                                                                                              -1/500,                                                                                                                                                              -1/500,                                                                                                                                                              -1/500,                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                                   0
                                         0, 0, 0, 0, 0, 0,                                                                                                                                                     0,                                                                                        0,                                                                                                                                                                                   0,                                                                                                                                                              -1/500,                                                                                                                                                              -1/500,                                                                                                                                                              -1/500,                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                                   0
                                         0, 0, 0, 0, 0, 0,                                                                                                                                                     0,                                                                                        0,                                                                                                                                                                                   0,                                                                                                                                                              -1/500,                                                                                                                                                              -1/500,                                                                                                                                                              -1/500,                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                                   0
                                         0, 0, 0, 0, 0, 0,                                                                                                                                                     0,                                                                                        0,                                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                              -1/500,                                                                                                                                                              -1/500,                                                                                                                                                              -1/500
                                         0, 0, 0, 0, 0, 0,                                                                                                                                                     0,                                                                                        0,                                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                              -1/500,                                                                                                                                                              -1/500,                                                                                                                                                              -1/500
                                         0, 0, 0, 0, 0, 0,                                                                                                                                                     0,                                                                                        0,                                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                                                                              -1/500,                                                                                                                                                              -1/500,                                                                                                                                                              -1/500];

                                        % Set LMI constraints
%                                         A_num = eval(A);
                                        lmi = A_num'*P+P*A_num-C'*R'-R*C+sigma*P;
                                        % Construct LMIs for solver
                                        F = F + [lmi <= 0];

                                    %end
                                %end
                            %end
                        end
                    end
                end
            end
        %end
    end


    % Choose solver
    %ops = sdpsettings('solver','LMILAB'); % MATLAB solver
    ops = sdpsettings('solver','mosek'); % need to install
    % 
    %ops = sdpsettings('solver', 'LMILAB');
    ops.verbose = 0;
    % Solve the LMI
    diagnostics = optimize(F, [ ], ops);

    % Display the results
    P = double(P);
    R = double(R);
    L = P\R
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