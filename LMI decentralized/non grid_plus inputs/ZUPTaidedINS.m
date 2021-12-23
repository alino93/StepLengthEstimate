%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%> @file ZUPTaidedINS.m
%>
%> @brief This file contains all the functions needed to implement a zero-
%> velocity aided inertial navigation system. 
%>
%> @details This file contains all the functions needed to implement a 
%> zero-velocity aided inertial navigation system, given a set of IMU data
%> and a vector that indicates when the system has zero-velocity. 
%>
%> @authors Isaac Skog, John-Olof Nilsson
%> @copyright Copyright (c) 2011 OpenShoe, ISC License (open source)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%% MAINFUNCTION


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  funtion [x_h cov]=ZUPTaidedINS(u,zupt)
%
%> @brief Function that runs the zero-velocity aided INS Kalman filter 
%> algorithm. 
%>
%> @details Function that runs the zero-velocity aided INS Kalman filter 
%> algorithm. All settings for the filter is done in setting.m. 
%>
%> @param[out]  x_h     Matrix with the estimated navigation states. Each row holds the [position, velocity, attitude, (biases, if turned on),(scale factors, if turned on)] for time instant k, k=1,...N.        
%> @param[out]  cov     Matrix with the diagonal elements of the state covariance matrices.
%> @param[in]   u       The IMU data vector.     
%> @param[in]   zupt    Vector with the decisions of the zero-velocity.
%>
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [x_h1, x_h2, cov1, cov2]=ZUPTaidedINS(u1,u2,zupt1,zupt2,var)

%%%%%%%%%%% RCU parameters

gamma_rcu = 1.05; % Range constraint
sig_rcu = 0.00072; % Stdv of RCU 
sig_gcu = 0.009; % Stdv of GCU 
z3 = zeros(3,3);
I3 = eye(3);
I2 = [eye(2) zeros(2,1);zeros(1,3)];
I3z = [1 0 0;0 1 0];
H_rcu = [I3 z3 z3 z3 z3];
R_rcu = I3*sig_rcu^2;
H_gcu = [zeros(2,3) I3z zeros(2,3) zeros(2,3) zeros(2,3)];
R_gcu = eye(2)*sig_gcu^2;
C_zupt = [z3 I3 z3 z3 z3];
C_gcu = [z3 z3 I3 z3 z3];
C = [C_zupt;C_gcu];
t_zupt = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%      Initialize the data fusion          %%       
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Get  the length of the IMU data vector
N=length(u1);

% Initialize the filter state covariance matrix P, the processes noise
% covariance matrix Q, the pseudo measurement noise covariance R, and the
% observation matrix H.
[P1 Q1 R1 H1]=init_filter;          % Subfunction located further down in the file.
[P2 Q2 R2 H2]=init_filter; 
% Allocate vecors
[x_h1 cov1 Id1]=init_vec(N,P1);     % Subfunction located further down in the file.
[x_h2 cov2 Id2]=init_vec(N,P2);     % Subfunction located further down in the file.

% Initialize the navigation state vector x_h, and the quaternion vector
% quat.
[x_h1(1:9,1) quat1]=init_Nav_eq(u1);  % Subfunction located further down in the file.
[x_h2(1:9,1) quat2]=init_Nav_eq(u2);  % Subfunction located further down in the file.

% sampling time
Ts=0.01;

% Calculate the filter gain
%L_zupts = SolveLMI_grid(C,var);
L_zupts = SolveLMI(C,var);
%L_gcu = SolveLMI_grid(C_gcu,x_h1(:,1));

% initial switch gain and swtich state
sw_st = zeros(1,N); %0:N 1:E 2:S 3:W
sw_std = 0;
yaw = zeros(1,N);
L_zupt = L_zupts{1};
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%        Run the filter algorithm          %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k=2:N
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %           Time  Update         %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Compensate the IMU measurements with the current estimates of the 
    % sensor errors  
    u_h1=comp_imu_errors(u1(:,k),x_h1(:,k-1)); % Subfunction located further down in the file.
    u_h2=comp_imu_errors(u2(:,k),x_h2(:,k-1)); % Subfunction located further down in the file.

    
    % Update the navigation equations.   
    [x_h1(:,k) quat1]=Navigation_equations(x_h1(:,k-1),u_h1,quat1); % Subfunction located further down in the file.
    [x_h2(:,k) quat2]=Navigation_equations(x_h2(:,k-1),u_h2,quat2); % Subfunction located further down in the file.

    
    % Update state transition matrix
    [F1 G1]=state_matrix(quat1,u_h1); % Subfunction located further down in the file.
    [F2 G2]=state_matrix(quat2,u_h2); % Subfunction located further down in the file.

    
    % Update the filter state covariance matrix P.
    P1=F1*P1*F1'+G1*Q1*G1';
    P2=F2*P2*F2'+G2*Q2*G2';
    
    % Make sure the filter state covariance matrix is symmetric. 
    P1=(P1+P1')/2;
    P2=(P2+P2')/2;
    
    % Store the diagonal of the state covariance matrix P.
    cov1(:,k)=diag(P1);
    cov2(:,k)=diag(P2);
    
    % switch gains conditions
    % x_h1(9,k-1) is by default -pi to pi
    % change it to 0 to 2pi
    % except for region 1 (-pi/3<yaw<pi/3)
    attitude = (real(x_h1(7:9,k))')*180/pi;
    r = attitude(1);
    p = attitude(2);
    y = attitude(3);
    yaw(k) = rem(x_h1(9,k)+2*pi,2*pi);
%     if sw_std == 0
%         yaw(k) = x_h1(9,k);
%     end
%     if sw_std*pi/2+pi/4+pi/3 < yaw(k)
%         sw_std = rem(sw_std + 1, 4);
%         L_zupt = L_zupts{sw_st + 1};
%     elseif sw_std*pi/2+pi/4-pi/3 > yaw(k)
%         sw_std = rem(sw_std + 4 - 1, 4);
%         L_zupt = L_zupts{sw_st + 1};
%     end
%     sw_st(k) = sw_std;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %      Zero-velocity update      %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Check if a zero velocity update should be done. If so, do the
    % following
    if zupt1(k)==true;
        
%         if k-t_zupt>1000
%             % Calculate the filter gain
%             L_zupt = SolveLMI_grid(C_zupt,x_h1(:,k));
%             %K1=(P1*H1')/(H1*P1*H1'+R1);
%             t_zupt = k;
%         end
        
        % Calculate the prediction error. Since the detector hypothesis 
        % is that the platform has zero velocity, the prediction error is 
        % equal to zero minu the estimated velocity.    
        %z=x_h1(4:6,k);   
        
        % Estimation of the perturbations in the estimated navigation
        % states
        f_u=mean(u_h1(1));
        f_v=mean(u_h1(2));
        f_w=mean(u_h1(3));
        ge = sqrt(f_u^2 + f_v^2 + f_w^2);
        roll=atan2(-f_v,-f_w);
        pitch=atan2(f_u,sqrt(f_v^2+f_w^2));
        y = [zeros(3,1);roll;pitch;];
        I23 = [eye(2) zeros(2,1)];
        z23 = zeros(2,3);
        C_zupt = [z3 I3 z3 z3 z3];
        C_gcu = [z23 z23 I23 z23 z23];
        C = [C_zupt;C_gcu];
        dx=L_zupt*(y-C*x_h1(:,k))*Ts;
        %dx(3) = 0;
        %dx(6) = 0;
        dx(7) = 0;
        dx(8) = 0;
        dx(9) = 0;
        %dx(10) = 0;
        %dx(11) = 0;
        %dx(12) = 0;
        %dx(13) = 0;
        %dx(14) = 0;
        %dx(15) = 0;

        %dx=-K1*z;
        
        % Correct the navigation state using the estimated perturbations. 
        % (Subfunction located further down in the file.)
        [x_h1(:,k) quat1]=comp_internal_states(x_h1(:,k),dx,quat1);    
    
        
        % Update the filter state covariance matrix P.
        %P1=(Id1-L_zupt*H1)*P1;
        %P1=(Id1-K1*H1)*P1;
        
        % Make sure the filter state covariance matrix is symmetric. 
        %P1=(P1+P1')/2;
        
        % Check RCU
%         rho = norm(x_h1(1:3,k)-x_h2(1:3,k));
%         if rho>gamma_rcu && zupt2(k)==0
%             p = 1/rho*(gamma_rcu*x_h2(1:3,k)+(rho-gamma_rcu)*x_h1(1:3,k));
%             K_rcu = P2*H_rcu'*(H_rcu*P2*H_rcu' + R_rcu)^-1;
%             dx = -K_rcu*(x_h2(1:3,k)-p);
%             [x_h2(:,k) quat2]=comp_internal_states(x_h2(:,k),dx,quat2);
%             P2 = (eye(15) - K_rcu*H_rcu)*P2;
%             P2=(P2+P2')/2;
%         end
        
        % compute estimated gravity component
        f_u=mean(u_h1(1));
        f_v=mean(u_h1(2));
        f_w=mean(u_h1(3));
        ge = sqrt(f_u^2 + f_v^2 + f_w^2);
        
        % Check gravity condition update (GCU)
        if ge<10
            roll=atan2(-f_v,-f_w);
            pitch=atan2(f_u,sqrt(f_v^2+f_w^2));
            p = [roll;pitch];
%             SolveLMI
            %K_gcu = P1*H_gcu'*(H_gcu*P1*H_gcu' + R_gcu)^-1;
            %dx = -K_gcu*(x_h1(4:5,k)-p);
%             dx = -L_gcu*[x_h1(4:5,k)-p;0]*Ts;
%             [x_h1(:,k) quat1]=comp_internal_states(x_h1(:,k),dx,quat1);
%             P1 = (eye(15) - K_gcu*H_gcu)*P1;
%             P1=(P1+P1')/2;
        end
        
        %Store the diagonal of the state covariance matrix P.
        cov1(:,k)=diag(P1);
        cov2(:,k)=diag(P2);
    end
    if zupt2(k)==true;
        
        % Calculate the Kalman filter gain
        K2=(P2*H2')/(H2*P2*H2'+R2);
        
        % Calculate the prediction error. Since the detector hypothesis 
        % is that the platform has zero velocity, the prediction error is 
        % equal to zero minu the estimated velocity.    
        z=-x_h2(4:6,k);   
        
        % Estimation of the perturbations in the estimated navigation
        % states
        dx=K2*z;
        
        % Correct the navigation state using the estimated perturbations. 
        % (Subfunction located further down in the file.)
        [x_h2(:,k) quat2]=comp_internal_states(x_h2(:,k),dx,quat2);     % Subfunction located further down in the file.
    
        
        % Update the filter state covariance matrix P.
        P2=(Id2-K2*H2)*P2;
        
        % Make sure the filter state covariance matrix is symmetric. 
        P2=(P2+P2')/2;
        
        % Check RCU
%         rho = norm(x_h1(1:3,k)-x_h2(1:3,k));
%         if rho>gamma_rcu && zupt1(k)==0
%             p = 1/rho*(gamma_rcu*x_h1(1:3,k)+(rho-gamma_rcu)*x_h2(1:3,k));
%             K_rcu = P1*H_rcu'*(H_rcu*P1*H_rcu' + R_rcu)^-1;
%             dx = -K_rcu*(x_h1(1:3,k)-p);
%             [x_h1(:,k) quat1]=comp_internal_states(x_h1(:,k),dx,quat1);
%             P1 = (eye(15) - K_rcu*H_rcu)*P1;
%             P1=(P1+P1')/2;
%         end
        
        % compute estimated gravity component
        f_u=mean(u_h2(1));
        f_v=mean(u_h2(2));
        f_w=mean(u_h2(3));
        ge = sqrt(f_u^2 + f_v^2 + f_w^2);
        
        % Check gravity condition update (GCU)
%         if ge<10
%             roll=atan2(-f_v,-f_w);
%             pitch=atan2(f_u,sqrt(f_v^2+f_w^2));
%             p = [roll;pitch];
%             K_gcu = P2*H_gcu'*(H_gcu*P2*H_gcu' + R_gcu)^-1;
%             dx = -K_gcu*(x_h2(4:5,k)-p);
%             [x_h2(:,k) quat2]=comp_internal_states(x_h2(:,k),dx,quat2);
%             P2 = (eye(15) - K_gcu*H_gcu)*P2;
%             P2=(P2+P2')/2;
%         end
        
        % Store the diagonal of the state covariance matrix P.
        cov1(:,k)=diag(P1);
        cov2(:,k)=diag(P2);
    end  
end
figure(8)
plot(unwrap(real(x_h1(9,:))')*180/pi)
figure(9)
plot(sw_st)
end



%% SUBFUNCTIONS

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  funtion [x_h cov Id]=init_vec(N,P)
%
%>
%> @brief Function that allocates memmory for the output of zero-velocity
%> aided inertial navigation algorithm.
%>
%> @param[out]  x_h     Matrix with the estimated navigation states. Each row holds the [position, velocity, attitude, (biases, if turned on),(scale factors, if turned on)] for time instant k, k=1,...N.
%> @param[out]  cov     Matrix with the diagonal elements of the state covariance matrices.
%> @param[out]  Id      Identity matrix.
%> @param[in]   N       The length of the IMU data vector u, i.e., the number of samples.
%> @param[in]   P       Initial state covariance matrix.
%>
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [x_h cov Id]=init_vec(N,P)

global simdata


% Check which errors that are included in the state space model
if (strcmp(simdata.scalefactors,'on') && strcmp(simdata.biases,'on'))
    % Both scale and bias errors included
    cov=zeros(9+6+6,N);
    x_h=zeros(9+6+6,N);
    
elseif strcmp(simdata.scalefactors,'on') && strcmp(simdata.biases,'off')
    % Scale errors included
    cov=zeros(9+6,N);
    x_h=zeros(9+6,N);
    
elseif strcmp(simdata.scalefactors,'off') && strcmp(simdata.biases,'on')
    % Bias errors included
    cov=zeros(9+6,N);
    x_h=zeros(9+6,N);
else
    % Only the standard errors included
    cov=zeros(9,N);
    x_h=zeros(9,N);
end


Id=eye(size(P));
cov(:,1)=diag(P);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  funtion [x quat]=init_Nav_eq(u)
%
%> @brief Function that calculates the initial state of the navigation
%> equations.
%>
%> @details Function that calculates the initial state of the navigation
%> equations. That is, it does a simple initial alignment of the navigation
%> system, where the roll and pitch of the system is estimated from the
%> 20 first accelerometer readings. All other states are set according to
%> the information given in the function "settings.m".
%>
%> @param[out]  x     Initial navigation state vector.
%> @param[out]  quat  Quaternion vector, representating the initial attitude of the platform.
%> @param[in]   u     Matrix with the IMU data.
%>
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [x quat]=init_Nav_eq(u)

global simdata;

% Under the assumption that the system is stationary during the first 20
% samples, the initial roll and pitch is calculate from the 20 first
% accelerometer readings.
f_u=mean(u(1,1:20));
f_v=mean(u(2,1:20));
f_w=mean(u(3,1:20));

roll=atan2(-f_v,-f_w);
pitch=atan2(f_u,sqrt(f_v^2+f_w^2));


% Set the attitude vector
attitude=[roll pitch simdata.init_heading]';

% Calculate quaternion corresponing to the initial attitude
Rb2t=Rt2b(attitude)';
quat=dcm2q(Rb2t);

% Set the initial state vector
x=zeros(9,1);
x(1:3,1)=simdata.init_pos;
x(7:9,1)=attitude;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  funtion [P Q R H]=init_filter()
%
%> @brief Function that initializes the Kalman filter.
%>
%> @details Function that initializes the Kalman filter. That is, the
%> function generates the initial covariance matrix P, the process noise
%> covariance matrix Q, the measurement noise covariance matrix R, and
%> observation matrix H, based upon the settings defined in the function
%> settings.m
%>
%> @param[out]   P     Initial state covariance matrix.
%> @param[out]   Q     Process noise covariance matrix.
%> @param[out]   R     Measurement noise covariance matrix.
%> @param[out]   H     Measurement observation matrix.
%>
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [P Q R H]=init_filter

global simdata;

% Check which errors that are included in the state space model and
% allocate P,Q, and H matrices with the right size.

if (strcmp(simdata.scalefactors,'on') && strcmp(simdata.biases,'on')) % Both scale and bias errors included
    
    
    
    %  Initial state covariance matrix
    P=zeros(9+6+6);
    P(10:12,10:12)=diag(simdata.sigma_initial_acc_bias.^2);
    P(13:15,13:15)=diag(simdata.sigma_initial_gyro_bias.^2);
    P(16:18,16:18)=diag(simdata.sigma_initial_acc_scale.^2);
    P(19:21,19:21)=diag(simdata.sigma_initial_gyro_scale.^2);
    
    
    
    Q=zeros(12);
    Q(7:9,7:9)=diag(simdata.acc_bias_driving_noise.^2);
    Q(10:12,10:12)=diag(simdata.gyro_bias_driving_noise.^2);
    
    
    H=zeros(3,9+6+6);
    
elseif strcmp(simdata.scalefactors,'on') && strcmp(simdata.biases,'off') % Scale errors included
    
    
    
    %  Initial state covariance matrix
    P=zeros(9+6);
    P(10:12,10:12)=diag(simdata.sigma_initial_acc_scale.^2);
    P(13:15,13:15)=diag(simdata.sigma_initial_gyro_scale.^2);
    
    % Process noise covariance matrix
    Q=zeros(6);
    
    % Observation matrix
    H=zeros(3,9+6);
    
elseif strcmp(simdata.scalefactors,'off') && strcmp(simdata.biases,'on') % Bias errors included
    
    
    
    %  Initial state covariance matrix
    P=zeros(9+6);
    P(10:12,10:12)=diag(simdata.sigma_initial_acc_bias.^2);
    P(13:15,13:15)=diag(simdata.sigma_initial_gyro_bias.^2);
    
    % Process noise covariance matrix
    Q=zeros(12);
    Q(1:3,1:3)=diag(simdata.acc_bias_driving_noise.^2);
    Q(4:6,4:6)=diag(simdata.gyro_bias_driving_noise.^2);
    Q(7:9,7:9)=diag(simdata.acc_bias_driving_noise.^2);
    Q(10:12,10:12)=diag(simdata.gyro_bias_driving_noise.^2);
    
    % Observation matrix
    H=zeros(3,9+6);
    
else % Only the standard errors included
    
    %  Initial state covariance matrix
    P=zeros(9);
    
    % Process noise covariance matrix
    Q=zeros(6);
    
    % Observation matrix
    H=zeros(3,9);
end


% General values for the observation matrix H
H(1:3,4:6)=eye(3);

% General values for the initial covariance matrix P
P(1:3,1:3)=diag(simdata.sigma_initial_pos.^2);
P(4:6,4:6)=diag(simdata.sigma_initial_vel.^2);
P(7:9,7:9)=diag(simdata.sigma_initial_att.^2);

% General values for the process noise covariance matrix Q
Q(1:3,1:3)=diag(simdata.sigma_acc.^2);
Q(4:6,4:6)=diag(simdata.sigma_gyro.^2);

% General values for the measurement noise matrix R
R=diag(simdata.sigma_vel.^2);

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  funtion [y,q]=Navigation_equations(x,u,q)
%
%> @brief The mechanized navigation equations of the inertial navigation 
%> system. 
%>
%> @details The mechanized navigation equations of the inertial navigation 
%> system. That is, the function takes the old state (position, velocity, 
%> and attitude) of the navigation system, togheter with the current IMU
%> data measurements (specific force, angular rates), and calculates 
%> the current state of the navigation system.  
%>
%> @Note The mechanization of the navigation equations that has been 
%> implemented is very simple, and several higher order terms has been 
%> neglected. Therefore, this mechanization of the navigation equations 
%> should only be used in systems using low-cost sensor and where only 
%> moderate velocities can be expected. 
%>
%> @param[out]   y     New navigation state [position,velocity, attitude (euler angles].
%> @param[out]   q     New quaternions
%> @param[in]    x     Old navigation state
%> @param[in]    u     IMU data [specific force, angular rates].
%> @param[in]    q     Old quaternions
%>
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [y,q]=Navigation_equations(x,u,q)

global simdata;

% Allocate memmory for the output vector
y=zeros(size(x));

% Get sampling period of the system
Ts=simdata.Ts;

%*************************************************************************%
% Update the quaternion vector "q"  given the angular rate measurements.
%*************************************************************************%

w_tb=u(4:6);

P=w_tb(1)*Ts;
Q=w_tb(2)*Ts;
R=w_tb(3)*Ts;

OMEGA=zeros(4);
OMEGA(1,1:4)=0.5*[0 R -Q P];
OMEGA(2,1:4)=0.5*[-R 0 P Q];
OMEGA(3,1:4)=0.5*[Q -P 0 R];
OMEGA(4,1:4)=0.5*[-P -Q -R 0];

v=norm(w_tb)*Ts;

if v~=0
    q=(cos(v/2)*eye(4)+2/v*sin(v/2)*OMEGA )*q;
    q=q./norm(q);
end

%*************************************************************************%
% Use the update quaternion to get attitude of the navigation system in
% terms of Euler angles.
%*************************************************************************%

% Get the roll, pitch and yaw
Rb2t=q2dcm(q);
% roll
y(7)=atan2(Rb2t(3,2),Rb2t(3,3));

% pitch
y(8)=-atan(Rb2t(3,1)/sqrt(1-Rb2t(3,1)^2));

%yaw
y(9)=atan2(Rb2t(2,1),Rb2t(1,1));


%*************************************************************************%
% Update position and velocity states using the measured specific force,
% and the newly calculated attitude.
%*************************************************************************%

% Gravity vector
g_t=[0 0 simdata.g]';

% Transform the specificforce vector into navigation coordinate frame.
f_t=q2dcm(q)*u(1:3);

% Subtract (add) the gravity, to obtain accelerations in navigation
% coordinat system.
acc_t=f_t+g_t;

% State space model matrices
A=eye(6);
A(1,4)=Ts;
A(2,5)=Ts;
A(3,6)=Ts;

B=[(Ts^2)/2*eye(3);Ts*eye(3)];

% Update the position and velocity estimates.
y(1:6)=A*x(1:6)+B*acc_t;
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  funtion [F G]=state_matrix(q,u)
%
%> @brief Function for calculating the state transition matrix F and 
%> the process noise gain matrix G. 
%>
%> @details Function for calculating the state transition matrix F and 
%> the process noise gain matrix G, given the current orientation of 
%> the platform and the specific force vector.  
%>
%> @param[out]   F     State transition matrix.
%> @param[out]   G     Process noise gain matrix.
%> @param[in]    u     IMU data [specific force, angular rates].
%> @param[in]    q     Old quaternions
%>
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [F G]=state_matrix(q,u)

global simdata

% Convert quaternion to a rotation matrix
Rb2t=q2dcm(q);

% Transform measured force to force in
% the navigation coordinate system.
f_t=Rb2t*u(1:3);

% Create a ske symmetric matrix of the specific fore vector
St=[0 -f_t(3) f_t(2); f_t(3) 0 -f_t(1); -f_t(2) f_t(1) 0];

% Zero matrix
O=zeros(3);

% Identity matrix
I=eye(3);

% Diagonal matrices with the specific fore and angular rate along the
% diagonals.
Da=diag(u(1:3));
Dg=diag(u(4:6));

% Correlation constant for accelerometer and gyro biases
B1=-1/simdata.acc_bias_instability_time_constant_filter*eye(3);
B2=-1/simdata.gyro_bias_instability_time_constant_filter*eye(3);


% Check which errors that are included in the state space model
if (strcmp(simdata.scalefactors,'on') && strcmp(simdata.biases,'on'))
    % Both scale and bias errors included
    Fc=[O I O   O     O     O         O    ;
        O O St Rb2t   O    Rb2t*Da    O    ;
        O O O   O   -Rb2t   O     -Rb2t*Dg ;
        O O O   B1    O     O         O    ;
        O O O   O     B2    O         O    ;
        O O O   O     O     O         O    ;
        O O O   O     O     O         O   ];
    
    % Noise gain matrix
    Gc=[O O O O; Rb2t O O O; O -Rb2t O O; O O I O; O O O I; O O O O; O O O O];
    
    
    
elseif strcmp(simdata.scalefactors,'on') && strcmp(simdata.biases,'off')
    % Scale errors included
    Fc=[O I O       O       O    ;
        O O St  Rb2t*Da     O    ;
        O O O       O   -Rb2t*Dg ;
        O O O       O       O    ;
        O O O       O       O];
    
    % Noise gain matrix
    Gc=[O O; Rb2t O ; O -Rb2t; O O; O O];
    
elseif strcmp(simdata.scalefactors,'off') && strcmp(simdata.biases,'on')
    % Bias errors included
    Fc=[O I O O O;
        O O St Rb2t O;
        O O O O -Rb2t;
        O O O B1 O;
        O O O O B2];
    
    % Noise gain matrix
    Gc=[O O O O; Rb2t O O O; O -Rb2t O O; O O I O; O O O I];
    
else
    Fc=[O I O;
        O O St;
        O O O];
    
    % Noise gain matrix
    Gc=[O O; Rb2t O; O -Rb2t];
    
end


% Approximation of the discret time state transition matrices
F=eye(size(Fc))+simdata.Ts*Fc;
G=simdata.Ts*Gc;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  function [x_out q_out]=comp_internal_states(x_in,dx,q_in)
%
%> @brief Function that corrects the estimated navigation states with 
%> the by the Kalman filter estimated  system perturbations (errors). 
%>
%> @details Function that corrects the estimated navigation states with 
%> the by the Kalman filter estimated system perturbations (errors). That
%> is, the current position an velocity estimates of the navigation 
%> platform is corrected by adding the estimated system perturbations to 
%> these states. To correct the orientation state (Euler angles and 
%> quaternion vector), the quaternion vector are first converted into a
%> rotation matrix, which then is corrected using the estimated orientation 
%> perturbations. The corrected rotation matrix is then transformed back 
%> into a quaternion vector, as well as the equivalent vector of Euler 
%> angles.         
%>
%> @param[out]   x_out     Corrected (posteriori) navigation state vector.
%> @param[out]   q_out     Corrected (posteriori) quaternion vector.
%> @param[in]    x_in      A priori estimated navigation state vector.
%> @param[in]    q_in      A priori estimated quaternion vector.
%> @param[in]    dx        Vector of system perturbations
%>
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [x_out q_out]=comp_internal_states(x_in,dx,q_in)

% Convert quaternion to a rotation matrix
R=q2dcm(q_in);

% Correct the state vector
x_out=x_in+dx;

% Correct the rotation matrics
epsilon=dx(7:9);
OMEGA=[0 -epsilon(3) epsilon(2); epsilon(3) 0 -epsilon(1); -epsilon(2) epsilon(1) 0];
R=(eye(3)-OMEGA)*R;


% Get the corrected roll, pitch and heading from the corrected rotation
% matrix
x_out(7)=atan2(R(3,2),R(3,3));
x_out(8)=-atan(R(3,1)/sqrt(1-R(3,1)^2));
x_out(9)=atan2(R(2,1),R(1,1));

% Calculte the corrected quaternions
q_out=dcm2q(R);



end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  function u_out=comp_imu_errors(u_in,x_h)
%
%> @brief Function that compensats for the artifacts in the IMU  
%> measurements with the current estimates of the sensors biases and/or  
%> scale factor errors.  
%>
%> @details Function that compensats for the artifacts in the IMU  
%> measurements with the current estimates of the sensors biases and/or  
%> scale factor errors. If the sensor errors are not included in the state 
%> space model used in the Kalman filter, no correction/compensation is 
%> done.   
%>     
%> @param[out]   u_out     Corrected/compensated IMU measurements.
%> @param[in]    u_in      Raw IMU measurements.
%> @param[in]    x_h       Navigation state vector, where the last states are the estimated sensor errors. 
%>
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function u_out=comp_imu_errors(u_in,x_h)

global simdata;

% Check which errors that are included in the state space model

if (strcmp(simdata.scalefactors,'on') && strcmp(simdata.biases,'on'))
    
    % Both scale and bias errors included
    temp=1./(ones(6,1)-x_h(16:end));
    u_out=diag(temp)*u_in+x_h(10:15);
    
    
elseif strcmp(simdata.scalefactors,'on') && strcmp(simdata.biases,'off')
    
    % Scale errors included
    temp=1./(ones(6,1)-x_h(10:end));
    u_out=diag(temp)*u_in;
    
elseif strcmp(simdata.scalefactors,'off') && strcmp(simdata.biases,'on')
    
    % Bias errors included
    u_out=u_in+x_h(10:end);
    
else
    
    % Only the standard errors included
    u_out=u_in;
end

end





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  function R=q2dcm(q)
%
%>
%> @brief Function that converts a  quaternion vector to a directional
%> cosine matrix (rotation matrix) 
%>
%> @param[out]   R      Rotation matrix.
%> @param[in]    q      Quaternion vector.
%>
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function R=q2dcm(q)

p=zeros(6,1);

p(1:4)=q.^2;

p(5)=p(2)+p(3);

if p(1)+p(4)+p(5)~=0
   p(6)=2/(p(1)+p(4)+p(5)); 
else
   p(6)=0;
end


R(1,1)=1-p(6)*p(5);
R(2,2)=1-p(6)*(p(1)+p(3));
R(3,3)=1-p(6)*(p(1)+p(2));

p(1)=p(6)*q(1); 
p(2)=p(6)*q(2);
p(5)=p(6)*q(3)*q(4);
p(6)=p(1)*q(2);

R(1,2)=p(6)-p(5);
R(2,1)=p(6)+p(5);

p(5)=p(2)*q(4);
p(6)=p(1)*q(3);

R(1,3)=p(6)+p(5);
R(3,1)=p(6)-p(5);

p(5)=p(1)*q(4);
p(6)=p(2)*q(3);

R(2,3)=p(6)-p(5);
R(3,2)=p(6)+p(5);

end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  function q=dcm2q(R)
%
%>
%> @brief Function that converts a directional cosine matrix (rotation 
%> matrix) in to a quaternion vector. 
%>
%> @param[out]    q      Quaternion vector.
%> @param[in]   R      Rotation matrix.
%>
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function q=dcm2q(R)

T = 1 + R(1,1) + R(2,2) + R(3,3);

if T > 10^-8
    
    S = 0.5 / sqrt(T);
    qw = 0.25 / S;
    qx = ( R(3,2) - R(2,3) ) * S;
    qy = ( R(1,3) - R(3,1) ) * S;
    qz = ( R(2,1) - R(1,2) ) * S;

else
    
    if (R(1,1) > R(2,2)) && (R(1,1) > R(3,3))
        
        S = sqrt( 1 + R(1,1) - R(2,2) - R(3,3)) * 2; % S=4*qx
        qw = (R(3,2) - R(2,3)) / S;
        qx = 0.25 * S;
        qy = (R(1,2) + R(2,1)) / S;
        qz = (R(1,3) + R(3,1)) / S;
        
    elseif (R(2,2) > R(3,3))
        
        S = sqrt( 1 + R(2,2) - R(1,1) - R(3,3) ) * 2; %S=4*qy
        qw = (R(1,3) - R(3,1)) / S;
        qx = (R(1,2) + R(2,1)) / S;
        qy = 0.25 * S;
        qz = (R(2,3) + R(3,2)) / S;

    else
        
        S = sqrt( 1 + R(3,3) - R(1,1) - R(2,2) ) * 2; % S=4*qz
        qw = (R(2,1) - R(1,2)) / S;
        qx = (R(1,3) + R(3,1)) / S;
        qy = (R(2,3) + R(3,2)) / S;
        qz = 0.25 * S;

    end

end

%Store in vector
q = [qx qy qz qw]';
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  function R=Rt2b(ang)
%
%>
%> @brief Function that calculates the rotation matrix for rotating a 
%> vector from coordinate frame t to the coordinate frame b, given a
%> vector of Euler angles.
%>
%> @param[out]  R      Rotation matrix.
%> @param[in]   ang    Euler angles [roll,pitch,heading]
%>
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function R=Rt2b(ang)


cr=cos(ang(1));
sr=sin(ang(1));

cp=cos(ang(2));
sp=sin(ang(2));

cy=cos(ang(3));
sy=sin(ang(3));

R=[cy*cp sy*cp -sp; 
    -sy*cr+cy*sp*sr cy*cr+sy*sp*sr cp*sr; 
    sy*sr+cy*sp*cr -cy*sr+sy*sp*cr cp*cr];

end

