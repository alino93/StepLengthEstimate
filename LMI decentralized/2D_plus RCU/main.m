%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%>
%> @file Main.m
%>                                                                           
%> @brief This is the skeleton file for processing data from a foot mounted
%> IMU. The data is processed using a Kalman filter based zero-velocity 
%> aided inertial navigation system algorithm.
%> 
%> @details This is the skeleton file for processing data data from a foot
%> mounted inertial measurement unit (IMU). The data is processed using a 
%> Kalman filter based zero-velocity aided inertial navigation system. The
%> processing is done in the following order. 
%> 
%> \li The IMU data and the settings controlling the Kalman filter is loaded.
%> \li The zero-velocity detector process all the IMU data.
%> \li The filter algorithm is processed.
%> \li The in data and results of the processing is plotted.
%>
%> @authors Isaac Skog, John-Olof Nilsson
%> @copyright Copyright (c) 2011 OpenShoe, ISC License (open source)
%>
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Loads the algorithm settings and the IMU data
disp('Loads the algorithm settings and the IMU data')
[u1 ,u2]=settings();


%% Run the zero-velocity detector 
disp('Runs the zero velocity detector')
[zupt1 T]=zero_velocity_detector(u1);
[zupt2 T]=zero_velocity_detector(u2);

%% Run the Kalman filter
disp('Runs the filter')
% errors = [];
% j = 0;
% for k1 = 1:1:5
%   for k2 = 1:1:5
%     j = j + 1;
%     [x_h1, x_h2, cov1, cov2]=ZUPTaidedINS(u1,u2,zupt1,zupt2,k1,k2);
%     dr = 0;
%     for i=101:100:length(x_h1(1,:))
%         dx = x_h1(1,i)-x_h1(1,i-100);
%         dy = x_h1(2,i)-x_h1(2,i-100);
%         dr = dr + sqrt(dx^2+dy^2);  
%     end
%     errors = [errors;dr,k1,k2];
%   end
% end
% [m, ind] = min(errors(:,1));
% var = errors(ind,2);
% [x_h1, x_h2, cov1, cov2]=ZUPTaidedINS(u1,u2,zupt1,zupt2,errors(ind,2),errors(ind,3)); %7,7
[x_h1, x_h2, cov1, cov2,L_zupt]=ZUPTaidedINS(u1,u2,zupt1,zupt2,100.0,100.00); 
%% Print the horizontal error and spherical error at the end of the
%% trajectory
sprintf('Horizontal error right = %0.5g , Spherical error = %0.5g',sqrt(sum((x_h1(1:2,end)).^2)), sqrt(sum((x_h1(1:3,end)).^2)))
sprintf('Horizontal error left= %0.5g , Spherical error = %0.5g',sqrt(sum((x_h2(1:2,end)).^2)), sqrt(sum((x_h2(1:3,end)).^2)))
dr1 = 0;dr2 = 0;
for i=101:100:length(x_h1(1,:))
    dx = x_h1(1,i)-x_h1(1,i-100);
    dy = x_h1(2,i)-x_h1(2,i-100);
    dr1 = dr1 + sqrt(dx^2+dy^2);  
    
    dx = x_h2(1,i)-x_h2(1,i-100);
    dy = x_h2(2,i)-x_h2(2,i-100);
    dr2 = dr2 + sqrt(dx^2+dy^2);
end

% sprintf('Total horiz trajectory path left (m) = %0.5g',dr1)
% sprintf('Total horiz trajectory path right (m) = %0.5g',dr2)

sprintf('Horizontal error mean= %0.5g',(sqrt(sum((x_h2(1:2,end)).^2))+sqrt(sum((x_h1(1:2,end)).^2)))/2)
sprintf('Total horiz trajectory path mean (m) = %0.5g',(dr1+dr2)/2)
%% View the result 
disp('Views the data')
view_data;
