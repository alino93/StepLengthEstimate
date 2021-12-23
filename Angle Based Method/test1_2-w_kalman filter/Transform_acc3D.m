function a_in = Transform_acc3D(a_body, tiltE, rollE, bias)
%% Tranform to the inertial coordinates
T = 0.01;
g = sqrt( a_body(1,1)^2 + a_body(1,2)^2 + a_body(1,3)^2);
%time
t = 0:T:T*(length(tiltE)-1);

a_in=zeros(length(t),3);%inertial acc
a_body = a_body - bias;
%Right Foot
for i=1:1:length(t)
    %transfer matrix
    Rntob = [1 0 0;0 cos(rollE(i)) sin(rollE(i));0 -sin(rollE(i)) cos(rollE(i))] * [cos(tiltE(i)) 0 -sin(tiltE(i));0 1 0;sin(tiltE(i)) 0 cos(tiltE(i))];
    a_in(i,:) = a_body(i,:)*Rntob;
end
%% remove gravity
a_in(:,3) = a_in(:,3) - g;
