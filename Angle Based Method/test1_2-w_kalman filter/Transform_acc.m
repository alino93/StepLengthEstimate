function a_in = Transform_acc(a_body, tiltE, bias)
%% Tranform to the inertial coordinates
T = 0.01;
g = sqrt( a_body(1,1)^2 +  a_body(1,2)^2);
%time
t = 0:T:T*(length(tiltE)-1);

a_in=zeros(2,length(t));%inertial acc
a_body = a_body - bias;
%Right Foot
for i=1:1:length(t)
    transMx = [cos(tiltE(i)) -sin(tiltE(i)); sin(tiltE(i)) cos(tiltE(i))]; %transfer matrix
    a_in(:,i) = transMx*[a_body(i,1); a_body(i,2)];
end
%% remove gravity
a_in(2,:) = a_in(2,:) - g;
