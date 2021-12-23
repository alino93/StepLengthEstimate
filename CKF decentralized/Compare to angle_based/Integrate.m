%*** (events(i,1) == 1 || events(i,1) == 3)
%*** remove XR XL tR tL
function [tR, vR, pR, tL, vL, pL, biasR, biasL] = Integrate(aR, aL, tiltRE, tiltLE, events, t)
% Find start and end of steps from events
n = 0;
i = 1;
stepR = zeros(2,2);
while i < length(t)
    if events(i,2) == 11 %&& (events(i,1) == 1 || events(i,1) == 3)
        n = n + 1;
        stepR(n,1) = i;
    end
    if events(i,2) == 12 %&& (events(i,1) == 1 || events(i,1) == 3)
        stepR(n,2) = i;
    end
    if events(i,1) == 2 && events(i,1) == 4
        while events(i,1) ~= 1  && events(i,1) ~= 3 && i < length(t)
            i = i + 1;
        end
        i = i - 1;
        n = n + 1;
    end
    i = i + 1;
end
n = 0;
i = 1;
stepL = zeros(2,2);
while i < length(t)
    if events(i,3) == 21 %&& (events(i,1) == 1 || events(i,1) == 3)
        n = n + 1;
        stepL(n,1) = i;
    end
    if events(i,3) == 22 %&& (events(i,1) == 1 || events(i,1) == 3)
        stepL(n,2) = i;
    end
    if events(i,1) == 2 && events(i,1) == 4
        while events(i,1) ~= 1  && events(i,1) ~= 3 && i < length(t)
            i = i + 1;
        end
        i = i - 1;
        n = n + 1;
    end
    i = i + 1;
end
% Integrate acceleration
vR = zeros(2,length(t));    % velocity right
vL = zeros(2,length(t));    % velocity left
pR = vR;                    % position right
pL = vL;                    % position left
tR = [];
tL = [];
biasR = zeros(2,1);  % steps displacement right
biasL = zeros(2,1);  % steps displacement left
% Right foot
for i=1:1:length(stepR(:,1))
    st = stepR(i,1);
    ed = stepR(i,2);
    if st ~= 0 && ed ~= 0
        vR (:,st:ed) = cumtrapz(t(st:ed),aR(:,st:ed),2);
        pR (:,st:ed) = cumtrapz(t(st:ed),vR(:,st:ed),2);
        if pR(1,ed-5) > 0
            tR(end+1) = ed;
        elseif pR(1,ed-5) < 0
            tR(end+1) = -ed;
        end
        %         if abs(pR(1,ed))>2 || abs(pR(2,ed))>1
%             vR (:,st:ed) = 0;
%             pR (:,st:ed) = 0;
%             continue
%         end
        %bias for only Y direction
        tilt_cos = trapz(t(st:ed),cumtrapz(t(st:ed), cos(tiltRE(st:ed))));
        tilt_sin = trapz(t(st:ed),cumtrapz(t(st:ed), sin(tiltRE(st:ed))));
        R = [tilt_cos -tilt_sin; tilt_sin tilt_cos];
        biasR = [biasR R^-1 * ([0; pR(2,ed)] - [0; 0])];
    end
end
biasR = biasR(:,2:end);
%Left foot
for i=1:1:length(stepL(:,1))
    st = stepL(i,1);
    ed = stepL(i,2);
    if st ~= 0 && ed ~= 0
        vL (:,st:ed) = cumtrapz(t(st:ed),aL(:,st:ed),2);
        pL (:,st:ed) = cumtrapz(t(st:ed),vL(:,st:ed),2);
        if abs(pL(1,ed))>2 || abs(pL(2,ed))>1
            vL (:,st:ed) = 0;
            pL (:,st:ed) = 0;
            continue
        end
        if pL(1,st+40) > 0
            tL(end+1) = ed;
        elseif pL(1,st+40) < 0
            tL(end+1) = -ed;
        end
        %bias for only Y direction
        tilt_cos = trapz(t(st:ed),cumtrapz(t(st:ed), cos(tiltLE(st:ed))));
        tilt_sin = trapz(t(st:ed),cumtrapz(t(st:ed), sin(tiltLE(st:ed))));
        R = [tilt_cos -tilt_sin; tilt_sin tilt_cos];
        biasL = [biasL R^-1 * ([0; pL(2,ed)] - [0; 0])];
    end
end
biasL = biasL(:,2:end);

