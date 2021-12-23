% %animation of body posture
% %leg and body lengths
% Ll=0.9;
% Ls=0.43;
% Lt=Ll-Ls;
% Lb=0.85;
% 
% % tilt estimation
% tiltR = tiltRE - mean(tiltRE(1:100));
% tiltL = tiltLE - mean(tiltLE(1:100));
% tiltT = tiltTE - mean(tiltTE(1:100));
% 
% figure(33)
% plot (t,tiltT)
% %postion from angles
% XRS = zeros(1,length(events(:,1)));
% XLS = zeros(1,length(events(:,1)));
% XS = 0; %total step lengthes
% flagR = 0;
% flagL = 0;
% for i=1:1:length(events(:,1))
%    SL = Ll*(sin(abs(tiltR(i)))+sin(abs(tiltL(i))));
%    XRS(i) = XS + flagR*(1+2*floor(sin(tiltR(i))))*SL;
%    XLS(i) = XS + flagL*(1+2*floor(sin(tiltL(i))))*SL;
%    if events(i,2) == 12
%        XRS(i+1:end) = XRS(:,i+1:end) + SL;
%        XS = XS + SL;
%        flagR = 0;
%        flagL = 1;
%    end
%    if events(i,3) == 22
%        XLS(:,i+1:end) = XLS(:,i+1:end) + SL;
%        XS = XS + SL;
%        flagR = 1;
%        flagL = 0; 
%    end
% end
% %find feet time update displacement
% CPR = pR; %cumlutive position right
% CPL = pL; %cumlutive position left
% for i=1:1:length(events(:,1))
%    if events(i,2) == 12
%        CPR(:,i+1:end) = CPR(:,i+1:end) + pR(:,i);
%    end
%    if events(i,3) == 22
%        CPL(:,i+1:end) = CPL(:,i+1:end) + pL(:,i);
%    end
% end
% %measurement update
% XR = CPR(1,:);
% XL = CPL(1,:);
% K = 1;
% for i=1:1:length(events(:,1))
%    if events(i,2) == 12
%        XR(i+1:end) = XR(i+1:end) + K * (XRS(i) - XR(i));
%    end
%    if events(i,3) == 22
%        XL(i+1:end) = XL(i+1:end) + K * (XLS(i) - XL(i));
%    end
% end
% figure(31)
% %CPR
% plot(t, CPR) 
% figure(32)
% %CPR
% plot(t, XRS)
% figure(33)
% %CPR
% plot(t, XR) 
% 
% figure(34)
% plot(t,tiltR,t,tiltL)
% %foot position based on CPR and CPL
% % XR1 = CPR(1,3471);
% % YR1 = CPR(2,3471);
% % XR2 = XR1 + Ll*sin(-tiltRE(3471));
% % YR2 = YR1 + Ll*cos(-tiltRE(3471));
% % 
% % XL1 = CPL(1,3471);
% % YL1 = CPL(2,3471);
% % XL2 = XL1 + Ll*sin(-tiltLE(3471));
% % YL2 = YL1 + Ll*cos(-tiltLE(3471));
% %
% % figure(30)
% % %draw right leg
% % plot([XR1; XR2], [YR1; YR2],'b','linewidth',2) 
% % hold on
% % %draw left leg
% % plot([XL1; XL2], [YL1; YL2],'r','linewidth',2)
% % hold off
% 
% CPB = (CPR + CPL)/2;
% for i=1:1:length(events(:,1))
%     if events(i,3) == 22
%         %body position as mean of right and left foot position
%         j = i + 10;
%         XB = CPB(1,j);
%         YB = 0 + ( Ll*cos(tiltR(j)) + Ll*cos(tiltL(j)) )/2;
% 
%         %foot postions from body position and foot angles
%         XR = XB + Ll*sin(tiltR(j));
%         YR = YB - Ll*cos(tiltR(j));
%         XL = XB + Ll*sin(tiltL(j));
%         YL = YB - Ll*cos(tiltL(j));
%         
%         %head position
%         XH = XB + Lb*sin(-tiltT(j));
%         YH = YB + Lb*cos(tiltT(j));
%         
%         figure(30)
%         %draw right leg
%         plot([XB; XR], [YB; YR],'b','linewidth',2)
%         title('Posture estimation');
%         xlabel('x (m)');
%         ylabel('y (m)');
%         hold on
%         %draw left leg
%         plot([XB; XL], [YB; YL],'r','linewidth',2)
%         %draw trunk
%         plot([XB; XH], [YB; YH],'g','linewidth',2)
%     end
% end
% hold off
XHd = XH; % head position relative to hip position
YHd = YH + Lb;
steps=Len;
figure(6);
ax = gca;
ax.XLim = [-1 13];
ax.YLim = [-1 3];
hPlot = plot(NaN,NaN,'-','linewidth',2);
title('Posture estimation');
set(gca,'DefaultLineLineWidth',2);

frame(steps+1) = struct('cdata',[],'colormap',[]);

v = VideoWriter('video.avi');
v.FrameRate = 10;
open(v);
for k = 1:10:steps
    
%     set(hPlot,'XData',[XH(k); XKr(k)],'YData',[YH(k); YKr(k)],...
%         'XData',[XAr(k); XKr(k)],'YData',[YAr(k); YKr(k)],'color','blue',...
%         'XData',[XH(k); XKl(k)],'YData',[YH(k); YKl(k)],'color','red',...
%         'XData',[XAl(k); XKl(k)],'YData',[YAl(k); YKl(k)],'color','red',...
%         'XData',[XH(k); XHd(k)],'YData',[YH(k); YHd(k)],'color','green');
%     set(hPlot,'XData',[XH(k); XKr(k)],'YData',[YH(k); YKr(k)],'color','blue');
%     set(hPlot,'XData',[XAr(k); XKr(k)],'YData',[YAr(k); YKr(k)],'color','blue');
%     set(hPlot,'XData',[XH(k); XKl(k)],'YData',[YH(k); YKl(k)],'color','red');
%     set(hPlot,'XData',[XAl(k); XKl(k)],'YData',[YAl(k); YKl(k)],'color','red');
%     set(hPlot,'XData',[XH(k); XHd(k)],'YData',[YH(k); YHd(k)],'color','green');
    
    %draw right thigh
    figure(7)
    plot([XH(k); XKr(k)], [YH(k); YKr(k)],'b',...
        [XAr(k); XKr(k)], [YAr(k); YKr(k)],'b',...
        [XH(k); XKl(k)], [YH(k); YKl(k)],'r',...
        [XAl(k); XKl(k)], [YAl(k); YKl(k)],'r',...
        [XH(k); XC(k)], [YH(k); YC(k)],'g','linewidth',2)
    title('Posture estimation');
    axis([-1 3 -1 3]);
    xlabel('x (m)');
    ylabel('y (m)');
    frame(k) = getframe(gcf); 
    writeVideo(v,frame(k));
end
close(v);
%movie(MM,2,10)
