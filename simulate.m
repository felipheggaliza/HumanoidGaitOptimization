function simulate(opt)
% Simulates the five link robot
% in order to run the simulation run the file named as
% runSimulationFiveLink.
% Last modification: 30/03/2014 (by Feliphe G. Galiza)

close all

L1 =0.4275; % [m]

L2 =0.3675; % [m]

L3 = 0.7050; % [m]

L4=L2; % [m]
L5 = L1; % [m]

r1 =0.2138; % [m]
r2 = 0.1838; % [m]
r3 = 0.3525; % [m]
r4 = r2; % [m]
r5 = r1; % [m]

for i=1:1:length(opt.tc)
    [M_d,RHS_d] = eqOfmotion(opt.q(i,:),opt.qd(i,:),opt.u(i,:));
    zmp(i) = computeZMP(opt.q(i,:),opt.qd(i,:),inv(M_d)*RHS_d);
end

zmp =[zmp 0];

subplot(3,3,3);
plot(opt.ti,opt.q)
legend('x1','x2','x3','x4','x5')

subplot(3,3,6);
plot(opt.ti,opt.qd)
legend('xd1','xd2','xd3','xd4','xd5')

subplot(3,3,9);
plot(opt.ti, [opt.xcm' opt.xzmp'])
legend('xCOM','xZMP')

sim = subplot(3,3,[1 2 4 5 7 8]);

for step=1:1:1
    
    rightLegStep(opt,sim);
    
end
end

function rightLegStep(opt,sim)

for t=1:1:length(opt.ti)
    
    theta(1) = opt.q(t,1);
    theta(2) = opt.q(t,2);
    theta(3) = opt.q(t,3);
    theta(4) = opt.q(t,4);
    theta(5) = opt.q(t,5);
    
    [pm, pcm] = computeFK(theta);
    
    x = [0 pm(1,1:3)  pm(1,2) pm(1,4:5) ] ;
    y = [0 pm(2,1:3) pm(2,2) pm(2,4:5) ];
    
     xcm = pcm(1,:);
     ycm = pcm(2,:);
    
    delete(sim);
    sim = subplot(3,3,[1 2 4 5 7 8]);
    
    text = sprintf('Time: %0.2f sec',opt.ti(t));
    title(text)
    
    hold on
    plot(x,y,'b','LineWidth',3)
            hold on
            plot(xcm,ycm,'o',...
                'LineWidth',2,...
                'MarkerSize',10,...
                'MarkerEdgeColor','b',...
                'MarkerFaceColor','g')
    
            hold on
            plot(0,0,'^',...
                'LineWidth',2,...
                'MarkerSize',13,...
                'MarkerEdgeColor','k',...
                'MarkerFaceColor','b')
    
            hold on
            plot(opt.xcm(t),0,'x',...
                'LineWidth',3,...
                'MarkerSize',8,...
                'MarkerEdgeColor','r',...
                'MarkerFaceColor','r')
            legend('CoM')
%             hold on
%             plot(zmp(t),0,'x',...
%                 'LineWidth',3,...
%                 'MarkerSize',8,...
%                 'MarkerEdgeColor','g',...
%                 'MarkerFaceColor','r')
%             legend('ZMP')
    
    %axis([-1.2*(L1+L2+L3) 1.2*(L1+L2+L3) 0 1.2*(L1+L2+L3)])
    axis equal
    %xlim([(-1.2*(L1+L2+L3)) (+1.2*(L1+L2+L3))])
    %ylim([ 0 1.2*(L1+L2+L3)])
    grid
    drawnow
    pause(1/40)
   % frames(t)=getframe(gcf);
end

end


% ------------------- NOT WORKING -----------
% function leftLegStep(opt,sim)
% 
% for t=1:1:length(opt.ti)
%     
%      theta(1) = opt.q(t,1);
%      theta(2) = opt.q(t,5);
%      theta(3) = opt.q(t,4);
%      theta(4) = opt.q(t,3);
%      theta(5) = opt.q(t,2); 
%     
%     [pm, pcm] = computeFK(theta);
%     
%     x = [pcm(1,5) pcm(1,4) pcm(1,2) pcm(1,3) pcm(1,2) pcm(1,1) ] ;
%     y = [pcm(2,5) pcm(2,4) pcm(2,2) pcm(2,3) pcm(2,2) pcm(2,1)  ];
%     
%     % xcm = pcm(1,:);
%     % ycm = pcm(2,:);
%     
%     delete(sim);
%     sim = subplot(3,3,[1 2 4 5 7 8]);
%     
%     text = sprintf('Time: %0.2f sec',opt.ti(t));
%     title(text)
%     
%     hold on
%     plot(x,y,'b','LineWidth',3)
%     %         hold on
%     %         plot(xcm,ycm,'o',...
%     %             'LineWidth',2,...
%     %             'MarkerSize',10,...
%     %             'MarkerEdgeColor','b',...
%     %             'MarkerFaceColor','g')
%     
%     %         hold on
%     %         plot(0,0,'^',...
%     %             'LineWidth',2,...
%     %             'MarkerSize',13,...
%     %             'MarkerEdgeColor','k',...
%     %             'MarkerFaceColor','b')
%     
%     %         hold on
%     %         plot(opt.xcm(t),0,'x',...
%     %             'LineWidth',3,...
%     %             'MarkerSize',8,...
%     %             'MarkerEdgeColor','r',...
%     %             'MarkerFaceColor','r')
%     %         legend('CoM')
%     %         hold on
%     %         plot(zmp(t),0,'x',...
%     %             'LineWidth',3,...
%     %             'MarkerSize',8,...
%     %             'MarkerEdgeColor','g',...
%     %             'MarkerFaceColor','r')
%     %         legend('ZMP')
%     
%     %axis([-1.2*(L1+L2+L3) 1.2*(L1+L2+L3) 0 1.2*(L1+L2+L3)])
%     axis equal
%     %xlim([(-1.2*(L1+L2+L3)) (+1.2*(L1+L2+L3))])
%     %ylim([ 0 1.2*(L1+L2+L3)])
%     grid
%     drawnow
%     pause(1/40)
%     %frames(t)=getframe(gcf);
% end
% 
% 
% end
% 
