function frames = simulate(opt)
% Simulates the five link robot
% in order to run the simulation run the file named as
% runSimulationFiveLink.
% Last modification: 30/03/2014 (by Feliphe G. Galiza)

close all

subplot(3,3,3);
plot(opt.ti,opt.q)
legend('x1','x2','x3','x4','x5','Location','northeastoutside')
grid

subplot(3,3,6);
plot(opt.ti,opt.qd)
legend('xd1','xd2','xd3','xd4','xd5','Location','northeastoutside')
grid

subplot(3,3,9);
plot(opt.ti,opt.xcm','r')
legend('xCOM','Location','northeastoutside')
grid
% hold on
% plot(opt.ti,opt.xzmp','r')
% legend('xCOM','xZMP')

sim = subplot(3,3,[1 2 4 5 7 8]);

for step=1:1:1
    
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
    
    axis equal
 
    grid
    drawnow
    pause(1/40)
    frames(t)=getframe(gcf);
end    
end
end

