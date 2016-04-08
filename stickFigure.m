
function stickFigure(opt)

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
end   
end