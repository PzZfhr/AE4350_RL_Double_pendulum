function plotDoublePend(th1, th2, l1, l2)

persistent f

L = l1;
d = L/10;

x1 = l1 * sin(th1);
y1 = l1 * cos(th1);
x2 = x1 + l2*sin(th1 + th2);
y2 = y1 + l2*cos(th1 + th2);

% alpha = atan(y2/x2);
% fprintf('Alpha = %.2d[rad]\n', alpha)

if isempty(f) || ~isvalid(f)
    f = figure(...
        'Toolbar','none',...
        'NumberTitle','off',...
        'Name','Double Pendulum Visualization',...
        'Visible','on',...
        'MenuBar','none');
    
    ha = gca(f);
    
    cla(ha);
    set(ha,'XLim',[-2*L,2*L].*1.5);
    set(ha,'YLim',[-2*L,2*L].*1.5);
    
    % set(ha,'xticklabel',[]);
    % set(ha,'yticklabel',[]);
    
    grid(ha,'on');
end

ha = gca(f);

rod1  = findobj(ha,'Tag','rod1');
mass1 = findobj(ha,'Tag','mass1');
rod2  = findobj(ha,'Tag','rod2');
mass2 = findobj(ha,'Tag','mass2');

if isempty(rod1)
    line(ha,'XData',[0,x1],'YData',[0,y1],...
        'Color','g','LineWidth',3,'tag','rod1');
else
    set(rod1,'XData',[0,x1]);
    set(rod1,'YData',[0,y1]);
end

if isempty(rod2)
    line(ha,'XData',[x1,x2],'YData',[y1,y2],...
        'Color','g','LineWidth',3,'tag','rod2');
else
    set(rod2,'XData',[x1,x2]);
    set(rod2,'YData',[y1,y2]);
end

if isempty(mass1)
    rectangle(ha,'Position',[x1-d/2,y1-d/2,d,d],'Curvature',[1,1],...
        'FaceColor','b','EdgeColor',[0,0,0],'LineWidth',3,'tag','mass1');
else
    set(mass1,'Position',[x1-d/2,y1-d/2,d,d]);
end

if isempty(mass2)
    rectangle(ha,'Position',[x2-d/2,y2-d/2,d,d],'Curvature',[1,1],...
        'FaceColor','b','EdgeColor',[0,0,0],'LineWidth',3,'tag','mass2');
else
    set(mass2,'Position',[x2-d/2,y2-d/2,d,d]);
end

drawnow();

