function plot_evtol_feedforward(xout,xdesout,uout)
% Called after simulation: trajectory snapshots, state plots and animation.
animate=true;
if evalin('base','exist(''vtolAnimate'',''var'')')
    animate=evalin('base','vtolAnimate');
end
visibility='off';if animate,visibility='on';end
state=rows(xout,6);reference=rows(xdesout,10);control=rows(uout,2);
time=xout.Time; desired=reference(:,1:2); position=state(:,[1 3]);
folder=fileparts(mfilename('fullpath'));results=fullfile(folder,'results');
if ~isfolder(results),mkdir(results);end
% A planar aircraft silhouette, rotated by the simulated theta.
shape=[-.12 -.035 -.02 .015 .035 .12 .035 .015 -.02 -.035 -.12; ...
       -.02 .005 .055 .055 .005 -.02 -.012 -.03 -.03 -.012 -.02];
fig=figure('Name','VTOL feedforward: path and attitude','NumberTitle','off', ...
    'Color','white','Visible',visibility);
ax=axes(fig);style(ax);hold(ax,'on');
plot(ax,desired(:,1),desired(:,2),'r-','LineWidth',1.4,'DisplayName','Desired trajectory');
plot(ax,position(:,1),position(:,2),'b--','LineWidth',1.3,'DisplayName','Model trajectory');
samples=unique(round(linspace(1,numel(time),13)));
for index=samples
    angle=state(index,5);rotation=[cos(angle) -sin(angle);sin(angle) cos(angle)];
    aircraft=rotation*shape+position(index,:).';
    plot(ax,aircraft(1,:),aircraft(2,:),'Color',[.25 .25 .25],'HandleVisibility','off');
end
axis(ax,'equal');grid(ax,'on');xlabel(ax,'x');ylabel(ax,'y');
title(ax,'Flatness-based feedforward: aircraft position and tilt','Color','black');
legend(ax,'Location','northwest','Color','white','TextColor','black');
xlim(ax,[min(desired(:,1))-.25 max(desired(:,1))+.25]);
ylim(ax,[min(desired(:,2))-.25 max(desired(:,2))+.25]);
exportgraphics(fig,fullfile(results,'feedforward_path.png'),'BackgroundColor','white');
fig2=figure('Name','VTOL feedforward: states and controls','NumberTitle','off', ...
    'Color','white','Visible',visibility);tiledlayout(fig2,3,1);
ax2=nexttile;style(ax2);hold(ax2,'on');
plot(ax2,time,state(:,1),'b-',time,desired(:,1),'b--', ...
    time,state(:,3),'r-',time,desired(:,2),'r--');
grid(ax2,'on');ylabel(ax2,'Position');legend(ax2,'x','x desired','y','y desired','Color','white','TextColor','black');
ax2=nexttile;style(ax2);plot(ax2,time,state(:,5));grid(ax2,'on');ylabel(ax2,'theta (rad)');
ax2=nexttile;style(ax2);plot(ax2,uout.Time,control);grid(ax2,'on');
ylabel(ax2,'Control inputs');xlabel(ax2,'Time (s)');legend(ax2,'u1: thrust','u2: angular acceleration','Color','white','TextColor','black');
exportgraphics(fig2,fullfile(results,'feedforward_states.png'),'BackgroundColor','white');
if animate
    animation=figure('Name','VTOL feedforward: moving aircraft','NumberTitle','off','Color','white');
    ax3=axes(animation);style(ax3);hold(ax3,'on');
    plot(ax3,desired(:,1),desired(:,2),'r-','LineWidth',1.4);
    axis(ax3,'equal');grid(ax3,'on');xlabel(ax3,'x');ylabel(ax3,'y');
    xlim(ax3,[min(desired(:,1))-.3 max(desired(:,1))+.3]);
    ylim(ax3,[min(desired(:,2))-.3 max(desired(:,2))+.3]);
    trail=plot(ax3,NaN,NaN,'b--');aircraftLine=plot(ax3,NaN,NaN,'k-','LineWidth',1.5);
    step=max(1,round(.05/mean(diff(time))));
    frames=unique([1:step:numel(time),numel(time)]);
    for index=frames
        if ~isgraphics(animation),break;end
        angle=state(index,5);rotation=[cos(angle) -sin(angle);sin(angle) cos(angle)];
        aircraft=rotation*shape+position(index,:).';
        set(aircraftLine,'XData',aircraft(1,:),'YData',aircraft(2,:));
        set(trail,'XData',position(1:index,1),'YData',position(1:index,2));
        title(ax3,sprintf('Feedforward flight | t = %.2f s',time(index)),'Color','black');
        drawnow;pause(.025);
    end
else
    close(fig);close(fig2);
end
end
function data=rows(log,width)
if ismatrix(log.Data)&&size(log.Data,1)==numel(log.Time)
    data=log.Data;
else
    data=reshape(log.Data,width,[]).';
end
end
function style(ax)
set(ax,'Color','white','XColor','black','YColor','black');
end
