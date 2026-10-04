function schedule_evtol_plot
% To Workspace data becomes available after the model StopFcn returns.
task=timer('StartDelay',0.2,'ExecutionMode','fixedSpacing','Period',0.2, ...
    'TasksToExecute',25,'TimerFcn',@finish,'StopFcn',@(t,~)delete(t));
start(task);
end
function finish(task,~)
ready=evalin('base','exist(''xout'',''var'') && exist(''xdesout'',''var'') && exist(''uout'',''var'')');
if ~ready,return;end
try
    evalin('base','plot_evtol_feedforward(xout,xdesout,uout);');
catch exception
    warning('eVTOL:PlotFailed','%s',exception.message);
end
stop(task);
end
