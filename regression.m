function report = regression(buildSil)
if nargin == 0, buildSil = true; end
root = fileparts(mfilename('fullpath')); cd(root);
if ~exist('results','dir'), mkdir('results'); end
assert(norm(plant([1;0;1;0;0;0],[1;0])) == 0,'Hover equilibrium failed');
if buildSil
    cfg = coder.config('mex'); cfg.TargetLang = 'C'; cfg.GenerateReport = true;
    codegen('-config',cfg,'controller','-args',{zeros(8,1),zeros(2,5)},'-d','build/controller');
end
assert(exist('controller_mex','file') == 3,'Generated controller MEX missing');
names = {'Test_Setpoint','Test_Circle','Test_Disturbance'};
kinds = {'point','circle','point'};
thresholds = [0.01,0.01,0.02];
report = struct([]); passed = true;
for index = 1:3
    mil = simulate(kinds{index},index==3,@controller);
    sil = simulate(kinds{index},index==3,@controller_mex);
    if index == 1
        metric = mil.error(end); silMetric = sil.error(end);
    elseif index == 2
        metric = sqrt(mean(mil.error.^2)); silMetric = sqrt(mean(sil.error.^2));
    else
        metric = max(mil.error(mil.time>=8)); silMetric = max(sil.error(sil.time>=8));
    end
    inputDifference = max(abs(mil.commands-sil.commands),[],1);
    stateDifference = max(abs(mil.states-sil.states),[],'all');
    ok = metric<thresholds(index) && silMetric<thresholds(index) ...
        && all(inputDifference<1e-8) && stateDifference<1e-8;
    report(index).test = names{index};
    report(index).requirement = sprintf('REQ-%02d',index);
    report(index).milMetric = metric; report(index).silMetric = silMetric;
    report(index).inputDifference = inputDifference;
    report(index).stateDifference = stateDifference; report(index).passed = ok;
    fprintf('%s: %s (MiL %.3g, SiL %.3g, input delta %.3g / %.3g)\n', ...
        names{index},stringPass(ok),metric,silMetric,inputDifference);
    passed = passed && ok;
    save(fullfile('results',[names{index} '.mat']),'mil','sil');
    plot_result(mil,sil,kinds{index},names{index});
end
if ~exist('results','dir'), mkdir('results'); end
save('results/report.mat','report');
assert(passed,'Regression failed');
end
function plot_result(mil,sil,kind,name)
desired = zeros(numel(mil.time),2);
for k = 1:numel(mil.time)
    r = trajectory(mil.time(k),kind); desired(k,:) = r(:,1).';
end
fig = figure('Visible','off');
layout = tiledlayout(2,2);
title(layout,strrep(name,'_',' '));
nexttile; plot(desired(:,1),desired(:,2),'--',mil.states(:,1),mil.states(:,3));
axis equal; xlabel('x'); ylabel('y'); legend('Reference','MiL'); grid on;
nexttile; semilogy(mil.time,max(mil.error,eps)); xlabel('Time (s)'); ylabel('Position error'); grid on;
nexttile; plot(mil.time,mil.commands(:,1),sil.time,sil.commands(:,1),'--');
xlabel('Time (s)'); ylabel('u1'); legend('MiL','SiL'); grid on;
nexttile; plot(mil.time,mil.commands(:,2),sil.time,sil.commands(:,2),'--');
xlabel('Time (s)'); ylabel('u2'); legend('MiL','SiL'); grid on;
exportgraphics(fig,fullfile('results',[name '.png'])); close(fig);
end
function label = stringPass(ok)
if ok, label = 'PASS'; else, label = 'FAIL'; end
end
