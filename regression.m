function report = regression
% Full local regression: requirement, controller-only C, custom host SiL and B2B.
folder = fileparts(mfilename('fullpath'));
previous = pwd; cd(folder);
restoreFolder = onCleanup(@() cd(previous));
addpath(fullfile(folder,'verification'));
resultFolder = fullfile(folder,'results');
if ~isfolder(resultFolder), mkdir(resultFolder); end
report = struct('test',{},'metric',{},'limit',{},'passed',{});
try
    load_system(fullfile(folder,'eVTOL.slx'));
    assert(strcmp(get_param('eVTOL','Dirty'),'off'), ...
        'Save or discard changes to eVTOL before running regression.');
    % Validate the saved model in copies, preserving the user's main model.
    verify_evtol_requirements;
    report(end+1) = item('REQ-01 linked unit test',0,1,true);
    build_controller_c(folder);
    report(end+1) = item('Controller-only C generation',0,1,true);
    [u1,u2] = vtol_controller_mex([1;0;0;0;0],[1;0;0;0;0]);
    hover = max(abs([u1-1,u2]));
    report(end+1) = item('Generated controller hover',hover,1e-12,hover<1e-12);
    try
        [~,~] = vtol_controller_mex(zeros(5,1),[0;0;-1;0;0]);
        rejected = false;
    catch exception
        rejected = strcmp(exception.identifier,'VTOL:Singular');
    end
    report(end+1) = item('Singularity guard',double(~rejected),1,rejected);
    milModel = 'eVTOL_mil_test'; silModel = 'eVTOL_sil_test';
    assert(~bdIsLoaded(milModel) && ~bdIsLoaded(silModel),'Close temporary test models before regression.');
    build = fullfile(folder,'build','harnesses');
    if ~isfolder(build), mkdir(build); end
    copyfile(fullfile(folder,'eVTOL.slx'),fullfile(build,[milModel '.slx']));
    copyfile(fullfile(folder,'eVTOL.slx'),fullfile(build,[silModel '.slx']));
    load_system(fullfile(build,[milModel '.slx']));
    load_system(fullfile(build,[silModel '.slx']));
    cleanup = onCleanup(@() closeHarnesses(milModel,silModel));
    for model={milModel,silModel}
        set_param(model{1},'StopFcn','','ReturnWorkspaceOutputs','on');
    end
    replaceController(silModel);
    save_system(silModel);
    root=sfroot;
    milPlant=root.find('-isa','Stateflow.EMChart','Path',[milModel '/VTOL Plant/Nonlinear Equations']);
    silPlant=root.find('-isa','Stateflow.EMChart','Path',[silModel '/VTOL Plant/Nonlinear Equations']);
    assert(~isempty(milPlant) && strcmp(milPlant.Script,silPlant.Script),'Plant implementations differ.');
    reqset=slreq.load(fullfile(folder,'eVTOL_requirements.slreqx'));
    req=find(reqset,'Id','REQ-01'); limit=str2double(getAttribute(req,'PositionErrorLimit'));
    tolerance=1e-10;
    names={'Point','Circle'};
    for mode=1:2
        for model={milModel,silModel}
            set_param([model{1} '/Trajectory Mode'],'Value',num2str(mode));
        end
        block='Transition Time'; if mode==2, block='Circle Period'; end
        duration=str2double(get_param([milModel '/Trajectory Generator/' block],'Value'));
        mil=readResult(sim(milModel,'StopTime',num2str(duration,17)));
        sil=readResult(sim(silModel,'StopTime',num2str(duration,17)));
        assert(isequal(mil.time,sil.time),'MiL/SiL time grids differ.');
        assert(max(abs(mil.reference-sil.reference),[],'all')<1e-12,'MiL/SiL references differ.');
        assert(isequal(mil.state(1,:),sil.state(1,:)),'MiL/SiL initial states differ.');
        for k=1:2
            delta=max(abs(mil.control(:,k)-sil.control(:,k)));
            report(end+1)=item(sprintf('%s u%d MiL/SiL',names{mode},k),delta,tolerance,delta<tolerance);
        end
        delta=max(abs(mil.state-sil.state),[],'all');
        report(end+1)=item([names{mode} ' state MiL/SiL'],delta,tolerance,delta<tolerance);
        for execution={'mil','sil'}
            result=mil; if strcmp(execution{1},'sil'),result=sil;end
            metric=max(sqrt(sum((result.state(:,[1 3])-result.reference(:,1:2)).^2,2)));
            report(end+1)=item([names{mode} ' ' execution{1} ' REQ-01'],metric,limit,metric<limit);
        end
        save(fullfile(resultFolder,[names{mode} '_mil_sil.mat']),'mil','sil');
        plotComparison(mil,sil,fullfile(resultFolder,[names{mode} '_mil_sil.png']),names{mode});
    end
catch exception
    report(end+1)=item(['ERROR: ' exception.identifier],NaN,NaN,false);
    writeReport(report,resultFolder);
    rethrow(exception);
end
writeReport(report,resultFolder);
assert(all([report.passed]),'VTOL:RegressionFailed','One or more regression tests failed.');
fprintf('PASS: all %d regression checks. Custom host SiL; no Embedded Coder.\n',numel(report));
end

function value=item(name,metric,limit,passed)
value=struct('test',name,'metric',metric,'limit',limit,'passed',passed);
end

function writeReport(report,folder)
for k=1:numel(report)
    label='FAIL';if report(k).passed,label='PASS';end
    fprintf('%s: %s (metric %.3g, limit %.3g)\n',label,report(k).test,report(k).metric,report(k).limit);
end
writetable(struct2table(report),fullfile(folder,'regression.csv'));
save(fullfile(folder,'regression.mat'),'report');
document=com.mathworks.xml.XMLUtils.createDocument('testsuites');
suite=document.createElement('testsuite');
suite.setAttribute('name','eVTOL regression');
suite.setAttribute('tests',num2str(numel(report)));
suite.setAttribute('failures',num2str(sum(~[report.passed])));
document.getDocumentElement.appendChild(suite);
for k=1:numel(report)
    test=document.createElement('testcase');
    test.setAttribute('name',report(k).test);
    test.setAttribute('classname','eVTOL.regression');
    if ~report(k).passed
        failure=document.createElement('failure');
        failure.setAttribute('message',sprintf('Metric %.17g, limit %.17g',report(k).metric,report(k).limit));
        test.appendChild(failure);
    end
    suite.appendChild(test);
end
xmlwrite(fullfile(folder,'regression.xml'),document);
end

function replaceController(model)
sub=[model '/Flatness Feedforward'];
for k=1:2
    delete_line(model,sprintf('Trajectory Generator/%d',k),sprintf('Flatness Feedforward/%d',k));
    delete_line(model,sprintf('Flatness Feedforward/%d',k),sprintf('Control Inputs/%d',k));
end
Simulink.SubSystem.deleteContents(sub);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',[sub '/Generated C'], ...
    'FunctionName','evtol_sil_sfunction','Position',[160 70 330 160]);
for k=1:2
    input=sprintf('z%d_d_%s',k,char('x'+k-1)); output=sprintf('u%d_ff',k);
    add_block('simulink/Sources/In1',[sub '/' input],'Port',num2str(k),'Position',[30 40+80*k 60 60+80*k]);
    add_block('simulink/Sinks/Out1',[sub '/' output],'Port',num2str(k),'Position',[410 40+80*k 440 60+80*k]);
    add_line(sub,[input '/1'],sprintf('Generated C/%d',k));
    add_line(sub,sprintf('Generated C/%d',k),[output '/1']);
end
% Restore only controller connections; leave reference-log branches intact.
for k=1:2
    add_line(model,sprintf('Trajectory Generator/%d',k),sprintf('Flatness Feedforward/%d',k),'autorouting','on');
    add_line(model,sprintf('Flatness Feedforward/%d',k),sprintf('Control Inputs/%d',k),'autorouting','on');
end
end

function result=readResult(output)
result=struct('time',output.xout.Time,'state',normalize(output.xout,6), ...
    'reference',normalize(output.xdesout,10),'control',normalize(output.uout,2));
end

function values=normalize(signal,width)
if ismatrix(signal.Data) && size(signal.Data,2)==width,values=signal.Data;
else,values=reshape(signal.Data,width,[]).';end
assert(all(isfinite(values),'all'),'VTOL:Nonfinite','Simulation results are not finite.');
end

function closeHarnesses(varargin)
for k=1:nargin,if bdIsLoaded(varargin{k}),close_system(varargin{k},0);end,end
end

function plotComparison(mil,sil,path,name)
fig=figure('Visible','off','Color','white','Position',[100 100 1000 700]);
cleanup=onCleanup(@() close(fig));
layout=tiledlayout(fig,2,2); title(layout,[name ' MiL / custom host SiL'],'Color','black');
for k=1:2
    ax=nexttile(layout);plot(ax,mil.time,mil.control(:,k),'b-',sil.time,sil.control(:,k),'r--');
    ylabel(ax,sprintf('u%d',k));xlabel(ax,'Normalized time');legend(ax,'MiL','SiL');grid(ax,'on');
    set(ax,'Color','white','XColor','black','YColor','black');
end
ax=nexttile(layout);plot(ax,mil.time,abs(mil.control-sil.control));
xlabel(ax,'Normalized time');ylabel(ax,'Absolute input difference');legend(ax,'u1','u2');grid(ax,'on');
set(ax,'Color','white','XColor','black','YColor','black');
ax=nexttile(layout);plot(ax,mil.reference(:,1),mil.reference(:,2),'k-', ...
    mil.state(:,1),mil.state(:,3),'b--',sil.state(:,1),sil.state(:,3),'r:');
axis(ax,'equal');xlabel(ax,'x');ylabel(ax,'y');legend(ax,'Desired','MiL','SiL');grid(ax,'on');
set(ax,'Color','white','XColor','black','YColor','black');
set(findall(fig,'Type','Legend'),'Color','white','TextColor','black');
exportgraphics(fig,path);
end
