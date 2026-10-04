function report = verify_evtol_requirements(simulationOnly)
% Run the linked unit test, or measure both paths for that test.
if nargin==0, simulationOnly=false; end
folder = fileparts(mfilename('fullpath'));
model = 'eVTOL';
load_system(fullfile(folder,[model '.slx']));
cleanup = onCleanup(@() close_system(model,0));
set_param(model,'StopFcn','','ReturnWorkspaceOutputs','on');
reqfile = fullfile(folder,'eVTOL_requirements.slreqx');
if isfile(reqfile)
    reqset = slreq.load(reqfile);
else
    reqset = slreq.new(reqfile);
end
req = find(reqset,'Id','REQ-01');
if isempty(req), req = add(reqset,'Id','REQ-01'); end
if ~ismember('PositionErrorLimit',reqset.CustomAttributeNames)
    addAttribute(reqset,'PositionErrorLimit','Edit','Description', ...
        'Maximum allowed Euclidean position error (dimensionless). Strictly less than this value.');
    setAttribute(req,'PositionErrorLimit','0.01');
end
limit = str2double(getAttribute(req,'PositionErrorLimit'));
assert(isscalar(limit) && isfinite(limit) && limit>0,'PositionErrorLimit must be a positive number.');
if ~simulationOnly
% Older versions linked this runner as though it were a unit test.
oldLinkFile = fullfile(folder,'verify_evtol_requirements~m.slmx');
if isfile(oldLinkFile)
    oldLinkSet = slreq.load(oldLinkFile);
    obsolete = getLinks(oldLinkSet);
    for k=1:numel(obsolete)
        if strcmp(obsolete(k).Type,'Verify')
            remove(obsolete(k));
        end
    end
    save(oldLinkSet);
end
req.Summary = sprintf('Position error must stay below %.3g',limit);
req.Description = sprintf(['With matched initial states and no disturbances, ' ...
    'sqrt((x-x_desired)^2+(y-y_desired)^2) must be strictly less than %.3g at every logged simulation time. ' ...
    'Check both point-to-point motion and one complete circle. ' ...
    'Change PositionErrorLimit in Attributes to change the test threshold. ' ...
    'Quantities are dimensionless.'],limit);
save(reqset);
file = fullfile(folder,'test_evtol_position.m');
testLinkFile = fullfile(folder,'test_evtol_position~m.slmx');
if isfile(testLinkFile), slreq.load(file); end
lines = splitlines(string(fileread(file)));
line = find(startsWith(strtrim(lines),'function testPositionTracking'),1);
range = slreq.getTextRange(file,line);
if isempty(range), range = slreq.createTextRange(file,line); end
if isempty(getLinks(range))
    link = slreq.createLink(range,req);
    link.Type = 'Verify';
    save(linkSet(link));
end
status = runTests(reqset);
disp(status);
assert(status.passed==1 && status.failed==0,'Linked requirements verification did not pass.');
saved = load(fullfile(folder,'results','requirements_results.mat'),'report');
report = saved.report;
return
end
data = cell(1,2);
metrics = zeros(2,1);
for mode=1:2
    set_param([model '/Trajectory Mode'],'Value',num2str(mode));
    stop = str2double(get_param([model '/Trajectory Generator/Transition Time'],'Value'));
    if mode==2
        stop = str2double(get_param([model '/Trajectory Generator/Circle Period'],'Value'));
    end
    assert(isfinite(stop) && stop>0,'Trajectory duration must be positive.');
    output = sim(model,'StopTime',num2str(stop,17));
    state = normalize(output.xout,6);
    reference = normalize(output.xdesout,10);
    data{mode} = struct('state',state,'reference',reference,'time',output.xout.Time);
    metrics(mode) = Test_PositionError(data{mode});
end
report = table(["REQ-01";"REQ-01"],["Point-to-point";"Circle"],metrics, ...
    repmat(limit,2,1),metrics<limit, ...
    'VariableNames',{'Requirement','Trajectory','MaximumPositionError','Limit','Pass'});
disp(report);
if ~isfolder(fullfile(folder,'results')), mkdir(fullfile(folder,'results')); end
writetable(report,fullfile(folder,'results','requirements_results.csv'));
save(fullfile(folder,'results','requirements_results.mat'),'report','data');
end

function metric = Test_PositionError(data)
error = data.state(:,[1 3])-data.reference(:,1:2);
metric = max(sqrt(sum(error.^2,2)));
end

function values = normalize(signal,width)
if ismatrix(signal.Data) && size(signal.Data,2)==width
    values = signal.Data;
else
    values = reshape(signal.Data,width,[]).';
end
assert(all(isfinite(values),'all'),'Nonfinite simulation data.');
end
