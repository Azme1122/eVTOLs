function codeFolder = build_controller_c(folder)
% Generate only the copied feedforward subsystem, never the complete plant model.
assert(license('test','Real-Time_Workshop'),'Simulink Coder is required.');
name = 'vtol_controller';
if bdIsLoaded(name), error('VTOL:ModelOpen','Close vtol_controller before building.'); end
load_system(fullfile(folder,'eVTOL.slx'));
sourceWorkspace = get_param('eVTOL','ModelWorkspace');
param = getVariable(sourceWorkspace,'param');
assert(param.g==1,'The custom host adapter requires normalized gravity g=1.');
new_system(name);
cleanup = onCleanup(@() close_system(name,0));
add_block('eVTOL/Flatness Feedforward',[name '/Controller'],'Position',[190 50 390 170]);
for k=1:2
    input = sprintf('z%d_d_%s',k,char('x'+k-1));
    add_block('simulink/Sources/In1',[name '/' input],'Port',num2str(k), ...
        'PortDimensions','[5 1]','SampleTime','0.005','Position',[40 40+80*k 70 60+80*k]);
    output = sprintf('u%d_ff',k);
    add_block('simulink/Sinks/Out1',[name '/' output],'Port',num2str(k), ...
        'Position',[460 40+80*k 490 60+80*k]);
    add_line(name,[input '/1'],sprintf('Controller/%d',k));
    add_line(name,sprintf('Controller/%d',k),[output '/1']);
end
assignin(get_param(name,'ModelWorkspace'),'param',param);
set_param(name,'SolverType','Fixed-step','Solver','FixedStepDiscrete','FixedStep','0.005', ...
    'SystemTargetFile','grt.tlc','GenCodeOnly','on','GenerateReport','off', ...
    'DefaultParameterBehavior','Inlined','SupportNonFinite','off','MatFileLogging','off');
build = fullfile(folder,'build','codegen');
if ~isfolder(build), mkdir(build); end
previous = Simulink.fileGenControl('getConfig');
restore = onCleanup(@() Simulink.fileGenControl('setConfig','config',previous));
Simulink.fileGenControl('set','CodeGenFolder',build,'CacheFolder',fullfile(build,'cache'),'createDir',true);
save_system(name,fullfile(build,[name '.slx']));
slbuild(name);
codeFolder = fullfile(build,[name '_grt_rtw']);
source = fileread(fullfile(codeFolder,[name '.c']));
assert(~contains(source,'Six-State Integrator') && ~contains(source,'VTOL Plant'), ...
    'Plant code must not be part of the generated controller.');
assert(isempty(find_system(name,'BlockType','Integrator')),'Code-generation target contains an integrator.');
destination = fullfile(folder,'generated','controller');
if ~isfolder(destination), mkdir(destination); end
copyfile(fullfile(codeFolder,[name '.c']),destination);
copyfile(fullfile(codeFolder,'*.h'),destination);
mexFolder = fullfile(folder,'build','host');
if ~isfolder(mexFolder), mkdir(mexFolder); end
clear vtol_controller_mex
mex('-R2018a',['-I' codeFolder],['-I' fullfile(matlabroot,'simulink','include')], ...
    ['-I' fullfile(matlabroot,'rtw','c','src')], ...
    fullfile(folder,'verification','controller_mex_adapter.c'), ...
    fullfile(codeFolder,[name '.c']),'-outdir',mexFolder,'-output','vtol_controller_mex');
addpath(mexFolder);
end
