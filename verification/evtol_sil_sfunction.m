function evtol_sil_sfunction(block)
% Custom host SiL adapter; only the controller executes compiled generated C.
block.NumInputPorts = 2;
block.NumOutputPorts = 2;
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;
for k=1:2
    block.InputPort(k).Dimensions = [5 1];
    block.InputPort(k).DatatypeID = 0;
    block.InputPort(k).Complexity = 'Real';
    block.InputPort(k).DirectFeedthrough = true;
    block.OutputPort(k).Dimensions = 1;
    block.OutputPort(k).DatatypeID = 0;
    block.OutputPort(k).Complexity = 'Real';
end
block.SampleTimes = [0 0];
block.SimStateCompliance = 'HasNoSimState';
block.RegBlockMethod('Outputs',@outputs);
end

function outputs(block)
[u1,u2] = vtol_controller_mex(block.InputPort(1).Data,block.InputPort(2).Data);
block.OutputPort(1).Data = u1;
block.OutputPort(2).Data = u2;
end
