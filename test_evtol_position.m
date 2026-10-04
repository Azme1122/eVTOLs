function tests = test_evtol_position
tests = functiontests(localfunctions);
end

function testPositionTracking(testCase)
testFolder=char(java.io.File(fileparts(mfilename('fullpath'))).getCanonicalPath());
runnerFolder=char(java.io.File(fileparts(which('verify_evtol_requirements'))).getCanonicalPath());
assertEqual(testCase,testFolder,runnerFolder,'Test and runner must belong to the same checkout.');
report = verify_evtol_requirements(true);
verifyLessThan(testCase,report.MaximumPositionError,report.Limit, ...
    'Actual-to-desired position distance exceeded PositionErrorLimit.');
end
