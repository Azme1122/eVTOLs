function tests = test_evtol_position
tests = functiontests(localfunctions);
end

function testPositionTracking(testCase)
report = verify_evtol_requirements(true);
verifyLessThan(testCase,report.MaximumPositionError,report.Limit, ...
    'Actual-to-desired position distance exceeded PositionErrorLimit.');
end
