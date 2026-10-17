function test_SIMauv1_regression(regenerate)
% Checks that SIMauv1 reproduces the stored baseline for every control method
% (ControlFlag 1-4) and kinematic representation (Euler angles, unit
% quaternions). The baseline keeps every 100th logged sample of each run, and
% the path the runs followed, so editing pathParams.m does not affect the test.
% The random ocean current of ControlFlag 4 is seeded, so the runs are
% repeatable. Takes a few minutes.
%
% Usage (from anywhere):
%   run test_SIMauv1_regression         : Compare SIMauv1 with the baseline
%   test_SIMauv1_regression(true)       : Rewrite the baseline with the current
%                                         pathParams(), only after an intended
%                                         change in behaviour

if nargin < 1, regenerate = false; end
tol = 1e-6;                     % Absolute; same machine and version gives 0
step = 100;                     % Keep every step-th sample

testsDir = fileparts(mfilename('fullpath'));
run(fullfile(fileparts(testsDir), 'startup_auv1.m'));
file = fullfile(testsDir, 'reference', 'SIMauv1_baseline.mat');
closeAll = onCleanup(@() delete(findall(groot, 'Type', 'figure')));

if regenerate
    path = pathParams();
else
    base = load(file);
    assert(base.step == step, 'Baseline was stored with a different step');
    path = base.path;
end

runs = struct();
for ControlFlag = 1:4
    for KinematicsFlag = 1:2
        rng(1, 'twister');
        [~, out] = evalc('SIMauv1(ControlFlag, KinematicsFlag, path)'); % Hides the waypoint log
        k = 1:step:numel(out.t);
        runs.(runName(ControlFlag, KinematicsFlag)) = logged(out, k);
    end
end

if regenerate
    save(file, 'runs', 'step', 'path');
    fprintf('Baseline written to %s\n', file);
    return
end

maxErr = 0;
for name = fieldnames(base.runs)'
    for group = fieldnames(base.runs.(name{1}))'
        a = runs.(name{1}).(group{1});
        b = base.runs.(name{1}).(group{1});
        assert(isequal(size(a), size(b)) && isequal(isnan(a), isnan(b)), ...
            'Run %s: %s has a different size or NaN pattern', name{1}, group{1});
        ok = ~isnan(a);
        err = max([0; abs(a(ok) - b(ok))]);
        assert(err <= tol, 'Run %s: %s differs by %.3g', name{1}, group{1}, err);
        maxErr = max(maxErr, err);
    end
end

fprintf('PASS: SIMauv1 matches the baseline (max abs difference %.3g, tol %.1g)\n', ...
    maxErr, tol);
end

function name = runName(ControlFlag, KinematicsFlag)
name = sprintf('control%d_kinematics%d', ControlFlag, KinematicsFlag);
end

function d = logged(out, k)
% Logged results at samples k, grouped as in the SIMauv1 output
d.t = out.t(k);
d.eta = out.eta(k,:);
d.nu = out.nu(k,:);
d.ui = out.ui(k,:);
d.ref = columns(out.ref, k);
d.current = columns(out.current, k);
d.alos = columns(out.alos, k);
end

function M = columns(s, k)
M = cell2mat(struct2cell(s)');
M = M(k,:);
end
