function test_auv1_matches_remus100()
% Checks that auv1 reproduces the unmodified MSS remus100 (the reference in
% tests/reference) for random states and inputs, Euler (12-state) and
% quaternion (13-state), with and without ocean currents, and for the mass
% matrix returned by the empty call. The reference folder is only on the path
% while this test runs.
%
% Usage (from anywhere): run test_auv1_matches_remus100

tol = 1e-12;
nSamples = 20;

testsDir = fileparts(mfilename('fullpath'));
matlabDir = fileparts(testsDir);
refDir = fullfile(testsDir, 'reference');

oldPath = path;
restorePath = onCleanup(@() path(oldPath)); % Remove the reference path again
run(fullfile(matlabDir, 'startup_auv1.m'));
addpath(refDir);

% Make sure we are comparing two different files
assert(strcmp(fileparts(which('auv1')), matlabDir), 'auv1 not resolved to matlab/');
assert(strcmp(fileparts(which('remus100')), refDir), 'remus100 not resolved to the reference');

rng(1);
maxErr = 0;
P = auv1Params('remus100'); % Explicit REMUS 100 parameter set; auv1 defaults to AUV1

% Mass matrix from the zero-state call (what the empty call returns, but with
% the REMUS 100 parameter set)
[~,~,M_auv] = auv1(zeros(12,1), zeros(3,1), 0, 0, 0, P.model);
[~,~,M_ref] = remus100();
assert(isequal(size(M_auv), [6 6]), 'M has wrong size');
assert(all(isfinite(M_auv(:))), 'M contains non-finite values');
maxErr = max(maxErr, max(abs(M_auv(:) - M_ref(:))));
assert(max(abs(M_auv(:) - M_ref(:))) <= tol, 'Mass matrix M differs');

for k = 1:nSamples
    nu  = [2*rand; 0.3*randn; 0.3*randn; 0.2*randn; 0.2*randn; 0.2*randn];
    pos = 100*randn(3,1);
    ang = [pi*(2*rand-1); (pi/3)*(2*rand-1); pi*(2*rand-1)]; % theta away from +-90 deg
    q = randn(4,1); q = q/norm(q);                           % Random unit quaternion

    states = {[nu; pos; ang], [nu; pos; q]};    % 12-state Euler, 13-state quaternion
    ui = [0.5*(2*rand-1); 0.5*(2*rand-1); 1700*rand]; % Exceeds saturation limits sometimes
    Vc = rand; betaVc = pi*(2*rand-1); w_c = 0.2*(2*rand-1);

    for s = 1:numel(states)
        x = states{s};

        % No currents
        [xdot_a, U_a] = auv1(x, ui, 0, 0, 0, P.model);
        [xdot_r, U_r] = remus100(x, ui);
        maxErr = max(maxErr, checkMatch(xdot_a, xdot_r, U_a, U_r, tol, k, s, 'no currents'));

        % 3-D currents
        [xdot_a, U_a] = auv1(x, ui, Vc, betaVc, w_c, P.model);
        [xdot_r, U_r] = remus100(x, ui, Vc, betaVc, w_c);
        maxErr = max(maxErr, checkMatch(xdot_a, xdot_r, U_a, U_r, tol, k, s, 'currents'));
    end
end

fprintf('PASS: auv1 matches remus100 (max abs difference %.3g, tol %.1g)\n', maxErr, tol);
end

function err = checkMatch(xdot_a, xdot_r, U_a, U_r, tol, k, s, label)
assert(all(isfinite(xdot_a)), 'Non-finite xdot (sample %d, state type %d, %s)', k, s, label);
err = max([abs(xdot_a(:) - xdot_r(:)); abs(U_a - U_r)]);
assert(err <= tol, 'auv1 and remus100 differ by %.3g (sample %d, state type %d, %s)', ...
    err, k, s, label);
end
