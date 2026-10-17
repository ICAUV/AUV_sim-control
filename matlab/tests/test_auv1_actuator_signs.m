function test_auv1_actuator_signs()
% Checks the direction of the AUV1 actuator forces and moments at u = 1 m/s,
% level, with commands passed through fin_mixer.m and deflection_to_pwm.m:
%
%   +yaw_cmd         -> N > 0   (heading increasing, turn to starboard)
%   +pitch_cmd       -> M > 0   (nose up)
%   stern PWM > 1500 -> X > 0
%   bow PWM > 1500   -> Y > 0 and N > 0
%
% It also checks that the controller-to-command mapping used in SIMauv1 gives
% the same sign of yaw and pitch moment as the REMUS rudder and stern planes,
% and reports (does not assert) the roll moment K for roll_cmd = +1 with each
% fin's contribution.
%
% Usage (from anywhere): run test_auv1_actuator_signs

testsDir = fileparts(mfilename('fullpath'));
matlabDir = fileparts(testsDir);
oldPath = path;
restorePath = onCleanup(@() path(oldPath));
run(fullfile(matlabDir, 'startup_auv1.m'));

P = auv1Params('auv1');
m = P.model;
x = zeros(12,1); x(1) = 1;                  % Level, u = 1 m/s
act = @(ui) actuatorForces(x, ui, m);       % Generalized actuator forces [X Y Z K M N]'
pwmOf = @(d) deflection_to_pwm(d);

% Neutral inputs give no actuator force at all
tau0 = act(m.ui0);
assert(all(abs(tau0) < 1e-12), 'Neutral PWM gives nonzero actuator force');

% Yaw command through the mixer
[~, defl] = fin_mixer(0, 0, 0.5);
ui = m.ui0; ui(1:4) = pwmOf(defl);
tau = act(ui);
fprintf('+yaw_cmd 0.5   : fins %s, N = %+.4f Nm (Y = %+.4f N, M = %+.4f Nm)\n', ...
    mat2str(ui(1:4)'), tau(6), tau(2), tau(5));
assert(tau(6) > 0, '+yaw_cmd must give N > 0');

% Pitch command through the mixer
[~, defl] = fin_mixer(0.5, 0, 0);
ui = m.ui0; ui(1:4) = pwmOf(defl);
tau = act(ui);
fprintf('+pitch_cmd 0.5 : fins %s, M = %+.4f Nm (Z = %+.4f N, N = %+.4f Nm)\n', ...
    mat2str(ui(1:4)'), tau(5), tau(3), tau(6));
assert(tau(5) > 0, '+pitch_cmd must give M > 0');

% Stern thruster
ui = m.ui0; ui(5) = 1700;
tau = act(ui);
fprintf('stern 1700 us  : X = %+.4f N\n', tau(1));
assert(tau(1) > 0, 'Stern PWM > 1500 must give X > 0');

% Bow thruster
ui = m.ui0; ui(6) = 1700;
tau = act(ui);
fprintf('bow 1700 us    : Y = %+.4f N, N = %+.4f Nm\n', tau(2), tau(6));
assert(tau(2) > 0 && tau(6) > 0, 'Bow PWM > 1500 must give Y > 0 and N > 0');

% Same sign of yaw and pitch moment as the REMUS actuators, using the SIMauv1
% mapping yaw_cmd = delta_r/delta_max, pitch_cmd = delta_s/delta_max
R = auv1Params('remus100').model;
delta = deg2rad(10);                        % Controller outputs delta_r, delta_s
[~,~,~,tauR] = auv1(x, [ delta 0 0]', 0, 0, 0, R);       % Rudder
assert(tauR(6) > 0, 'REMUS rudder reference should give N > 0');
[~, defl] = fin_mixer(0, 0, delta / m.delta_max);
ui = m.ui0; ui(1:4) = pwmOf(defl);
tau = act(ui);
assert(tau(6) > 0, 'AUV1 yaw sign differs from REMUS');
[~,~,~,tauR] = auv1(x, [0 -delta 0]', 0, 0, 0, R);       % Stern plane, ui(2) = -delta_s
assert(tauR(5) > 0, 'REMUS stern-plane reference should give M > 0');
[~, defl] = fin_mixer(delta / m.delta_max, 0, 0);
ui = m.ui0; ui(1:4) = pwmOf(defl);
tau = act(ui);
assert(tau(5) > 0, 'AUV1 pitch sign differs from REMUS');

% Roll: reported, not asserted. All four fins get the same deflection.
[~, defl] = fin_mixer(0, 1, 0);
ui = m.ui0; ui(1:4) = pwmOf(defl);
tauAll = act(ui);
names = {'top', 'bottom', 'left', 'right'};
fprintf('\nroll_cmd = +1 : fin PWM %s (deflections %s)\n', mat2str(ui(1:4)'), mat2str(defl'));
Ki = zeros(4,1);
for i = 1:4
    u1 = m.ui0; u1(i) = ui(i);
    t1 = act(u1);
    Ki(i) = t1(4);
    fprintf('  %-6s fin alone: K = %+.4f Nm\n', names{i}, Ki(i));
end
fprintf('  all four fins : K = %+.4f Nm\n', tauAll(4));
fprintf('  top + bottom  : K = %+.4f Nm, left + right: K = %+.4f Nm\n', ...
    Ki(1) + Ki(2), Ki(3) + Ki(4));
fprintf('  (the roll column of fin_mixer.SIGNS gives the same deflection on all four fins;\n');
fprintf('   whether the contributions add or cancel depends on the per-fin force signs)\n\n');

fprintf('PASS: auv1 actuator signs\n');
end

function tau = actuatorForces(x, ui, model)
[~,~,~,tau] = auv1(x, ui, 0, 0, 0, model);
end
