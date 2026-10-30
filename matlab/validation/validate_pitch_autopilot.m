function validate_pitch_autopilot()
% Validation of the pitch autopilot gains (P.ctrl, control/pitchGains.m).
%
% 1. Repeats the central finite differences of auv1.m that give the linearised
%    pitch model m_th*theta_dd + d_th*q + k_th*theta = M_delta*delta_s at the
%    design speed U0, checks them against the values stored in auv1Params.m,
%    and cross-checks m_th against M(5,5) and k_th against W*BG_z.
% 2. Closed-loop pitch step at U0: theta_d stepped by 10 deg, the depth loop
%    bypassed, physicsStep + trackingController + actuatorInputs from the
%    level trimmed state without ocean current. Reports rise time, overshoot,
%    settling time and peak fin angle against the design w and zeta, and checks
%    that the fins and the servo slew limit are not saturated. The linearised
%    model with the PID and feedforward and the ideal second-order response are
%    plotted for comparison.
% 3. The same step with M_delta in the plant scaled by 0.75 and 1.25 (stern-
%    plane lift coefficient scaled) and k_th scaled by 0.5 and 1.5 (vertical
%    centre of gravity r_bG(3) scaled), controller unchanged: the feedforward
%    Kff = k_th/M_delta is then off by the scale, and the integral has to clear
%    the residual.
%
% The figure is saved next to this file as validate_pitch_autopilot.png.
%
% Usage: run validate_pitch_autopilot

valDir = fileparts(mfilename('fullpath'));
matlabDir = fileparts(valDir);
oldPath = path;
restorePath = onCleanup(@() path(oldPath));
run(fullfile(matlabDir, 'startup_auv1.m'));

h = 0.05;                                  % Sampling time (s)
P = auv1Params('auv1');
model = P.model;
ctrl = P.ctrl;
U0 = ctrl.U0_theta;

%% 1. Linearised pitch model at U0
fprintf('=== 1. Linearised pitch model at U0 = %.2f m/s ===\n', U0);
lin = linearisePitch(model, U0);
[~,~,M] = auv1(zeros(12,1), model.ui0, 0, 0, 0, model);
W = model.m_target * model.g_mu;
fprintf('Trim (fins neutral): w0 = %.4f m/s, theta0 = %.4f deg, surge acceleration %.4f m/s2\n', ...
    lin.w0, rad2deg(lin.theta0), lin.udot);
fprintf('%-8s %10s %10s   %s\n', 'term', 'finite diff', 'auv1Params', 'unit');
fprintf('%-8s %10.4f %10.4f   kg m2\n', 'm_th', lin.m_th, ctrl.m_theta);
fprintf('%-8s %10.4f %10.4f   N m s/rad\n', 'd_th', lin.d_th, ctrl.d_theta);
fprintf('%-8s %10.4f %10.4f   N m/rad\n', 'k_th', lin.k_th, ctrl.k_theta);
fprintf('%-8s %10.4f %10.4f   N m/rad of delta_s\n', 'M_delta', lin.M_delta, ctrl.M_delta_theta);
fprintf('Cross-check: M(5,5) = %.4f kg m2 (finite difference %+.2f%%)\n', ...
    M(5,5), 100 * (lin.m_th / M(5,5) - 1));
fprintf('Cross-check: W*BG_z = %.4f N m/rad (finite difference %+.2f%%)\n', ...
    W * model.r_bG(3), 100 * (lin.k_th / (W * model.r_bG(3)) - 1));
assert(abs(lin.m_th / M(5,5) - 1) < 0.01, 'm_th differs from M(5,5) by more than 1%%');
assert(abs(lin.k_th / (W * model.r_bG(3)) - 1) < 0.02, 'k_th differs from W*BG_z by more than 2%%');
assert(abs(lin.udot) < 0.01, 'U0 is not a surge equilibrium at the stern command');
ok = abs(lin.m_th / ctrl.m_theta - 1) < 0.005 && abs(lin.d_th / ctrl.d_theta - 1) < 0.005 ...
    && abs(lin.k_th / ctrl.k_theta - 1) < 0.005 && abs(lin.M_delta / ctrl.M_delta_theta - 1) < 0.005;
fprintf('Values in auv1Params.m match the finite differences within 0.5%%  ->  %s\n', passfail(ok));
assert(ok, 'auv1Params.m pitch model values are out of date: re-derive them');

%% 2. Gains
[Kp, Kd, Ki, Kff] = pitchGains(lin.m_th, lin.d_th, lin.k_th, lin.M_delta, ctrl.w_theta, ctrl.zeta_theta);
fprintf('\n=== 2. Gains for w = %.2f rad/s, zeta = %.2f ===\n', ctrl.w_theta, ctrl.zeta_theta);
fprintf('Kp = %.4f, Kd = %.4f, Ki = %.4f, Kff = %.4f (auv1Params.m: %.4f, %.4f, %.4f, %.4f)\n', ...
    Kp, Kd, Ki, Kff, ctrl.Kp_theta, ctrl.Kd_theta, ctrl.Ki_theta, ctrl.Kff_theta);
fprintf('Old gains (w = 0.8, M(5,5) only): Kp = 2.0188, Kd = 4.0376, Ki = 0.1615\n');
assert(abs(Kp / ctrl.Kp_theta - 1) < 0.005 && abs(Kd / ctrl.Kd_theta - 1) < 0.005 ...
    && abs(Kff / ctrl.Kff_theta - 1) < 0.005, ...
    'Gains in auv1Params.m do not follow from the finite differences');

%% 3. Closed-loop 10 deg step at U0
step = deg2rad(10);
t_step = 1;                                % Step time (s)
T = 80;                                    % Run length (s)
fprintf('\n=== 3. Closed-loop theta_d step of %.0f deg at U0 = %.2f m/s ===\n', rad2deg(step), U0);
w = ctrl.w_theta; zeta = ctrl.zeta_theta; wd = w * sqrt(1 - zeta^2);
% Plant cases, controller unchanged: M_delta scaled through the stern-plane lift
% coefficient, k_th through the vertical centre of gravity r_bG(3)
kinds = {'M_delta', 'M_delta', 'M_delta', 'k_th', 'k_th'};
scales = [1 0.75 1.25 0.5 1.5];            % Plant term over the value in auv1Params.m
names = {'Nonlinear (physicsStep)', 'Linearised + PID + FF', 'Ideal 2nd order (w, zeta)'};
cases = cell(size(scales));
for j = 1:numel(scales)
    s = scales(j);
    plant = model;
    scaleM = strcmp(kinds{j}, 'M_delta'); scaleK = strcmp(kinds{j}, 'k_th');
    if scaleM, plant.CL_delta_s = s * model.CL_delta_s; end   % Scales the fin forces, so M_delta
    if scaleK, plant.r_bG(3) = s * model.r_bG(3); end          % Scales W*BG_z, so k_th
    linS = linearisePitch(plant, U0);
    ratio = [linS.m_th / lin.m_th, linS.d_th / lin.d_th, linS.k_th / lin.k_th, linS.M_delta / lin.M_delta];
    expect = [1 1 1 1]; expect(3) = expect(3) * s^scaleK; expect(4) = expect(4) * s^scaleM;
    % auv1.m builds the pitch damping with Dmtrx.m, D(5,5) = M(5,5)*2*zeta5*w5 with
    % w5 = sqrt(W*BG_z/M(5,5)), so scaling BG_z also scales d_th by about sqrt(s)
    if scaleK
        assert(abs(ratio(2) / sqrt(s) - 1) < 0.05, 'd_th ratio %.3f is not close to sqrt(%.2f)', ratio(2), s);
        expect(2) = ratio(2);
    end
    assert(all(abs(ratio ./ expect - 1) < 1e-3), ...
        'Scaling %s by %.2f did not scale only that term (m_th, d_th, k_th, M_delta ratios %s)', ...
        kinds{j}, s, mat2str(ratio, 4));
    cl = closedLoopStep(plant, ctrl, linS, step, t_step, T, h);

    % Linearised model with the PID, and the ideal second-order response
    lm = linearPID(linS, ctrl, step, t_step, T, h);
    tt = max(cl.t - t_step, 0);
    ideal = step * (1 - exp(-zeta*w*tt) .* (cos(wd*tt) + zeta*w/wd * sin(wd*tt)));
    cases{j} = struct('cl', cl, 'lm', lm, 'ideal', ideal);

    fprintf('\n--- Plant %s x %.2f (M_delta %.2f N m/rad, k_th %.3f N m/rad, d_th %.3f N m s/rad), controller unchanged ---\n', ...
        kinds{j}, s, linS.M_delta, linS.k_th, linS.d_th);
    resp = {cl.dtheta, lm, ideal};
    fprintf('%-26s %10s %10s %12s %12s\n', 'response', 'rise 10-90', 'overshoot', 'settle 2%', 'settle 5%');
    for k = 1:3
        m = stepMetrics(cl.t, resp{k}, step, t_step);
        fprintf('%-26s %9.2f s %8.1f %% %10.1f s %10.1f s\n', names{k}, m.rise, m.overshoot, m.settle2, m.settle5);
    end
    fprintf('(settling is measured to the end of the %.0f s run; the integral term sets the slow tail)\n', T);
    kPD = linS.M_delta * (ctrl.Kp_theta + ctrl.Kff_theta) / (linS.k_th + linS.M_delta * ctrl.Kp_theta);
    fprintf('Reached %.1f s after the step: %.1f%% of the step; without the integral the linear model would stop at %.1f%%\n', ...
        T - t_step, 100 * cl.dtheta(end) / step, 100 * kPD);
    [~, iv] = max(cl.dtheta);
    fprintf('Nonlinear pitch 3 s after the step: %.1f%% of the step; peak %.1f%% at %.1f s\n', ...
        100 * interp1(cl.t, cl.dtheta, t_step + 3) / step, 100 * cl.dtheta(iv) / step, cl.t(iv) - t_step);

    peak = max(abs(cl.delta_s));
    peakFin = max(abs(cl.fin_right));
    fprintf('Peak |delta_s| = %.2f deg = %.0f%% of delta_max (%.0f deg), peak fin angle %.2f deg\n', ...
        rad2deg(peak), 100 * peak / model.delta_max, rad2deg(model.delta_max), rad2deg(peakFin));
    fprintf('Largest command change per sample %.2f deg, servo slew limit %.2f deg per sample\n', ...
        rad2deg(cl.maxJump), rad2deg(model.servo_slew * h));
    fprintf('Peak pitch rate %.1f deg/s, speed %.3f to %.3f m/s, max |phi| %.3f deg, max |psi| %.3f deg\n', ...
        rad2deg(max(abs(cl.q))), min(cl.u), max(cl.u), rad2deg(max(abs(cl.phi))), rad2deg(max(abs(cl.psi))));
    fprintf('Fins not saturated (peak < delta_max)  ->  %s\n', passfail(peak < model.delta_max));
    fprintf('Servo slew limit not reached           ->  %s\n', passfail(cl.maxJump < model.servo_slew * h));
    assert(peak < model.delta_max, 'Stern planes saturate in the 10 deg step (%s x %.2f)', kinds{j}, s);
    assert(cl.maxJump < model.servo_slew * h, 'Servo slew limit reached in the 10 deg step (%s x %.2f)', kinds{j}, s);
    assert(all(isfinite(cl.dtheta)), 'Non-finite pitch response (%s x %.2f)', kinds{j}, s);
end
cl = cases{1}.cl; lm = cases{1}.lm; ideal = cases{1}.ideal;

%% Figure
fig = figure('Visible', 'off', 'Position', [100 100 800 800]);
subplot(311)
colours = {'m', 'c', [0.93 0.55 0], [0.2 0.6 0.2]};   % Plant cases 2-5
labels = {'M_\delta \times 0.75', 'M_\delta \times 1.25', 'k_\theta \times 0.5', 'k_\theta \times 1.5'};
plot(cl.t, rad2deg(cl.theta_d), 'k:', cl.t, rad2deg(cl.theta0 + cl.dtheta), 'b', ...
    cl.t, rad2deg(cl.theta0 + lm), 'r--', cl.t, rad2deg(cl.theta0 + ideal), 'g-.'); hold on;
for j = 2:5
    plot(cl.t, rad2deg(cases{j}.cl.theta), 'Color', colours{j-1});
end
hold off;
grid on; ylabel('\theta (deg)');
legend([{'\theta_d', 'Nonlinear', 'Linearised + PID + FF', ...
    sprintf('Ideal 2nd order, \\omega = %.1f rad/s, \\zeta = %.1f', w, zeta)}, ...
    cellfun(@(c) ['Nonlinear, plant ' c], labels, 'UniformOutput', false)], 'Location', 'southeast');
title(sprintf('Pitch step %.0f deg at U_0 = %.2f m/s, K_p = %.3f, K_d = %.3f, K_i = %.3f, K_{ff} = %.3f', ...
    rad2deg(step), U0, Kp, Kd, Ki, Kff));
subplot(312)
plot(cl.t, rad2deg(cl.q), 'b'); hold on;
for j = 2:5
    plot(cl.t, rad2deg(cases{j}.cl.q), 'Color', colours{j-1});
end
hold off;
grid on; ylabel('q (deg/s)');
subplot(313)
plot(cl.t, rad2deg(cl.delta_s), 'b', cl.t, rad2deg(cl.fin_right), 'r--'); hold on;
for j = 2:5
    plot(cl.t, rad2deg(cases{j}.cl.delta_s), 'Color', colours{j-1});
end
yline(rad2deg(model.delta_max), 'k:'); yline(-rad2deg(model.delta_max), 'k:'); hold off;
grid on; ylabel('Fin angle (deg)'); xlabel('Time (s)');
legend([{'\delta_s command', 'Right fin (after slew limit)'}, ...
    cellfun(@(c) ['\delta_s, ' c], labels, 'UniformOutput', false), ...
    {'\pm\delta_{max}'}], 'Location', 'east');
saveas(fig, fullfile(valDir, 'validate_pitch_autopilot.png')); close(fig);
fprintf('\nFigure saved to %s\n', fullfile(valDir, 'validate_pitch_autopilot.png'));
fprintf('PASS: pitch autopilot\n');

end

function lin = linearisePitch(model, U0)
% Central finite differences of auv1.m about the level trimmed state at U0
% (no current, fins neutral, stern thruster at the SIMauv1 command). The
% trim solves wdot = qdot = 0 for the heave velocity w0 and the pitch angle
% theta0 at u = U0. Pitch command steps of +-0.125 give integer PWM offsets
% (+-50 us), so deflection_to_pwm.m does not truncate them.
ui0 = model.ui0;
ui0(5) = model.pwm_centre + 0.5 * model.pwm_range;
state = @(w, th, q) [U0 0 w 0 q 0 0 0 20 0 th 0]';
ui = @(cmd) finInputs(ui0, cmd);

z = [0; 0];                                % [w0; theta0]
for it = 1:30
    r = trimResidual(z);
    J = zeros(2);
    for j = 1:2
        e = zeros(2,1); e(j) = 1e-6;
        J(:,j) = (trimResidual(z + e) - trimResidual(z - e)) / 2e-6;
    end
    dz = -J \ r;
    z = z + dz;
    if norm(dz) < 1e-12, break; end
end
lin.w0 = z(1); lin.theta0 = z(2);
xd0 = auv1(state(lin.w0, lin.theta0, 0), ui(0), 0, 0, 0, model);
lin.udot = xd0(1);

dc = 0.125;                                % Pitch command step
d = dc * model.delta_max;                  % Stern-plane step (rad)
[xp, ~, ~, tp] = auv1(state(lin.w0, lin.theta0, 0), ui(+dc), 0, 0, 0, model);
[xm, ~, ~, tm] = auv1(state(lin.w0, lin.theta0, 0), ui(-dc), 0, 0, 0, model);
lin.M_delta = (tp(5) - tm(5)) / (2 * d);                 % N m/rad of delta_s
lin.m_th = lin.M_delta / ((xp(5) - xm(5)) / (2 * d));    % kg m2

dth = 1e-3; dq = 1e-3;
xp = auv1(state(lin.w0, lin.theta0 + dth, 0), ui(0), 0, 0, 0, model);
xm = auv1(state(lin.w0, lin.theta0 - dth, 0), ui(0), 0, 0, 0, model);
lin.k_th = -lin.m_th * (xp(5) - xm(5)) / (2 * dth);      % N m/rad
xp = auv1(state(lin.w0, lin.theta0, +dq), ui(0), 0, 0, 0, model);
xm = auv1(state(lin.w0, lin.theta0, -dq), ui(0), 0, 0, 0, model);
lin.d_th = -lin.m_th * (xp(5) - xm(5)) / (2 * dq);       % N m s/rad

    function r = trimResidual(z)
        xd = auv1(state(z(1), z(2), 0), ui(0), 0, 0, 0, model);
        r = [xd(3); xd(5)];
    end
end

function ui = finInputs(ui, pitch_cmd)
% Fin PWM for a pitch command through the mixer, other inputs unchanged
[~, deflection] = fin_mixer(pitch_cmd, 0, 0);
ui(1:4) = deflection_to_pwm(deflection);
end

function cl = closedLoopStep(model, ctrl, lin, step, t_step, T, h)
% theta_d step with the sample order of SIMauv1.m, ControlFlag 1, depth loop
% bypassed, heading held at psi_d = 0
clear integralSMCheading;
U0 = ctrl.U0_theta;
n = round(T / h) + 1;
t = (0:n-1)' * h;

act.fin_angle = zeros(4,1);
act.pwm_stern_cmd = model.pwm_centre + 0.5 * model.pwm_range;
act.pwm_stern = act.pwm_stern_cmd;
act.pwm_bow = model.pwm_centre;

x = [U0; 0; lin.w0; 0; 0; 0; 0; 0; 20; 0; lin.theta0; 0];
ref.theta_d = lin.theta0; ref.q_d = 0;
ref.psi_d = 0; ref.r_d = 0; ref.a_d = 0; ref.z_d = 0; ref.zdot_d = 0;
theta_int = 0;

rec = zeros(n, 9);
maxJump = 0;
for i = 1:n
    ref.theta_d = lin.theta0 + step * (t(i) >= t_step);
    y = measurements(x);
    [delta_r, delta_s] = trackingController(y, ref, theta_int, 1, ctrl, model.delta_max, h);
    fin_prev = act.fin_angle(4);
    [ui, ~, act] = actuatorInputs(delta_r, delta_s, act, model, h);
    maxJump = max(maxJump, abs(delta_s - fin_prev));   % Right fin follows delta_s
    rec(i,:) = [ref.theta_d y.theta y.q delta_s act.fin_angle(4) x(1) x(3) x(10) y.psi];
    x = physicsStep(x, ui, 0, 0, 0, model, h);
    theta_int = theta_int + h * ssa(y.theta - ref.theta_d);
end

cl.t = t;
cl.theta0 = lin.theta0;
cl.theta_d = rec(:,1);
cl.theta = rec(:,2);
cl.dtheta = rec(:,2) - lin.theta0;
cl.q = rec(:,3);
cl.delta_s = rec(:,4);
cl.fin_right = rec(:,5);
cl.u = rec(:,6);
cl.phi = rec(:,8);
cl.psi = rec(:,9);
cl.maxJump = maxJump;
end

function dtheta = linearPID(lin, ctrl, step, t_step, T, h)
% Linearised pitch model with the PID and feedforward of trackingController,
% exact zero-order-hold discretisation. State [theta; q; integral of theta
% error]. The feedforward acts on theta_d - theta0 (trim has delta_s = 0 at
% theta0; the offset Kff*theta0 is 0.006 deg of stern plane).
m = lin.m_th; d = lin.d_th; k = lin.k_th; Md = lin.M_delta;
Kp = ctrl.Kp_theta; Kd = ctrl.Kd_theta; Ki = ctrl.Ki_theta; Kff = ctrl.Kff_theta;
A = [0 1 0;
     -(k + Md*Kp)/m, -(d + Md*Kd)/m, -Md*Ki/m;
     1 0 0];
B = [0; Md*(Kp + Kff)/m; -1];              % Input: theta_d - theta0
E = expm([A B; zeros(1,4)] * h);
Ad = E(1:3,1:3); Bd = E(1:3,4);
n = round(T / h) + 1;
t = (0:n-1)' * h;
z = zeros(3,1);
dtheta = zeros(n,1);
for i = 1:n
    dtheta(i) = z(1);
    z = Ad * z + Bd * step * (t(i) >= t_step);
end
end

function m = stepMetrics(t, y, step, t_step)
% Rise time (10-90%), overshoot and settling time (2%, 5% band) of the step
% response y (rad, relative to the start) to a step of size step at t_step.
yn = y / step;
after = t >= t_step;
ta = t(after) - t_step; ya = yn(after);
t10 = crossing(ta, ya, 0.1); t90 = crossing(ta, ya, 0.9);
m.rise = t90 - t10;
m.overshoot = 100 * max(0, max(ya) - 1);
m.settle2 = settling(ta, ya, 0.02);
m.settle5 = settling(ta, ya, 0.05);
end

function tc = crossing(t, y, level)
% First time y reaches level (linear interpolation), NaN if it never does
k = find(y >= level, 1);
if isempty(k), tc = NaN; return; end
if k == 1, tc = t(1); return; end
tc = t(k-1) + (level - y(k-1)) / (y(k) - y(k-1)) * (t(k) - t(k-1));
end

function ts = settling(t, y, band)
% Last time |y - 1| exceeds band, NaN if y is outside the band at the end
out = find(abs(y - 1) > band);
if isempty(out), ts = 0; return; end
if out(end) == numel(t), ts = NaN; return; end
ts = t(out(end) + 1);
end

function s = passfail(ok)
if ok, s = 'PASS'; else, s = 'FAIL'; end
end
