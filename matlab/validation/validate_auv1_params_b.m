function validate_auv1_params_b()
% Validation of the AUV1 parameter set, pass B (actuators: T500 stern thruster,
% four independent fins, T200 bow thruster).
%
% Runs the equivalence test, the actuator sign test, TG5 cases 2-4 (top speed
% against thrust, turning circle, stern-plane step), regression checks of the
% pass A results with neutral PWM, and SIMauv1 end to end. Plots and a text
% log are saved next to this file.
%
% Usage: run validate_auv1_params_b

valDir = fileparts(mfilename('fullpath'));
matlabDir = fileparts(valDir);
oldPath = path;
restorePath = onCleanup(@() path(oldPath));
run(fullfile(matlabDir, 'startup_auv1.m'));
addpath(fullfile(matlabDir, 'tests'));

logFile = fullfile(valDir, 'validation_results_pass_b.txt');
if exist(logFile, 'file'), delete(logFile); end
diary(logFile);
closeDiary = onCleanup(@() diary('off'));

h = 0.05;                                  % Sampling time (s)
P = auv1Params('auv1');
model = P.model;
rho_mss = mssConstants().rho_water;        % Density used by the MSS hull drag

%% 0. Thrust tables
fprintf('=== 0. Thrust tables (14.8 V, bollard) ===\n');
fprintf('T500: F(1500) = %.3f N, F(1600) = %.3f N, F(1700) = %.3f N, F(1900) = %.3f N, F(1100) = %.3f N\n', ...
    tbl(model.T500, 1500), tbl(model.T500, 1600), tbl(model.T500, 1700), tbl(model.T500, 1900), tbl(model.T500, 1100));
fprintf('T200: F(1500) = %.3f N, F(1700) = %.3f N, F(1900) = %.3f N, F(1100) = %.3f N\n', ...
    tbl(model.T200, 1500), tbl(model.T200, 1700), tbl(model.T200, 1900), tbl(model.T200, 1100));
fig = figure('Visible', 'off', 'Position', [100 100 700 450]);
plot(model.T500.pwm, model.T500.force, model.T200.pwm, model.T200.force); grid on;
xlabel('PWM (\mus)'); ylabel('Force (N)'); legend('T500 (stern)', 'T200 (bow)', 'Location', 'northwest');
title('Thrust tables at 14.8 V (bollard)');
saveas(fig, fullfile(valDir, 'thrust_tables_14v8.png')); close(fig);

%% 1. Equivalence test (REMUS 100 parameter set and actuators)
fprintf('\n=== 1. Equivalence test ===\n');
test_auv1_matches_remus100();

%% 2. Actuator sign test
fprintf('\n=== 2. Actuator sign test ===\n');
test_auv1_actuator_signs();

%% 3. Regression of pass A results with neutral PWM
fprintf('\n=== 3. Pass A regression, neutral PWM ===\n');
m0 = model; m0.dB = 0;
[~, X] = simulate(m0, zeros(12,1), m0.ui0, 60, h);
maxVel = max(abs(X(:,1:6)), [], 'all');
fprintf('At rest, dB = 0, 60 s: max |velocity| = %.3g (limit 1e-6)  ->  %s\n', maxVel, passfail(maxVel < 1e-6));
x0 = zeros(12,1); x0(9) = 20;
[~, X] = simulate(model, x0, model.ui0, 120, h);
riseSpeed = -(X(end,9) - X(end-round(10/h),9)) / 10;
fprintf('At rest, dB = 2 N: rise speed %.4f m/s (pass A: 0.1128)  ->  %s\n', riseSpeed, passfail(abs(riseSpeed - 0.1128) < 5e-4));

%% 4. TG5 case 2: top speed against thrust
fprintf('\n=== 4. TG5 case 2: straight runs, stern PWM 1600-1900 ===\n');
xl = zeros(12,1); xl(1) = 0.1;
pwmList = [1600 1700 1800 1900];
Uss = zeros(size(pwmList)); Uhand = Uss; Thr = Uss;
fprintf('Hand calc: T = 0.5*rho*Cd*A*U^2 with the model hull drag (rho = %d as used inside the MSS drag helper, Cd = %.2f, A = pi*b^2 = %.5f m2)\n', ...
    rho_mss, model.Cd, pi*model.b^2);
fprintf('%8s %10s %12s %12s %10s\n', 'PWM us', 'T500 (N)', 'U sim (m/s)', 'U hand (m/s)', 'sim/hand');
figure_t = cell(numel(pwmList),1);
for k = 1:numel(pwmList)
    ui = model.ui0; ui(5) = pwmList(k);
    [t, X] = simulate(model, xl, ui, 300, h);
    Uss(k) = X(end,1);
    Thr(k) = tbl(model.T500, pwmList(k));
    Uhand(k) = sqrt(2*Thr(k) / (rho_mss * model.Cd * pi * model.b^2));
    figure_t{k} = [t(:), X(:,1)];
    fprintf('%8d %10.2f %12.3f %12.3f %10.3f   (dU/dt at end %.1e m/s2, pitch %.2f deg)\n', ...
        pwmList(k), Thr(k), Uss(k), Uhand(k), Uss(k)/Uhand(k), (X(end,1)-X(end-1,1))/h, rad2deg(X(end,11)));
end
fprintf('Bollard thrust is used at every speed (no loss with forward speed), so these speeds are an upper bound.\n');
fig = figure('Visible', 'off', 'Position', [100 100 700 600]);
subplot(2,1,1); hold on;
for k = 1:numel(pwmList), plot(figure_t{k}(:,1), figure_t{k}(:,2)); end
grid on; xlabel('Time (s)'); ylabel('Surge speed (m/s)'); title('TG5 case 2: straight runs');
legend(arrayfun(@(p) sprintf('%d \\mus', p), pwmList, 'UniformOutput', false), 'Location', 'southeast');
subplot(2,1,2); plot(pwmList, Uss, 'o-', pwmList, Uhand, 's--'); grid on;
xlabel('Stern PWM (\mus)'); ylabel('Steady speed (m/s)'); legend('Simulated', 'Hand calc', 'Location', 'northwest');
saveas(fig, fullfile(valDir, 'tg5_case2_top_speed.png')); close(fig);

%% 5. TG5 case 3: turning circle at 1 m/s, 20 deg rudder
fprintf('\n=== 5. TG5 case 3: turning circle ===\n');
steady = @(pwm) steadySpeed(model, pwm, h);
pwm1 = fzero(@(p) steady(p) - 1.0, [1520 1700], optimset('TolX', 0.01));
fprintf('Stern PWM for 1.00 m/s straight: %.1f us (T500 force %.2f N)\n', pwm1, tbl(model.T500, pwm1));
x1 = zeros(12,1); x1(1) = 1.0;
ui = model.ui0; ui(5) = pwm1;
delta_cmd = deg2rad(20);
[~, defl] = fin_mixer(0, 0, delta_cmd / model.delta_max);   % yaw_cmd = 20 deg / 25 deg
ui(1:4) = deflection_to_pwm(defl);
fprintf('Rudder 20 deg: yaw_cmd = %.2f, fin PWM %s\n', delta_cmd/model.delta_max, mat2str(ui(1:4)'));
[t, X] = simulate(model, x1, ui, 300, h);
last = t >= t(end) - 30;
Ug = mean(hypot(X(last,1), X(last,2)));
r_ss = mean(X(last,6));
R_kin = Ug / abs(r_ss);
[R_fit, xc, yc] = circleFit(X(last,7), X(last,8));
fprintf('Steady turn: yaw rate %.3f rad/s (%.2f deg/s), speed %.3f m/s, roll %.2f deg, sideslip %.2f deg\n', ...
    r_ss, rad2deg(r_ss), Ug, rad2deg(mean(X(last,10))), rad2deg(atan2(mean(X(last,2)), mean(X(last,1)))));
fprintf('Turning radius: %.2f m from speed/yaw rate, %.2f m from a circle fit of the last 30 s (auv1-sim earlier estimate ~7 m, rough cross-check only)\n', ...
    R_kin, R_fit);
fprintf('Turn direction: yaw rate %s\n', ternary(r_ss > 0, 'positive (starboard), as +yaw_cmd requires', 'NEGATIVE'));
fig = figure('Visible', 'off', 'Position', [100 100 800 650]);
subplot(2,2,[1 3]); plot(X(:,8), X(:,7)); hold on; plot(yc, xc, 'k+');
theta = linspace(0, 2*pi, 100); plot(yc + R_fit*sin(theta), xc + R_fit*cos(theta), 'k:');
axis equal; grid on; xlabel('East (m)'); ylabel('North (m)'); title('Turning circle, 20 deg rudder');
subplot(2,2,2); plot(t, rad2deg(X(:,6))); grid on; ylabel('Yaw rate (deg/s)');
subplot(2,2,4); plot(t, hypot(X(:,1), X(:,2))); grid on; ylabel('Speed (m/s)'); xlabel('Time (s)');
saveas(fig, fullfile(valDir, 'tg5_case3_turning_circle.png')); close(fig);

%% 6. TG5 case 4: 10 deg stern-plane step at 1 m/s
fprintf('\n=== 6. TG5 case 4: 10 deg stern-plane step ===\n');
x0 = x1; x0(9) = 20;
ui = model.ui0; ui(5) = pwm1;
[~, defl] = fin_mixer(deg2rad(10) / model.delta_max, 0, 0);   % pitch_cmd = 10 deg / 25 deg
ui(1:4) = deflection_to_pwm(defl);
fprintf('Stern plane 10 deg: pitch_cmd = %.2f, fin PWM %s\n', deg2rad(10)/model.delta_max, mat2str(ui(1:4)'));
[t, X] = simulate(model, x0, ui, 60, h);
zdot = gradient(X(:,9), h);                        % Depth rate, positive down
for tt = [5 10 20 40 60]
    k = find(t >= tt, 1);
    fprintf('  t = %2d s: pitch %6.2f deg, depth rate %+.3f m/s (negative = rising), speed %.3f m/s\n', ...
        tt, rad2deg(X(k,11)), zdot(k), X(k,1));
end
fprintf('Final: pitch %.2f deg, depth rate %+.3f m/s, depth %.2f m (started at 20 m)\n', ...
    rad2deg(X(end,11)), mean(zdot(t >= 50)), X(end,9));
fig = figure('Visible', 'off', 'Position', [100 100 700 650]);
subplot(3,1,1); plot(t, rad2deg(X(:,11))); grid on; ylabel('Pitch (deg)'); title('TG5 case 4: 10 deg stern-plane step at 1 m/s');
subplot(3,1,2); plot(t, X(:,9)); grid on; set(gca, 'YDir', 'reverse'); ylabel('Depth (m)');
subplot(3,1,3); plot(t, zdot); grid on; ylabel('Depth rate (m/s)'); xlabel('Time (s)');
saveas(fig, fullfile(valDir, 'tg5_case4_stern_plane_step.png')); close(fig);

%% 7. SIMauv1 end to end, option 1, Euler angles
fprintf('\n=== 7. SIMauv1(1, 1) end to end ===\n');
close all force;
set(0, 'DefaultFigureVisible', 'off');
resetVis = onCleanup(@() set(0, 'DefaultFigureVisible', 'on'));
clear integralSMCheading ALOS3D ALOSpsi;
tic; SIMauv1(1, 1); tSim = toc;
figs = findall(0, 'type', 'figure');
lines = findall(figs, 'type', 'line');
nNaN = 0; nPts = 0;
for k = 1:numel(lines)
    xd = lines(k).XData; yd = lines(k).YData; zd = lines(k).ZData;
    nNaN = nNaN + nnz(~isfinite(xd)) + nnz(~isfinite(yd)) + nnz(~isfinite(zd));
    nPts = nPts + numel(yd);
end
fprintf('Completed without error in %.1f s; %d figures, %d plotted points, %d non-finite  ->  %s\n', ...
    tSim, numel(figs), nPts, nNaN, passfail(nNaN == 0 && nPts > 0));
% Tracking (reported only, controllers are still tuned for REMUS)
fig2 = figs([figs.Number] == 2);
ax = findall(fig2, 'type', 'axes');
[~, order] = sort(arrayfun(@(a) a.Position(2), ax), 'descend');
ax = ax(order);                                    % Top to bottom: heave, roll, pitch, yaw
names = {'Depth (m)', 'Roll (deg)', 'Pitch (deg)', 'Yaw (deg)'};
for k = [1 3 4]
    c = flipud(ax(k).Children);                    % True first, desired second
    tt = c(1).XData; yTrue = c(1).YData; yDes = c(2).YData;
    err = yTrue - yDes;
    if k == 4, err = mod(err + 180, 360) - 180; end
    sel = tt > 100;
    fprintf('  %-12s tracking error after 100 s: RMS %.3f, max %.3f, final %.3f\n', names{k}, ...
        sqrt(mean(err(sel).^2)), max(abs(err(sel))), err(end));
end
fprintf('  depth: min %.2f m, max %.2f m; final true %.2f vs desired %.2f\n', ...
    min(ax(1).Children(2).YData), max(ax(1).Children(2).YData), ax(1).Children(2).YData(end), ax(1).Children(1).YData(end));
fig1 = figs([figs.Number] == 1);
ax1 = findall(fig1, 'type', 'axes');
[~, order] = sort(arrayfun(@(a) a.Position(2), ax1), 'descend');
ax1 = ax1(order);
u = ax1(1).Children(1).YData;
fprintf('  surge speed: initial %.2f, min %.2f, max %.2f, final %.2f m/s\n', u(1), min(u), max(u), u(end));
for k = 1:numel(figs)
    saveas(figs(k), fullfile(valDir, sprintf('passB_SIMauv1_opt1_euler_fig%d.png', figs(k).Number)));
end
close all force;
end

% ------------------------------------------------------------------------------
function [t, X] = simulate(model, x0, ui, T, h)
% RK4 simulation with constant inputs and no ocean current
t = 0:h:T;
X = zeros(numel(t), numel(x0));
x = x0;
for i = 1:numel(t)
    X(i,:) = x';
    x = rk4(@auv1, h, x, ui, 0, 0, 0, model);
end
end

function U = steadySpeed(model, pwm, h)
% Steady straight-run surge speed at a constant stern PWM, fins neutral
ui = model.ui0; ui(5) = pwm;
x0 = zeros(12,1); x0(1) = 1;
[~, X] = simulate(model, x0, ui, 200, h);
U = X(end,1);
end

function [R, xc, yc] = circleFit(x, y)
% Algebraic least-squares circle fit
x = x(:); y = y(:);
A = [x, y, ones(size(x))];
c = A \ (-(x.^2 + y.^2));
xc = -c(1)/2; yc = -c(2)/2;
R = sqrt(xc^2 + yc^2 - c(3));
end

function f = tbl(T, pwm)
f = interp1(T.pwm, T.force, pwm);
end

function r = ternary(c, a, b)
if c, r = a; else, r = b; end
end

function s = passfail(ok)
if ok, s = 'PASS'; else, s = 'FAIL'; end
end
