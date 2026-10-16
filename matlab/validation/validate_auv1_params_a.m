function validate_auv1_params_a()
% Validation of the AUV1 parameter set, pass A (hull, mass, buoyancy, fins).
% Runs the equivalence test, the at-rest checks, the free-decay checks and
% SIMauv1 end to end. Plots and a text log are saved next to this file.
%
% Usage: run validate_auv1_params_a

valDir = fileparts(mfilename('fullpath'));
matlabDir = fileparts(valDir);
oldPath = path;
restorePath = onCleanup(@() path(oldPath));
run(fullfile(matlabDir, 'startup_auv1.m'));
addpath(fullfile(matlabDir, 'tests'));

logFile = fullfile(valDir, 'validation_results.txt');
if exist(logFile, 'file'), delete(logFile); end
diary(logFile);
closeDiary = onCleanup(@() diary('off'));

h = 0.05;                                  % Sampling time (s)
P = auv1Params('auv1');
model = P.model;

fprintf('=== Parameter summary (auv1) ===\n');
[~,~,M] = auv1(zeros(12,1), zeros(3,1), 0, 0, 0, model);
MRB = spheroid(model.a, model.b, zeros(3,1), model.r_bG);
fprintf('Spheroid mass            : %.4f kg (target 22)\n', MRB(1,1));
fprintf('a, b                     : %.4f m, %.4f m (L/2 = %.3f, D/2 = %.3f)\n', ...
    model.a, model.b, model.L_auv/2, model.D_auv/2);
fprintf('Added mass diag(M-MRB)   : %s\n', mat2str(diag(M - MRB)', 4));
fprintf('W = %.3f N, dB = %.3f N\n', MRB(1,1)*model.g_mu, model.dB);

%% 1. Equivalence test (REMUS 100 parameter set)
fprintf('\n=== 1. Equivalence test ===\n');
test_auv1_matches_remus100();

%% 2. At rest, dB = 0, zero inputs, 60 s
fprintf('\n=== 2. At rest, dB = 0, zero inputs, 60 s ===\n');
m0 = model; m0.dB = 0;
[t, X] = simulate(m0, zeros(12,1), 60, h);
maxVel = max(abs(X(:,1:6)), [], 'all');
fprintf('max |velocity| over 60 s : %.3g (limit 1e-6)  ->  %s\n', maxVel, passfail(maxVel < 1e-6));
fprintf('max |position change|    : %.3g m\n', max(abs(X(:,7:9)), [], 'all'));

fig = figure('Visible', 'off', 'Position', [100 100 700 450]);
plot(t, X(:,1:6)); grid on; xlabel('Time (s)'); ylabel('Velocity (m/s, rad/s)');
legend('u','v','w','p','q','r','Location','best');
title('At rest, dB = 0, zero inputs');
saveas(fig, fullfile(valDir, 'at_rest_dB0.png')); close(fig);

%% 3. At rest, dB = 2 N: steady rise
fprintf('\n=== 3. At rest, dB = 2 N (rise) ===\n');
x0 = zeros(12,1); x0(9) = 20;                         % Start 20 m deep
[t, X] = simulate(model, x0, 120, h);
w_end = X(end,3);  zdot_end = -(X(end,9) - X(end-round(10/h),9)) / 10;  % Rise speed from last 10 s
dwdt = (X(end,3) - X(end-1,3)) / h;
fprintf('final w                  : %.4f m/s (dw/dt = %.2g m/s2)\n', w_end, dwdt);
fprintf('steady rise speed        : %.4f m/s (from last 10 s of depth)\n', zdot_end);
fprintf('final pitch, roll        : %.4f deg, %.4f deg\n', rad2deg(X(end,11)), rad2deg(X(end,10)));

% Hand calculation: net upward force dB balanced by cross-flow drag of the
% hull side profile, F = 0.5*rho_mss*Cd*D*L*w^2 (rho_mss = 1025 inside crossFlowDrag)
rho_mss = mssConstants().rho_water;
Cd_cyl = 1.2;                                         % Cylinder, Re ~ 1e4-1e5 (DNV-RP-C205)
w_hand = sqrt(2*model.dB / (rho_mss * Cd_cyl * model.D_auv * model.L_auv));
% Same balance using crossFlowDrag (strip theory, cylinder model incl. end effect)
w_cf = fzero(@(w) crossZ(model, -w) - model.dB, 0.1);
fprintf('hand calc, Cd = %.1f      : %.4f m/s\n', Cd_cyl, w_hand);
fprintf('balance with crossFlowDrag: %.4f m/s\n', w_cf);
fprintf('simulated / hand (Cd=1.2) : %.2f ; simulated / crossFlowDrag : %.2f\n', ...
    abs(zdot_end)/w_hand, abs(zdot_end)/w_cf);

fig = figure('Visible', 'off', 'Position', [100 100 700 600]);
subplot(2,1,1); plot(t, -X(:,3)); hold on; yline(w_hand, '--'); yline(w_cf, ':');
grid on; ylabel('Rise speed (m/s)'); title('At rest, dB = 2 N');
legend('Simulated','Hand calc (Cd = 1.2)','crossFlowDrag balance','Location','southeast');
subplot(2,1,2); plot(t, X(:,9)); grid on; set(gca,'YDir','reverse');
xlabel('Time (s)'); ylabel('Depth z (m)');
saveas(fig, fullfile(valDir, 'at_rest_dB2_rise.png')); close(fig);

%% 4. Free decay from 10 deg pitch and 10 deg roll, dB = 0
fprintf('\n=== 4. Free decay, dB = 0, zero inputs ===\n');
% Eigenvalues of the linearised model at rest (surge, sway, heave, roll, pitch)
ev = eig(linearise(m0));
ev = ev(imag(ev) >= 0);
[~, order] = sort(abs(ev));
fprintf('linearised modes at rest (|lambda| ascending):\n');
for e = ev(order).'
    if abs(imag(e)) > 1e-9
        fprintf('  %.3f%+.3fi  wn %.3f rad/s, zeta %.3f, T %.2f s\n', real(e), imag(e), abs(e), -real(e)/abs(e), 2*pi/imag(e));
    else
        fprintf('  %.3f (real)\n', real(e));
    end
end
cases = {'pitch', 11, 'Pitch angle (deg)'; 'roll', 10, 'Roll angle (deg)'};
fig = figure('Visible', 'off', 'Position', [100 100 700 600]);
for c = 1:size(cases,1)
    x0 = zeros(12,1); x0(cases{c,2}) = deg2rad(10);
    [t, X] = simulate(m0, x0, 60, h);
    ang = rad2deg(X(:, cases{c,2}));
    vel = X(:, cases{c,2} - 6);                       % q or p
    idx = find(abs(ang) > 0.2, 1, 'last');            % Last time outside 2% band
    if isempty(idx), tset = 0; else, tset = t(min(idx+1, numel(t))); end
    fprintf('%s: final %.4g deg, max |%s rate| at end %.2g, 2%% settling time %.1f s  ->  %s\n', ...
        cases{c,1}, ang(end), cases{c,1}, abs(vel(end)), tset, passfail(abs(ang(end)) < 0.2 && tset < t(end)));
    [Tp, zeta_p, nPk] = decayFromPeaks(t, ang);
    if nPk >= 2
        fprintf('  measured: period %.2f s, damping ratio %.3f (log decrement, %d same-sign peaks)\n', Tp, zeta_p, nPk);
    else
        fprintf('  measured: no repeated overshoot, motion is (near) critically/over-damped\n');
    end
    subplot(2,1,c); plot(t, ang); grid on; ylabel(cases{c,3});
    title(sprintf('Free decay from 10 deg %s, dB = 0', cases{c,1}));
    if c == 2, xlabel('Time (s)'); end
end
saveas(fig, fullfile(valDir, 'free_decay_pitch_roll.png')); close(fig);

%% 5. SIMauv1 end to end, option 1, Euler angles
fprintf('\n=== 5. SIMauv1(1, 1) end to end ===\n');
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
fprintf('completed without error in %.1f s; %d figures, %d plotted points, %d non-finite  ->  %s\n', ...
    tSim, numel(figs), nPts, nNaN, passfail(nNaN == 0 && nPts > 0));
for k = 1:numel(figs)
    saveas(figs(k), fullfile(valDir, sprintf('SIMauv1_opt1_euler_fig%d.png', figs(k).Number)));
end
close all force;
end

% ------------------------------------------------------------------------------
function [t, X] = simulate(model, x0, T, h)
% RK4 simulation with zero control inputs and no ocean current
t = 0:h:T;
X = zeros(numel(t), numel(x0));
x = x0;
for i = 1:numel(t)
    X(i,:) = x';
    x = rk4(@auv1, h, x, zeros(3,1), 0, 0, 0, model);
end
end

function A = linearise(model)
% Numerical Jacobian of xdot w.r.t. the 12 Euler states at rest (zero inputs)
n = 12; A = zeros(n); eps_ = 1e-6;
for j = 1:n
    dx = zeros(n,1); dx(j) = eps_;
    fp = auv1(dx,  zeros(3,1), 0, 0, 0, model);
    fm = auv1(-dx, zeros(3,1), 0, 0, 0, model);
    A(:,j) = (fp - fm) / (2*eps_);
end
A = A(setdiff(1:n, [7 8 9 12]), setdiff(1:n, [7 8 9 12])); % Drop x, y, z, psi (no restoring)
end

function [Tp, zeta, nPk] = decayFromPeaks(t, y)
% Period and damping ratio from the logarithmic decrement of same-sign peaks
y = y(:);
isPk = [false; y(2:end-1) > y(1:end-2) & y(2:end-1) >= y(3:end); false] & y > 1e-3;
loc = find(isPk);  pk = y(loc);
nPk = numel(pk);
Tp = NaN; zeta = NaN;
if nPk >= 2
    Tp = mean(diff(t(loc)));
    delta = mean(log(pk(1:end-1) ./ pk(2:end)));
    zeta = delta / sqrt(4*pi^2 + delta^2);
end
end

function Z = crossZ(model, w)
% Heave cross-flow drag force (N) on the hull for body heave velocity w
tau = crossFlowDrag(model.L_auv, model.D_auv, model.D_auv, [0 0 w 0 0 0]', 'cylinder');
Z = tau(3);
end

function s = passfail(ok)
if ok, s = 'PASS'; else, s = 'FAIL'; end
end
