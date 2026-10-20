function test_auv1Visualiser()
% Checks auv1Visualiser against the data it is given, and through its UI:
%   - every time plot draws the logged arrays exactly, within its axis limits
%   - each frame shows one logged sample everywhere: vehicle position,
%     attitude and fin angles, trail, readout, clock, plot cursors, slider
%   - the active waypoint matches ALOSpsi's k_active (2.5-D) and the leg
%     being flown (3-D), and is withheld when the logged errors disagree
%   - the controls work when used through the UI (App Testing Framework):
%     Play/Pause at the selected speed, slider, step buttons, plot clicks,
%     keys, View, Speed, Vehicle scale, Vert. exag., playing to the end and
%     replaying, and closing while playing leaves no timer behind
%   - playback also works when it is called with no output, as SIMauv1 does
%   - bad input is rejected and a diverged run is cut at its first NaN
% The runs are synthetic, with the tracking errors computed by the real
% ALOSpsi and ALOS3D, so no simulation is needed. Runs without a display
% (matlab -batch) in about a minute.
%
% Usage (from anywhere): run test_auv1Visualiser

testsDir = fileparts(mfilename('fullpath'));
run(fullfile(fileparts(testsDir), 'startup_auv1.m'));
tc = matlab.uitest.TestCase.forInteractiveUse;
nTimers = numel(timerfindall);
lastwarn('');

% ------------------------------------------------------------------------------
% 3-D ALOS run: plots, frames, active leg and every control
% ------------------------------------------------------------------------------
closeAll = onCleanup(@() delete(findall(groot, 'Type', 'figure', 'Tag', 'auv1Visualiser')));
[s, leg] = makeRun(4);
fig = auv1Visualiser(s);
checkPlots(fig, s);
assert(~isempty(tagged(fig, 'activeLeg')), 'Active leg not shown for a consistent run');
for k = [1 2 400 1201 numel(s.t) - 1 numel(s.t)]
    seekTo(fig, s.t(k));
    checkFrame(fig, s, k, legInside(s, leg, k));
end

play = tagged(fig, 'play');  slider = tagged(fig, 'time');
seekTo(fig, s.t(1));

% Play and pause with the button
pick(tc, fig, 'speed', '5x');
t0 = slider.Value;
tc.press(play);
assert(strcmp(play.Text, 'Pause'), 'Play button did not start playback');
pause(1);
assert(slider.Value > t0, 'Play button pressed, but the time did not move');
checkFrame(fig, s, []);   % Consistent while playing
tc.press(play);
assert(strcmp(play.Text, 'Play'), 'Pause button did not stop playback');
tPaused = slider.Value;
pause(0.5);
assert(slider.Value == tPaused, 'Still playing after Pause');
checkFrame(fig, s, sampleAt(s, tPaused));

% Playback rate at 5x. Started by the button's own callback: after a press
% gesture the test framework can hold MATLAB for a second or more (seen in
% R2025b), which starves the playback timer during the measurement.
play.ButtonPushedFcn(play, []);
t0 = slider.Value;  wall = tic;
pause(1.5);
t1 = slider.Value;  el = toc(wall);
play.ButtonPushedFcn(play, []);
assert(t1 - t0 > 0.7 * 5 * el && t1 - t0 <= 5 * (el + 0.1), ...
    'Played %.2f s in %.2f s at 5x', t1 - t0, el);

% Slider, step buttons, plot click and keys
tc.drag(slider, s.t(1), 60);
assert(abs(slider.Value - 60) < 1, 'Slider drag went to %.2f s', slider.Value);
checkFrame(fig, s, sampleAt(s, slider.Value));
k0 = sampleAt(s, slider.Value);
tc.press(tagged(fig, 'forward'));
checkFrame(fig, s, sampleAt(s, s.t(k0) + 1));
tc.press(tagged(fig, 'back'));
checkFrame(fig, s, k0);
depthAx = tagged(fig, 'depth').Parent;
tc.press(depthAx, [30 mean(depthAx.YLim)]);
assert(abs(slider.Value - 30) < 1, 'Plot click went to %.2f s', slider.Value);
checkFrame(fig, s, sampleAt(s, slider.Value));
k0 = sampleAt(s, slider.Value);
pressKey(fig, 'rightarrow');            checkFrame(fig, s, sampleAt(s, s.t(k0) + 1));
pressKey(fig, 'rightarrow', 'shift');   checkFrame(fig, s, sampleAt(s, s.t(k0) + 11));
pressKey(fig, 'leftarrow', 'shift');    checkFrame(fig, s, sampleAt(s, s.t(k0) + 1));
pressKey(fig, 'end');                   checkFrame(fig, s, numel(s.t));
pressKey(fig, 'home');                  checkFrame(fig, s, 1);
pressKey(fig, 'space');
assert(strcmp(play.Text, 'Pause'), 'Space did not start playback');
pressKey(fig, 'space');
assert(strcmp(play.Text, 'Play'), 'Space did not stop playback');

% Views, vertical exaggeration and vehicle scale
seekTo(fig, 80);
ax = tagged(fig, 'vehicle').Parent.Parent;
pick(tc, fig, 'view', 'Follow');
k = sampleAt(s, 80);
assert(strcmp(ax.Projection, 'perspective') && isequal(ax.CameraTarget, s.eta(k,1:3)), ...
    'Follow view does not look at the vehicle');
pressKey(fig, 'rightarrow');
assert(isequal(ax.CameraTarget, s.eta(sampleAt(s, 81), 1:3)), 'Follow camera did not move');
pick(tc, fig, 'view', 'Top');
assert(strcmp(ax.Projection, 'orthographic') && isequal(ax.CameraUpVector, [1 0 0]), ...
    'Top view not set');
pick(tc, fig, 'view', 'Overview');
pick(tc, fig, 'vertExag', '1x');   % Then the attitude is the logged one exactly
tc.type(tagged(fig, 'scale'), 3);
checkFrame(fig, s, sampleAt(s, 81));
pick(tc, fig, 'vertExag', '10x');
checkFrame(fig, s, sampleAt(s, 81));

% Play to the end, then Play replays from the start
pick(tc, fig, 'speed', '200x');
tc.press(play);
wall = tic;
while strcmp(play.Text, 'Pause') && toc(wall) < 10; pause(0.1); end
assert(strcmp(play.Text, 'Play'), 'Playback did not stop at the end');
checkFrame(fig, s, numel(s.t));
pick(tc, fig, 'speed', '1x');
tc.press(play);
pause(0.3);
assert(strcmp(play.Text, 'Pause') && slider.Value < 10, ...
    'Play at the end did not replay (button "%s", t = %.2f s)', play.Text, slider.Value);

% Close while playing
delete(fig);
pause(0.3);
assert(numel(timerfindall) == nTimers, 'Timer left behind after closing while playing');
[msg, id] = lastwarn;
assert(isempty(msg), 'Unexpected warning %s: %s', id, msg);

% ------------------------------------------------------------------------------
% 2.5-D ALOS run: the active waypoint is ALOSpsi's k_active at every sample
% ------------------------------------------------------------------------------
[s, ~, kRef] = makeRun(3);
fig = auv1Visualiser(s);
checkPlots(fig, s);
switches = find(diff(kRef)) + 1;
for k = unique([1; switches - 1; switches; switches + 1; numel(s.t)])'
    seekTo(fig, s.t(k));
    checkFrame(fig, s, k, kRef(k));
end
delete(fig);

% A run whose logged errors disagree with the replay: no active leg
s4 = makeRun(4);
s4.alos.y_e(700) = s4.alos.y_e(700) + 0.01;
[~, fig] = evalc('auv1Visualiser(s4)');     % Captures the expected warning
[~, id] = lastwarn;
assert(strcmp(id, 'auv1Visualiser:waypointReplay'), 'Mismatch not reported');
assert(isempty(tagged(fig, 'activeLeg', false)), 'Unverified active leg shown');
delete(fig);

% ------------------------------------------------------------------------------
% Autopilot run (fin angle plot), saved to and replayed from a .mat file
% ------------------------------------------------------------------------------
s = makeRun(1);
file = [tempname '.mat'];
deleteFile = onCleanup(@() delete(file));
save(file, '-struct', 's');
fig = auv1Visualiser(file);
checkPlots(fig, s);
assert(isempty(tagged(fig, 'activeLeg', false)), 'Waypoints shown for an autopilot run');
seekTo(fig, 20);
checkFrame(fig, s, sampleAt(s, 20));
delete(fig);

% ------------------------------------------------------------------------------
% Called with no output, as SIMauv1 does: playback must still run. Errors in a
% timer callback do not throw, so check that the replay time moved.
% ------------------------------------------------------------------------------
s = makeRun(1);
evalc('auv1Visualiser(s)');                 % No output, so no handle to use
fig = findall(groot, 'Type', 'figure', 'Tag', 'auv1Visualiser');
assert(isscalar(fig), 'Expected one replay window, found %d', numel(fig));
play = tagged(fig, 'play');  slider = tagged(fig, 'time');
pick(tc, fig, 'speed', '5x');
t0 = slider.Value;
tc.press(play);
assert(strcmp(play.Text, 'Pause'), 'Play button did not start playback (no output)');
pause(1);
assert(slider.Value > t0, ['Replay time did not move when called with no ' ...
    'output (timer callback failing?)']);
tc.press(play);
assert(strcmp(play.Text, 'Play'), 'Pause button did not stop playback (no output)');
delete(fig);

% ------------------------------------------------------------------------------
% Input checks
% ------------------------------------------------------------------------------
lastwarn('');
s = makeRun(1);
s.eta(1001:end, 3) = NaN;                   % Diverged run
[~, fig] = evalc('auv1Visualiser(s)');
[~, id] = lastwarn;
slider = tagged(fig, 'time');
assert(strcmp(id, 'auv1Visualiser:nonFinite') && slider.Limits(2) == s.t(1000), ...
    'Diverged run not cut at its first NaN');
delete(fig);

s = makeRun(1);
fig = auv1Visualiser(struct('t', s.t, 'eta', s.eta));   % Only the required fields
seekTo(fig, 50);
delete(fig);

expectError(@() auv1Visualiser(struct('t', [0 1 1], 'eta', zeros(3,6))), 'auv1Visualiser:badInput');
expectError(@() auv1Visualiser(struct('t', [0 1 2], 'eta', zeros(3,5))), 'auv1Visualiser:badInput');
expectError(@() auv1Visualiser(42), 'auv1Visualiser:badInput');

assert(numel(timerfindall) == nTimers, 'Timers left behind');
fprintf('PASS: auv1Visualiser draws the logged data and its controls work\n');
end

% ==============================================================================
% CHECKS
% ==============================================================================
function checkPlots(fig, s)
% Each time plot line is the logged array, and lies within the axis limits
t = s.t(:)';
cf = s.ControlFlag;
checkLine(fig, 'depth', t, s.eta(:,3)');
if cf == 4
    assert(isempty(tagged(fig, 'depthRef', false)), 'Desired depth shown in 3-D ALOS');
else
    checkLine(fig, 'depthRef', t, s.ref.z_d');
end
checkAngleLine(fig, 'yaw', t, rad2deg(ssa(s.eta(:,6)))');
checkAngleLine(fig, 'yawRef', t, rad2deg(ssa(s.ref.psi_d))');
if cf >= 3
    checkLine(fig, 'crossTrack', t, s.alos.y_e');
    if cf == 4
        checkLine(fig, 'vertTrack', t, s.alos.z_e');
    else
        assert(isempty(tagged(fig, 'vertTrack', false)), 'Vertical-track error in 2.5-D');
    end
else
    checkLine(fig, 'rudder', t, rad2deg(s.ui(:,1))');
    checkLine(fig, 'stern', t, rad2deg(s.ui(:,2))');
end
for c = findall(fig, 'Tag', 'cursor')'
    a = c.Parent;
    assert(isequal(a.XLim, [t(1) t(end)]), 'Plot does not span the run');
    for h = findall(a, 'Type', 'line')'
        y = h.YData(isfinite(h.YData));
        assert(all(y >= a.YLim(1) & y <= a.YLim(2)), 'Line %s clipped by YLim', h.Tag);
    end
end
end

function checkLine(fig, tag, x, y)
h = tagged(fig, tag);
assert(isequal(h.XData, x) && isequal(h.YData, y), 'Line %s is not the logged data', tag);
end

function checkAngleLine(fig, tag, x, y)
% The logged angle, with NaN inserted exactly at the +-180 deg wraps
h = tagged(fig, tag);
ok = ~isnan(h.YData);
assert(isequal(h.XData(ok), x) && isequal(h.YData(ok), y), 'Line %s is not the logged data', tag);
assert(nnz(~ok) == nnz(abs(diff(y)) > 180), 'Line %s has wrong breaks', tag);
dy = diff(h.YData);
assert(all(abs(dy(isfinite(dy))) <= 180), 'Line %s draws a wrap', tag);
end

function checkFrame(fig, s, k, kWpt)
% Every part of the window shows sample k; k = [] for the sample the slider
% is at. All values are read before any is checked, as finding objects can
% let a playback tick run.
c = findall(fig, 'Tag', 'cursor');
H = struct('slider', tagged(fig, 'time'), 'clock', tagged(fig, 'clock'), ...
    'readout', tagged(fig, 'readout'), 'vertExag', tagged(fig, 'vertExag'), ...
    'scale', tagged(fig, 'scale'), 'vehicle', tagged(fig, 'vehicle'), ...
    'rudder', tagged(fig, 'rudderFins'), 'stern', tagged(fig, 'sternFins'), ...
    'trail', tagged(fig, 'trail'), 'shadow', tagged(fig, 'shadow'), ...
    'active', tagged(fig, 'activeLeg', false));
v = struct('cursors', {get(c, 'XData')}, 'slider', H.slider.Value, ...
    'clock', H.clock.Text, 'readout', {H.readout.Text}, 've', H.vertExag.Value, ...
    'scale', H.scale.Value, 'M', H.vehicle.Matrix, 'Mr', H.rudder.Matrix, ...
    'Ms', H.stern.Matrix, 'trail', [H.trail.XData(:) H.trail.YData(:) H.trail.ZData(:)], ...
    'shadow', [H.shadow.XData H.shadow.YData], 'active', []);
if ~isempty(H.active)
    v.active = [H.active.XData(:) H.active.YData(:) H.active.ZData(:)];
end
if isempty(k); k = sampleAt(s, v.slider); end

N = numel(s.t);  tk = s.t(k);  p = s.eta(k,1:3);
assert(numel(c) == 3 && all(cellfun(@(x) isequal(x, [tk tk]), v.cursors)), ...
    'Plot cursors not at sample %d', k);
assert(v.slider == tk, 'Slider not at sample %d', k);
assert(strcmp(v.clock, sprintf('t = %.2f / %.2f s', tk, s.t(end))), ...
    'Clock not at sample %d', k);
txt = strjoin(string(v.readout), newline);
row = @(label, fmt, varargin) contains(txt, sprintf(['%-14s' fmt], label, varargin{:}));
assert(row('Time', '%8.2f s  (sample %d of %d)', tk, k, N) && ...
    row('North / East', '%8.1f / %.1f m', p(1), p(2)) && ...
    row('Depth', '%8.1f m', p(3)) && ...
    row('Yaw', '%8.1f deg', rad2deg(ssa(s.eta(k,6)))) && ...
    row('Rudder', '%8.1f deg', rad2deg(s.ui(k,1))), ...
    'Readout not at sample %d', k);

% Vehicle: at the logged position; a rotation that is the logged attitude
% when depth is not exaggerated
D = diag([1 1 v.ve]);
assert(norm(D \ v.M(1:3,4) - p') < 1e-9, 'Vehicle not at sample %d', k);
Q = v.M(1:3,1:3) / v.scale;
assert(norm(Q' * Q - eye(3)) < 1e-9 && det(Q) > 0, 'Vehicle attitude is not a rotation');
if v.ve == 1
    assert(norm(Q - Rzyx(s.eta(k,4), s.eta(k,5), s.eta(k,6))) < 1e-9, ...
        'Vehicle attitude is not the logged one at sample %d', k);
end

% Fins: the trailing edge turns to starboard for positive delta_r and down
% for positive delta_s, by the logged angle saturated at delta_max
dMax = s.P.model.delta_max;
xh = H.rudder.UserData;
q = v.Mr * [xh - 1; 0; 0; 1];
assert(abs(atan2(q(2), xh - q(1)) - min(max(s.ui(k,1), -dMax), dMax)) < 1e-9, ...
    'Rudder angle wrong at sample %d', k);
q = v.Ms * [xh - 1; 0; 0; 1];
assert(abs(atan2(q(3), xh - q(1)) - min(max(s.ui(k,2), -dMax), dMax)) < 1e-9, ...
    'Stern-plane angle wrong at sample %d', k);

% Trail: logged positions from the start up to this sample
assert(isequal(v.trail(1,:), s.eta(1,1:3)) && isequal(v.trail(end,:), p) && ...
    all(ismember(v.trail, s.eta(1:k,1:3), 'rows')), 'Trail does not end at sample %d', k);
assert(isequal(v.shadow, p(1:2)), 'Shadow not at sample %d', k);

if nargin > 3 && isfinite(kWpt)
    W = [s.wpt.pos.x(:) s.wpt.pos.y(:) s.wpt.pos.z(:)];
    assert(~isempty(v.active) && isequal(v.active(1,:), W(kWpt,:)), ...
        'Active leg at sample %d does not start at waypoint %d', k, kWpt);
    if kWpt < size(W, 1)
        assert(isequal(v.active(2,:), W(kWpt+1,:)), ...
            'Active leg at sample %d does not end at waypoint %d', k, kWpt + 1);
        assert(contains(txt, sprintf('%5d -> %d', kWpt, kWpt + 1)), ...
            'Readout waypoint wrong at sample %d', k);
    else
        assert(contains(txt, 'final, holding bearing'), 'Readout waypoint wrong at sample %d', k);
    end
end
end

% ==============================================================================
% HELPERS
% ==============================================================================
function [s, leg, kRef] = makeRun(ControlFlag)
% Synthetic run: the vehicle moves along the waypoint path at 2.5 m/s, with
% small sideways and vertical offsets, and 50 m past the last waypoint; its
% yaw crosses +-180 deg. leg is the path segment each sample lies on. The
% tracking errors are those of the real guidance law; kRef is ALOSpsi's
% k_active (ControlFlag 3).
h = 0.05;  t = (0:h:120)';  N = numel(t);
P = auv1Params();
wpt.pos.x = [0 10 60 60];  wpt.pos.y = [0 80 150 230];  wpt.pos.z = [0 10 10 20];
W = [wpt.pos.x' wpt.pos.y' wpt.pos.z'];
W(end+1,:) = W(end,:) + [0 100 0];          % On along the last bearing
dW = diff(W);  len = sqrt(sum(dW.^2, 2));  S = [0; cumsum(len)];
along = 2.5 * t;
leg = discretize(along, S);
nrm = [-dW(leg,2) dW(leg,1)] ./ sqrt(sum(dW(leg,1:2).^2, 2)); % Horizontal normal
pos = W(leg,:) + (along - S(leg)) .* dW(leg,:) ./ len(leg) ...
    + 1.5 * sin(t/7) .* [nrm zeros(N,1)] + [zeros(N,2) 0.5 * (1 - cos(t/5))];
psi = pi - 0.5 + 0.02 * t;

s.t = t;
s.h = h;
s.ControlFlag = ControlFlag;
s.KinematicsFlag = 1;
s.eta = [pos 0.1 * sin(t/5) 0.2 * sin(t/7) psi];
s.nu = [2.5 + 0.1 * sin(t/3), 0.1 * sin(t/4), 0.05 * cos(t/6), 0.01 * sin(t), ...
    0.01 * cos(t), 0.02 * ones(N,1)];
s.ui = [0.5 * sin(t/3), 0.3 * cos(t/4), 1000 + 3 * t];  % Rudder beyond delta_max
s.ref.z_d = pos(:,3) + 0.3 * sin(t/9);
s.ref.theta_d = s.eta(:,5) + 0.01;
s.ref.psi_d = psi + 0.05;
s.current.Vc = 0.5 + 0.001 * t;
s.current.betaVc = deg2rad(150) + 0.001 * t;
s.alos.y_e = zeros(N,1);  s.alos.z_e = zeros(N,1);
kRef = NaN(N,1);
if ControlFlag >= 3
    [~, s.alos.y_e, s.alos.z_e, kRef] = evalc('guidance(ControlFlag, pos, P.guidance, h, wpt)');
end
s.wpt = wpt;
s.P = P;
end

function [y_e, z_e, kRef] = guidance(ControlFlag, p, g, h, wpt)
% Tracking errors from ALOSpsi (ControlFlag 3) or ALOS3D (4), called as in SIMauv1
clear ALOSpsi ALOS3D
N = size(p, 1);
y_e = NaN(N,1);  z_e = NaN(N,1);  kRef = NaN(N,1);
for i = 1:N
    if ControlFlag == 3
        [~, y_e(i), ~, kRef(i)] = ALOSpsi(p(i,1), p(i,2), g.Delta_h, g.gamma_h, h, ...
            g.R_switch, wpt);
    else
        [~, ~, y_e(i), z_e(i)] = ALOS3D(p(i,1), p(i,2), p(i,3), g.Delta_h, g.Delta_v, ...
            g.gamma_h, g.gamma_v, g.M_theta, h, g.R_switch, wpt);
    end
end
clear ALOSpsi ALOS3D
end

function kWpt = legInside(s, leg, k)
% The leg sample k lies on, if it is more than 10 m from both of its ends
% (well clear of the 5 m switching radius); NaN otherwise
W = [s.wpt.pos.x(:) s.wpt.pos.y(:) s.wpt.pos.z(:)];
kWpt = NaN;
j = leg(k);
p = s.eta(k,1:3);
if j < size(W, 1) && norm(p - W(j,:)) > 10 && norm(p - W(j+1,:)) > 10
    kWpt = j;
elseif j == size(W, 1) && norm(p - W(j,:)) > 10
    kWpt = j;
end
end

function h = tagged(fig, tag, required)
h = findall(fig, 'Tag', tag);
if nargin < 3 || required
    assert(numel(h) == 1, 'Expected one object tagged %s, found %d', tag, numel(h));
end
end

function pick(tc, fig, tag, item)
% Chooses item in a dropdown, then lets its callback run
tc.choose(tagged(fig, tag), item);
drawnow
end

function seekTo(fig, tq)
slider = tagged(fig, 'time');
slider.Value = tq;
slider.ValueChangedFcn(slider, []);
end

function k = sampleAt(s, tq)
[~, k] = min(abs(s.t - tq));
end

function pressKey(fig, key, modifier)
if nargin < 3; modifier = {}; else; modifier = {modifier}; end
fig.KeyPressFcn(fig, struct('Key', key, 'Modifier', {modifier}));
end

function expectError(f, id)
try
    f();
catch err
    assert(strcmp(err.identifier, id), 'Expected error %s, got %s', id, err.identifier);
    return
end
error('Expected error %s, got none', id);
end
