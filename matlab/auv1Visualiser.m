function fig = auv1Visualiser(simOut)
% auv1Visualiser(simOut) opens a replay of an SIMauv1 run: a 3-D view of the
% vehicle, with moving rudders and stern planes, on its path, the waypoints
% and active leg (ControlFlag 3-4), a readout of the state and time plots.
%
% SIMauv1 opens it at the end of every run. To replay a run again later:
%
% Usage:
%   simOut = SIMauv1(4, 2);                 % Run the simulator, keep the results
%   auv1Visualiser(simOut)                  % Replay them again
%
%   save('run.mat', '-struct', 'simOut');   % Or save a run ...
%   auv1Visualiser('run.mat')               % ... and replay it later
%
%   fig = auv1Visualiser(...)               % Also return the figure handle
%
% Controls:
%   Play/Pause, -1 s/+1 s and the time slider (dragging it pauses). Click a
%   time plot to jump to that time. Keys, once the 3-D view or a plot has
%   been clicked: Space, Left/Right (1 s, Shift: 10 s), Home/End.
%   Speed  Replay speed as a multiple of real time. When drawing cannot keep
%          up, samples are skipped, so the replay never runs slow.
%   View   Overview - whole mission, drag to rotate, scroll to zoom
%          Follow   - chase camera off the vehicle's starboard quarter
%          Top      - North-East plane seen from above
%   Scale  Vehicle size multiplier, so the 1.6 m hull is visible over km.
%          In Follow view it also sets the camera distance; 1 = true size.
%   Vert.  Vertical exaggeration of depth. The vehicle keeps its shape and is
%          tilted to line up with the exaggerated path; the readout shows the
%          true angles.
%
% What is shown is the logged data. Each frame is one logged sample k, never
% interpolated, and every part of the window shows that sample: vehicle,
% trail, readout, plot cursors and time slider. The plots draw the logged
% arrays at full resolution. The active waypoint is replayed from the logged
% positions with the switching rule of ALOSpsi (ControlFlag 3) or ALOS3D (4),
% and is only shown if the replay reproduces the logged tracking errors.
%
% simOut fields (see the end of SIMauv1.m). Only t and eta are required:
%   t, eta, nu, ui, ref.{z_d,theta_d,psi_d}, current.{Vc,betaVc},
%   alos.{y_e,z_e}, wpt, P, ControlFlag, KinematicsFlag
%
% Plot frame: NED (x = North, y = East, z = Down). The y- and z-axes are
% reversed, which together is a rotation, so the scene is not mirrored.
%
% Tests: tests/test_auv1Visualiser.m
% Dependencies: Rzyx.m, ssa.m and auv1Params.m (run startup_auv1 first)

if exist('Rzyx', 'file') ~= 2 || exist('ssa', 'file') ~= 2
    error('auv1Visualiser:path', 'MSS functions not found. Run startup_auv1 first.');
end

d = loadRun(simOut);
t = d.t;  N = numel(t);
eta = d.eta;
pos = eta(:,1:3);                   % NED positions (m)
L = d.model.L_auv;
deltaMax = d.model.delta_max;
U = sqrt(sum(d.nu(:,1:3).^2, 2));   % Speed (m/s)
hasTrack = any(isfinite(d.y_e));    % Cross-track error logged (ControlFlag 3-4)
hasVert = any(isfinite(d.z_e));     % Vertical-track error logged (ControlFlag 4)

% ==============================================================================
% WAYPOINTS AND SCENE EXTENT
% ==============================================================================
hasPath = ~isempty(d.wpt);
kAct = [];                          % Active waypoint per sample, if verified
if hasPath
    W = [d.wpt.pos.x(:) d.wpt.pos.y(:) d.wpt.pos.z(:)]; % NED (m)
    nW = size(W, 1);
    kAct = replayWaypoints(pos, W, d.R_switch, d.ControlFlag, d.y_e, d.z_e);
end

pts = pos;
if hasPath; pts = [pts; W]; end
lo = min(pts, [], 1);  hi = max(pts, [], 1);
lo(3) = min(lo(3), 0);                       % Include the sea surface
hor = max(norm(hi(1:2) - lo(1:2)), 10 * L);  % Horizontal extent (m)
pad = max(0.05 * (hi - lo), [2 2 1]);
lims = [lo - pad; hi + pad];
zFloor = lims(2,3);

if hasPath
    % After the last waypoint ALOS holds the last bearing (and depth)
    last = W(nW,:) - W(nW-1,:);  last(3) = 0;
    W(nW+1,:) = W(nW,:) + max(100, 0.2 * hor) * last / max(norm(last), eps);
end

% Defaults: vehicle visible at mission scale, depth exaggerated so dives
% show, and a replay that takes about a minute
scale = max(1, round(0.06 * hor / L));
veOptions = [1 2 5 10];
ve = veOptions(find(veOptions <= max(1, hor / (4 * max(hi(3) - lo(3), 1))), 1, 'last'));
speedOptions = [0.5 1 2 5 10 20 50 100 200];
[~, iSpeed] = min(abs(log(speedOptions * 60 / (t(end) - t(1)))));
speed = speedOptions(iSpeed);
viewNames = {'Overview', 'Follow', 'Top'};
viewMode = 'Overview';

% The trail is drawn through every trailStride-th sample, and always ends
% at the current one. At 1 s or less apart the chords are within a few cm of
% the logged path, and drawing stays fast on long runs.
trailStride = max(1, min(round(1 / mean(diff(t))), ceil(N / 3000)));

% Playback state. The playback clock is t = clock0 + speed * toc(wall0).
k = 1;                 % Current sample, the only thing showFrame draws
playing = false;
clock0 = t(1);
wall0 = tic;

% ==============================================================================
% WINDOW LAYOUT
% ==============================================================================
mono = get(groot, 'FixedWidthFontName');
ss = get(groot, 'ScreenSize');
fig = uifigure('Name', ['AUV1 replay - ' d.modeName], 'Tag', 'auv1Visualiser', 'Color', 'w', ...
    'Position', [ss(1:2) + [0.05 0.07] .* ss(3:4), [0.9 0.85] .* ss(3:4)], ...
    'KeyPressFcn', @onKey);
main = uigridlayout(fig, [2 2], 'RowHeight', {'1x', 'fit'}, ...
    'ColumnWidth', {'2.2x', '1x'}, 'BackgroundColor', 'w');
side = uigridlayout(main, [4 1], 'RowHeight', {'fit', '1x', '1x', '1x'}, ...
    'Padding', 0, 'BackgroundColor', 'w');
side.Layout.Row = 1;  side.Layout.Column = 2;
controls = uigridlayout(main, [2 1], 'RowHeight', {'fit', 'fit'}, ...
    'Padding', 0, 'BackgroundColor', 'w');
controls.Layout.Row = 2;  controls.Layout.Column = [1 2];

% ==============================================================================
% 3-D SCENE
% ==============================================================================
% The panel clips the scene; a perspective camera inside the axes box
% would otherwise draw over the readout and the time plots
scenePanel = uipanel(main, 'BorderType', 'none', 'BackgroundColor', 'w');
scenePanel.Layout.Row = 1;  scenePanel.Layout.Column = 1;
ax = axes(uigridlayout(scenePanel, [1 1], 'BackgroundColor', 'w'));
hold(ax, 'on');  grid(ax, 'on');  box(ax, 'on');
set(ax, 'YDir', 'reverse', 'ZDir', 'reverse', 'XLim', lims(:,1)', ...
    'YLim', lims(:,2)', 'ZLim', lims(:,3)', 'DataAspectRatio', [1 1 1/ve]);
xlabel(ax, 'North (m)');  ylabel(ax, 'East (m)');  zlabel(ax, 'Down (m)');

% Sea surface, path shadow on the floor, full path and travelled path
patch(ax, lims([1 2 2 1],1), lims([1 1 2 2],2), [0 0 0 0], [0.55 0.75 0.95], ...
    'FaceAlpha', 0.15, 'EdgeColor', 'none', 'FaceLighting', 'none');
plot3(ax, pos(:,1), pos(:,2), repmat(zFloor, N, 1), 'Color', [0.82 0.82 0.82]);
plot3(ax, pos(:,1), pos(:,2), pos(:,3), 'Color', [0.7 0.75 0.85]);
plot3(ax, pos(1,1), pos(1,2), pos(1,3), 'o', 'MarkerSize', 6, ...
    'MarkerFaceColor', [0.2 0.7 0.3], 'MarkerEdgeColor', 'w');
hTrail = plot3(ax, NaN, NaN, NaN, 'Color', [0 0.35 0.8], 'LineWidth', 2, 'Tag', 'trail');
hDrop = plot3(ax, [NaN NaN], [NaN NaN], [NaN NaN], ':', 'Color', [0.4 0.4 0.4]);
hShadow = plot3(ax, NaN, NaN, zFloor, 'o', 'MarkerSize', 7, 'Tag', 'shadow', ...
    'MarkerFaceColor', [1 0.8 0.1], 'MarkerEdgeColor', [0.3 0.3 0.3]); % Finds the vehicle

% Waypoints and path between them, with the active leg highlighted
hActive = gobjects(0);
if hasPath
    plot3(ax, W(:,1), W(:,2), W(:,3), '--', 'Color', [0.85 0.3 0.2]);
    plot3(ax, W(1:nW,1), W(1:nW,2), W(1:nW,3), 'o', 'MarkerSize', 7, ...
        'MarkerFaceColor', [0.85 0.3 0.2], 'MarkerEdgeColor', 'w');
    text(ax, W(1:nW,1), W(1:nW,2), W(1:nW,3), "  WP" + (1:nW)', ...
        'FontSize', 9, 'Color', [0.6 0.15 0.1]);
    if ~isempty(kAct)
        hActive = plot3(ax, NaN, NaN, NaN, 'Color', [1 0.55 0], 'LineWidth', 2, ...
            'Tag', 'activeLeg');
    end
end

% Vehicle (body frame: x forward, y starboard, z down). hExag undoes the
% vertical exaggeration, so hVehicle can hold a pure rotation and scale
% (an hgtransform with shear may not render correctly).
hExag = hgtransform(ax, 'Matrix', makehgtform('scale', [1 1 1/ve]));
hVehicle = hgtransform(hExag, 'Tag', 'vehicle');
[hRudder, hStern] = buildVehicle(hVehicle, d.model);

light(ax, 'Style', 'infinite', 'Position', [0.3 -0.5 -1]);  % From above
light(ax, 'Style', 'infinite', 'Position', [-0.4 0.6 0.5], 'Color', [0.45 0.45 0.45]);

% ==============================================================================
% READOUT AND TIME PLOTS
% ==============================================================================
hHud = uilabel(side, 'Text', '', 'FontName', mono, 'FontSize', 12, ...
    'VerticalAlignment', 'top', 'WordWrap', 'off', ...
    'BackgroundColor', [0.95 0.96 0.98], 'Tag', 'readout');
hHud.Layout.Row = 1;

c1 = [0 0.35 0.8];  c2 = [0.85 0.33 0.1];
tAx = gobjects(3,1);  hCursor = gobjects(3,1);
for i = 1:3
    % Padded limits keep peaks off the frame, where they would look clipped
    tAx(i) = axes(side, 'FontSize', 9, 'Box', 'on', 'NextPlot', 'add', ...
        'XLim', [t(1) t(end)], 'YLimitMethod', 'padded');
    tAx(i).Layout.Row = i + 1;
    grid(tAx(i), 'on');
end

plot(tAx(1), t, eta(:,3), 'Color', c1, 'Tag', 'depth');
if any(isfinite(d.z_d))
    plot(tAx(1), t, d.z_d, '--', 'Color', c2, 'Tag', 'depthRef');
    title(tAx(1), ['Depth (m): ' coloured('true', c1) ', ' coloured('desired', c2)], ...
        'FontWeight', 'normal');
else
    title(tAx(1), 'Depth (m)', 'FontWeight', 'normal');
end
set(tAx(1), 'YDir', 'reverse');

% Yaw in [-180, 180) deg, broken at the wraps
yaw = rad2deg(ssa(eta(:,6)));
[tt, yy] = breakWraps(t, yaw);
plot(tAx(2), tt, yy, 'Color', c1, 'Tag', 'yaw');
wraps = any(abs(diff(yaw)) > 180);
if any(isfinite(d.psi_d))
    yawRef = rad2deg(ssa(d.psi_d));
    [tt, yy] = breakWraps(t, yawRef);
    plot(tAx(2), tt, yy, '--', 'Color', c2, 'Tag', 'yawRef');
    wraps = wraps || any(abs(diff(yawRef)) > 180);
    title(tAx(2), ['Yaw angle (deg): ' coloured('true', c1) ', ' coloured('desired', c2)], ...
        'FontWeight', 'normal');
else
    title(tAx(2), 'Yaw angle (deg)', 'FontWeight', 'normal');
end
if wraps; set(tAx(2), 'YLim', [-180 180], 'YTick', -180:90:180); end

if hasTrack
    plot(tAx(3), t, d.y_e, 'Color', c1, 'Tag', 'crossTrack');
    if hasVert
        plot(tAx(3), t, d.z_e, 'Color', c2, 'Tag', 'vertTrack');
        title(tAx(3), ['Tracking error (m): ' coloured('cross-track y_e', c1) ...
            ', ' coloured('vertical-track z_e', c2)], 'FontWeight', 'normal');
    else
        title(tAx(3), ['Tracking error (m): ' coloured('cross-track y_e', c1)], ...
            'FontWeight', 'normal');
    end
else
    plot(tAx(3), t, rad2deg(d.ui(:,1)), 'Color', c1, 'Tag', 'rudder');
    plot(tAx(3), t, rad2deg(d.ui(:,2)), 'Color', c2, 'Tag', 'stern');
    title(tAx(3), ['Fin angle (deg): ' coloured('rudder', c1) ', ' ...
        coloured('stern planes', c2)], 'FontWeight', 'normal');
end
xlabel(tAx(3), 'Time (s)');

for i = 1:3
    set(tAx(i).Children, 'HitTest', 'off');
    tAx(i).YLim = tAx(i).YLim;   % Freeze, so the cursor never rescales the plot
    hCursor(i) = plot(tAx(i), [t(1) t(1)], tAx(i).YLim, 'Color', [0.8 0.1 0.1], ...
        'LineWidth', 1.2, 'HitTest', 'off', 'Tag', 'cursor');
    disableDefaultInteractivity(tAx(i));
    tAx(i).Toolbar.Visible = 'off';
    tAx(i).ButtonDownFcn = @(src, ~) seek(src.CurrentPoint(1,1));
end

% ==============================================================================
% CONTROLS
% ==============================================================================
row = uigridlayout(controls, [1 5], 'ColumnWidth', {80, 60, 60, '1x', 'fit'}, ...
    'Padding', 0, 'BackgroundColor', 'w');
hPlay = uibutton(row, 'Text', 'Play', 'FontWeight', 'bold', 'Tag', 'play', ...
    'ButtonPushedFcn', @(~,~) togglePlay());
uibutton(row, 'Text', '-1 s', 'Tag', 'back', 'ButtonPushedFcn', @(~,~) step(-1));
uibutton(row, 'Text', '+1 s', 'Tag', 'forward', 'ButtonPushedFcn', @(~,~) step(1));
hSlider = uislider(row, 'Limits', [t(1) t(end)], 'Value', t(1), ...
    'MajorTicks', [], 'MinorTicks', [], 'Tag', 'time', ...
    'ValueChangingFcn', @onDrag, 'ValueChangedFcn', @(src, ~) seek(src.Value));
hTime = uilabel(row, 'Text', '', 'FontName', mono, 'FontSize', 12, 'Tag', 'clock');

row = uigridlayout(controls, [1 10], 'ColumnWidth', ...
    {'fit', 70, 'fit', 95, 90, 'fit', 60, 'fit', 60, '1x'}, ...
    'Padding', 0, 'BackgroundColor', 'w');
uilabel(row, 'Text', 'Speed');
uidropdown(row, 'Items', compose('%gx', speedOptions), 'ItemsData', speedOptions, ...
    'Value', speed, 'Tag', 'speed', 'ValueChangedFcn', @onSpeed);
uilabel(row, 'Text', 'View');
uidropdown(row, 'Items', viewNames, 'Value', viewMode, 'Tag', 'view', ...
    'ValueChangedFcn', @onView);
uibutton(row, 'Text', 'Reset view', 'Tag', 'resetView', 'ButtonPushedFcn', @(~,~) applyView());
uilabel(row, 'Text', 'Vehicle scale');
uieditfield(row, 'numeric', 'Limits', [0.01 Inf], 'Value', scale, 'Tag', 'scale', ...
    'ValueChangedFcn', @onScale);
uilabel(row, 'Text', 'Vert. exag.');
uidropdown(row, 'Items', compose('%gx', veOptions), 'ItemsData', veOptions, ...
    'Value', ve, 'Tag', 'vertExag', 'ValueChangedFcn', @onVerticalExaggeration);

tmr = timer('Name', 'auv1Visualiser', 'ExecutionMode', 'fixedSpacing', ...
    'Period', 0.03, 'BusyMode', 'drop', 'TimerFcn', @(~,~) onTick());
fig.DeleteFcn = @(~,~) onDelete();

applyView();   % Also draws the first frame
if nargout == 0; clear fig; end

% ==============================================================================
% NESTED FUNCTIONS (share the state above)
% ==============================================================================
    function showFrame(kNew)
        % Draws sample kNew in every part of the window that shows time
        k = max(1, min(N, kNew));
        p = pos(k,:);

        % Vehicle pose in exaggerated units (hExag scales back). D*R is the
        % attitude as seen with depth exaggerated; its nearest rotation keeps
        % the hull shape while lining the vehicle up with the exaggerated path.
        D = diag([1 1 ve]);
        [Us, ~, Vs] = svd(D * Rzyx(eta(k,4), eta(k,5), eta(k,6)));
        hVehicle.Matrix = [scale * (Us * Vs'), D * p'; 0 0 0 1];

        % Fin deflections: positive delta_r turns the trailing edge to
        % starboard (yaw to starboard), positive delta_s turns it down (pitch
        % nose down), matching the signs of Y_r, Z_s and tau in auv1.m, and
        % saturated at delta_max as in auv1.m
        hRudder.Matrix = finRotation(hRudder.UserData, 'zrotate', ...
            -min(max(d.ui(k,1), -deltaMax), deltaMax));
        hStern.Matrix = finRotation(hStern.UserData, 'yrotate', ...
            min(max(d.ui(k,2), -deltaMax), deltaMax));

        setLine(hTrail, pos([1:trailStride:k-1, k], :));
        set(hDrop, 'XData', [p(1) p(1)], 'YData', [p(2) p(2)], 'ZData', [p(3) zFloor]);
        set(hShadow, 'XData', p(1), 'YData', p(2));
        if ~isempty(hActive); setLine(hActive, W(kAct(k):kAct(k)+1,:)); end
        if strcmp(viewMode, 'Follow'); followCamera(); end

        set(hCursor, 'XData', [t(k) t(k)]);
        hSlider.Value = t(k);
        hTime.Text = sprintf('t = %.2f / %.2f s', t(k), t(end));
        hHud.Text = hudText();
    end

    function lines = hudText()
        e = eta(k,:);
        lines = {
            sprintf(' %s, %s', d.modeName, d.kinematicsName)
            ''
            sprintf(' Time          %8.2f s  (sample %d of %d)', t(k), k, N)
            sprintf(' North / East  %8.1f / %.1f m', e(1), e(2))
            sprintf(' Depth         %8.1f m%s', e(3), refText(d.z_d(k)))
            sprintf(' Speed U       %8.2f m/s', U(k))
            sprintf(' Roll          %8.1f deg', rad2deg(e(4)))
            sprintf(' Pitch         %8.1f deg%s', rad2deg(e(5)), refText(rad2deg(d.theta_d(k))))
            sprintf(' Yaw           %8.1f deg%s', rad2deg(ssa(e(6))), ...
                refText(rad2deg(ssa(d.psi_d(k)))))
            sprintf(' Rudder        %8.1f deg', rad2deg(d.ui(k,1)))
            sprintf(' Stern planes  %8.1f deg', rad2deg(d.ui(k,2)))
            sprintf(' Propeller     %8.0f rpm', d.ui(k,3))
            sprintf(' Current       %8.2f m/s towards %.0f deg', d.Vc(k), ...
                rad2deg(mod(d.betaVc(k), 2*pi)))
            };
        if ~isempty(kAct)
            kA = kAct(k);
            if kA < nW
                lines{end+1} = sprintf(' Waypoint      %5d -> %d (%.0f m to go)', ...
                    kA, kA + 1, norm(W(kA+1,:) - e(1:3)));
            else
                lines{end+1} = sprintf(' Waypoint      %5d (final, holding bearing)', kA);
            end
        end
        if hasTrack; lines{end+1} = sprintf(' Cross-track   %8.2f m', d.y_e(k)); end
        if hasVert;  lines{end+1} = sprintf(' Vert.-track   %8.2f m', d.z_e(k)); end
    end

    function followCamera()
        % Off the starboard quarter, 20 deg above the vehicle (z/ve undoes
        % the vertical exaggeration). Camera vectors are in data units, so
        % up is -z.
        dist = 4 * L * scale;
        b = eta(k,6) + pi - deg2rad(35);   % Bearing from vehicle to camera
        el = deg2rad(20);
        offset = dist * [cos(el) * cos(b), cos(el) * sin(b), -sin(el) / ve];
        set(ax, 'CameraTarget', pos(k,:), 'CameraPosition', pos(k,:) + offset, ...
            'CameraUpVector', [0 0 -1], 'CameraViewAngle', 35);
    end

    function applyView()
        % Cameras are set explicitly in NED, since view(az,el) is ambiguous
        % on reversed axes. Overview looks from the SSW, 25 deg above.
        centre = mean(lims, 1);
        far = 10 * hor;
        set(ax, 'CameraViewAngleMode', 'auto', 'Projection', 'orthographic', ...
            'Visible', 'on');
        switch viewMode
            case 'Overview'
                b = deg2rad(205);  el = deg2rad(25);
                set(ax, 'CameraTarget', centre, 'CameraUpVector', [0 0 -1], ...
                    'CameraPosition', centre + far * [cos(el) * cos(b), ...
                    cos(el) * sin(b), -sin(el) / ve]);
            case 'Top'
                set(ax, 'CameraTarget', centre, 'CameraUpVector', [1 0 0], ...
                    'CameraPosition', centre - [0 0 far / ve]);
            case 'Follow'
                % Grid and box seen up close in perspective are clutter
                set(ax, 'Projection', 'perspective', 'Visible', 'off');
        end
        if ~strcmp(viewMode, 'Follow')
            % The auto fit leaves a wide margin; use more of the axes. Setting
            % the angle also freezes the zoom while rotating.
            ax.CameraViewAngle = 0.8 * ax.CameraViewAngle;
        end
        showFrame(k);
    end

    function seek(tq)
        % Jumps to the logged sample nearest to time tq
        [~, kNew] = min(abs(t - tq));
        showFrame(kNew);
        if playing; clock0 = t(k); wall0 = tic; end   % Play on from here
    end

    function step(dt)
        % Moves by dt seconds. Callbacks must call this rather than
        % @() seek(t(k) + dt): an anonymous function keeps the k it was made with.
        seek(t(k) + dt);
    end

    function togglePlay()
        if playing
            stopPlayback();
        else
            if k == N; showFrame(1); end   % Replay from the start
            clock0 = t(k);  wall0 = tic;  playing = true;
            hPlay.Text = 'Pause';
            if strcmp(tmr.Running, 'off'); start(tmr); end
        end
    end

    function stopPlayback()
        playing = false;
        if isvalid(tmr) && strcmp(tmr.Running, 'on'); stop(tmr); end
        if isvalid(hPlay); hPlay.Text = 'Play'; end
    end

    function onTick()
        % Shows the last sample at or before the playback clock. limitrate
        % does not wait for the window to draw (each wait costs tens of ms);
        % frames it drops are replaced by the next one.
        if ~playing || ~isvalid(fig); return; end
        try
            tq = clock0 + speed * toc(wall0);
            if tq >= t(end)
                showFrame(N);
                stopPlayback();
                drawnow
            else
                showFrame(find(t <= tq, 1, 'last'));
                drawnow limitrate
            end
        catch err
            stopPlayback();
            warning('auv1Visualiser:playback', 'Playback stopped:\n%s', ...
                getReport(err, 'extended', 'hyperlinks', 'off'));
        end
    end

    function onDrag(~, evt)
        stopPlayback();
        seek(evt.Value);
    end

    function onSpeed(src, ~)
        speed = src.Value;
        if playing; clock0 = t(k); wall0 = tic; end
    end

    function onView(src, ~)
        viewMode = src.Value;
        applyView();
    end

    function onScale(src, ~)
        scale = src.Value;
        showFrame(k);
    end

    function onVerticalExaggeration(src, ~)
        ve = src.Value;
        ax.DataAspectRatio = [1 1 1/ve];
        hExag.Matrix = makehgtform('scale', [1 1 1/ve]);
        applyView();
    end

    function onKey(~, evt)
        dt = 1;
        if any(strcmp(evt.Modifier, 'shift')); dt = 10; end
        switch evt.Key
            case 'space';      togglePlay();
            case 'rightarrow'; step(dt);
            case 'leftarrow';  step(-dt);
            case 'home';       seek(t(1));
            case 'end';        seek(t(end));
        end
    end

    function onDelete()
        playing = false;
        if isvalid(tmr)
            stop(tmr);
            delete(tmr);
        end
    end

end % auv1Visualiser

% ==============================================================================
% LOCAL FUNCTIONS
% ==============================================================================
function d = loadRun(in)
% Validates the SIMauv1 output (struct or .mat file) and fills in missing
% optional fields with NaN, so the rest of the visualiser need not check.

if ischar(in) || isstring(in)
    file = char(in);
    S = load(file);
    if isfield(S, 't') && isfield(S, 'eta')
        in = S;
    else
        in = [];
        f = fieldnames(S);
        for i = 1:numel(f)
            if isstruct(S.(f{i})) && isfield(S.(f{i}), 't') && isfield(S.(f{i}), 'eta')
                in = S.(f{i});
                break
            end
        end
        if isempty(in)
            error('auv1Visualiser:badFile', ...
                'No SIMauv1 output (struct with fields t and eta) found in %s.', file);
        end
    end
end
if ~isstruct(in) || ~isfield(in, 't') || ~isfield(in, 'eta')
    error('auv1Visualiser:badInput', ['Input must be the simOut struct returned ' ...
        'by SIMauv1 (fields t and eta), or a .mat file containing it.']);
end

t = double(in.t(:));
eta = double(in.eta);
N = numel(t);
if N < 2 || size(eta,1) ~= N || size(eta,2) < 6
    error('auv1Visualiser:badInput', 'eta must be N-by-6 with N = numel(t) >= 2.');
end

% Show a diverged run up to the first non-finite sample
keep = (1:N)';
bad = find(~all(isfinite([t eta(:,1:6)]), 2), 1);
if ~isempty(bad)
    if bad < 3
        error('auv1Visualiser:badInput', 'The run has no finite data to show.');
    end
    warning('auv1Visualiser:nonFinite', ...
        'Non-finite states from t = %.2f s; showing the run up to there.', t(bad));
    keep = (1:bad-1)';
end
if any(diff(t(keep)) <= 0)
    error('auv1Visualiser:badInput', 't must be strictly increasing.');
end

d.t = t(keep);
d.eta = eta(keep, 1:6);
d.nu = column(in, {'nu'}, 6, N, keep);
d.ui = column(in, {'ui'}, 3, N, keep);
d.z_d = column(in, {'ref', 'z_d'}, 1, N, keep);
d.theta_d = column(in, {'ref', 'theta_d'}, 1, N, keep);
d.psi_d = column(in, {'ref', 'psi_d'}, 1, N, keep);
d.Vc = column(in, {'current', 'Vc'}, 1, N, keep);
d.betaVc = column(in, {'current', 'betaVc'}, 1, N, keep);
d.y_e = column(in, {'alos', 'y_e'}, 1, N, keep);
d.z_e = column(in, {'alos', 'z_e'}, 1, N, keep);

d.ControlFlag = NaN;
if isfield(in, 'ControlFlag'); d.ControlFlag = in.ControlFlag; end
modeNames = {'Depth/heading autopilot (PID)', 'Depth/heading autopilot (SMC)', ...
    '2.5-D ALOS path following', '3-D ALOS path following'};
if any(d.ControlFlag == 1:4)
    d.modeName = modeNames{d.ControlFlag};
else
    d.modeName = 'AUV1';
end
d.kinematicsName = 'unknown kinematics';
if isfield(in, 'KinematicsFlag') && any(in.KinematicsFlag == [1 2])
    names = {'Euler angles', 'unit quaternions'};
    d.kinematicsName = names{in.KinematicsFlag};
end

% Logged signals that the selected control method does not drive
if d.ControlFlag == 4; d.z_d(:) = NaN; end           % Desired depth unused
if any(d.ControlFlag == [1 2]); d.y_e(:) = NaN; d.z_e(:) = NaN; end

% Vehicle and guidance parameters
if isfield(in, 'P') && isfield(in.P, 'model')
    P = in.P;
else
    P = auv1Params();
end
d.model = P.model;
d.R_switch = 5;
if isfield(P, 'guidance') && isfield(P.guidance, 'R_switch')
    d.R_switch = P.guidance.R_switch;
end

% Waypoints are only followed in ControlFlag 3-4
d.wpt = [];
if isfield(in, 'wpt') && isfield(in.wpt, 'pos') && numel(in.wpt.pos.x) >= 2 ...
        && (isnan(d.ControlFlag) || d.ControlFlag >= 3)
    d.wpt = in.wpt;
end
end

function v = column(s, path, nCol, N, keep)
% Returns s.(path{1}).(path{2})... as numel(keep)-by-nCol, or NaN if it is
% absent. A field of the wrong size is not shown, with a warning.
v = NaN(numel(keep), nCol);
for i = 1:numel(path)
    if ~isstruct(s) || ~isfield(s, path{i}); return; end
    s = s.(path{i});
end
if isvector(s) && numel(s) == N && nCol == 1; s = s(:); end
if isnumeric(s) && size(s,1) == N && size(s,2) >= nCol
    v = double(s(keep, 1:nCol));
else
    warning('auv1Visualiser:fieldSize', '%s is %s, expected %d-by-%d; not shown.', ...
        strjoin(path, '.'), mat2str(size(s)), N, nCol);
end
end

function kAct = replayWaypoints(p, W, R_switch, ControlFlag, y_e, z_e)
% Replays the waypoint switching of ALOSpsi (ControlFlag 3) or ALOS3D (4) on
% the logged positions (SIMauv1 logs the state the guidance law sees), and
% recomputes the tracking errors the guidance law logged along the way. The
% formulas are those of ALOSpsi.m and ALOS3D.m. If the errors do not match,
% the replay is not what the simulation did, and kAct is returned empty.
kAct = [];
if ~any(ControlFlag == [3 4]) || ~any(isfinite(y_e)); return; end
n = size(W, 1);  N = size(p, 1);
kAct = ones(N, 1);
e = NaN(N, 2);              % Replayed [y_e z_e]
k = 1;
for i = 1:N
    xk = W(k,1);  yk = W(k,2);  zk = W(k,3);
    if k < n                % Next waypoint, or continue with the last bearing
        xn = W(k+1,1);  yn = W(k+1,2);  zn = W(k+1,3);
    else
        bearing = atan2(W(n,2) - W(n-1,2), W(n,1) - W(n-1,1));
        xn = W(n,1) + 1e10 * cos(bearing);  yn = W(n,2) + 1e10 * sin(bearing);
        zn = W(n,3);
    end
    pi_h = atan2(yn - yk, xn - xk);
    if ControlFlag == 3     % Along-track distance left to the next waypoint
        x_e = (p(i,1) - xk) * cos(pi_h) + (p(i,2) - yk) * sin(pi_h);
        e(i,1) = -(p(i,1) - xk) * sin(pi_h) + (p(i,2) - yk) * cos(pi_h);
        dist = sqrt((xn - xk)^2 + (yn - yk)^2);
        switchNow = dist - x_e < R_switch;
    else                    % Inside the circle of acceptance
        pi_v = atan2(-(zn - zk), sqrt((xn - xk)^2 + (yn - yk)^2));
        Rz = [cos(pi_h) -sin(pi_h) 0; sin(pi_h) cos(pi_h) 0; 0 0 1];
        Ry = [cos(pi_v) 0 sin(pi_v); 0 1 0; -sin(pi_v) 0 cos(pi_v)];
        ev = Ry' * Rz' * [p(i,1) - xk, p(i,2) - yk, p(i,3) - zk]';
        e(i,:) = ev(2:3)';
        switchNow = sqrt((xn - p(i,1))^2 + (yn - p(i,2))^2 + (zn - p(i,3))^2) < R_switch;
    end
    if switchNow && k < n; k = k + 1; end
    kAct(i) = k;
end

dev = abs(e(:,1) - y_e);
if ControlFlag == 4; dev = max(dev, abs(e(:,2) - z_e)); end
if ~all(dev <= 1e-6)        % Also fails on NaN
    warning('auv1Visualiser:waypointReplay', ['The waypoint replay does not ' ...
        'reproduce the logged tracking errors (first mismatch at sample %d); ' ...
        'the active waypoint is not shown.'], find(~(dev <= 1e-6), 1));
    kAct = [];
end
end

function [hRudder, hStern] = buildVehicle(parent, model)
% Builds the vehicle in its body frame (x forward, y starboard, z down, origin
% at the CO) under the hgtransform parent: hull of revolution, tail cone,
% propeller disc, a mast on top (shows roll) and cruciform tail fins. The fins
% sit under their own hgtransforms (hinge x-position in UserData).

L = model.L_auv;  R = model.D_auv / 2;
lNose = 0.15 * L;  lTail = 0.35 * L;  rEnd = 0.3 * R;
xNose0 = L/2 - lNose;  xTail0 = -L/2 + lTail;
th = linspace(0, 2*pi, 25);
surfOpts = {'EdgeColor', 'none', 'FaceLighting', 'gouraud', 'Clipping', 'off', ...
    'AmbientStrength', 0.45, 'DiffuseStrength', 0.8, 'SpecularStrength', 0.3};
revolve = @(xs, rs, color) surface(repmat(xs(:), 1, numel(th)), rs(:) * cos(th), ...
    rs(:) * sin(th), 'Parent', parent, 'FaceColor', color, surfOpts{:});

% Cylindrical mid-body with an elliptical nose
s = sin(linspace(0, pi/2, 14));
revolve([xTail0, xNose0 + s * lNose], [R, R * sqrt(max(0, 1 - s.^2))], [1 0.8 0.1]);

% Tail cone, cubic taper from R to rEnd
s = linspace(0, 1, 12);
rTail = @(s) R - (R - rEnd) * (3 * s.^2 - 2 * s.^3);
revolve(xTail0 - s * lTail, rTail(s), [0.25 0.25 0.27]);

% Propeller hub and translucent propeller disc
rProp = 0.07;
if isfield(model, 'D_prop'); rProp = model.D_prop / 2; end
xProp = -L/2 - 0.02 * L;
revolve([-L/2, xProp - 0.01 * L], [rEnd, 0.2 * rEnd], [0.15 0.15 0.15]);
patch('Parent', parent, 'XData', repmat(xProp, size(th)), 'YData', rProp * cos(th), ...
    'ZData', rProp * sin(th), 'FaceColor', [0.3 0.3 0.3], 'FaceAlpha', 0.35, ...
    'EdgeColor', [0.2 0.2 0.2], 'Clipping', 'off');

% Mast on top (-z), so roll is visible
bx = 0.12 * L + [0 0.04] * L;  by = [-0.004 0.004] * L;  bz = -R + [0.01 -0.05] * L;
[V, F] = box3(bx, by, bz);
patch('Parent', parent, 'Vertices', V, 'Faces', F, 'FaceColor', [0.2 0.2 0.22], ...
    'EdgeColor', 'none', 'Clipping', 'off');

% Tail fins: area S_fin each, root on the tail cone, hinge at 35% root chord
cRoot = 0.056 * L;  cTip = 0.0375 * L;
span = 0.055 * L;
if isfield(model, 'S_fin'); span = model.S_fin / ((cRoot + cTip) / 2); end
xHinge = -L/2 + 0.03 * L + 0.65 * cRoot;
rRoot = 0.9 * rTail((xTail0 - xHinge) / lTail);
sweep = 0.3 * cRoot;
xs = xHinge + [0.35 * cRoot, 0.35 * cRoot - sweep, 0.35 * cRoot - sweep - cTip, -0.65 * cRoot];
rs = [rRoot, rRoot + span, rRoot + span, rRoot];
finOpts = {'FaceColor', [0.12 0.12 0.14], 'EdgeColor', 'none', 'Clipping', 'off'};

hRudder = hgtransform(parent, 'UserData', xHinge, 'Tag', 'rudderFins'); % Top and bottom
hStern = hgtransform(parent, 'UserData', xHinge, 'Tag', 'sternFins');   % Port and starboard
for dirn = [-1 1]
    patch('Parent', hRudder, 'XData', xs, 'YData', 0 * rs, 'ZData', dirn * rs, finOpts{:});
    patch('Parent', hStern, 'XData', xs, 'YData', dirn * rs, 'ZData', 0 * rs, finOpts{:});
end
end

function setLine(h, P)
% Sets the vertices of line h to the rows of P
set(h, 'XData', P(:,1), 'YData', P(:,2), 'ZData', P(:,3));
end

function T = finRotation(xHinge, axisName, angle)
% Rotation by angle about a body axis through the hinge point (xHinge, 0, 0)
T = makehgtform('translate', [xHinge 0 0], axisName, angle, ...
    'translate', [-xHinge 0 0]);
end

function [V, F] = box3(x, y, z)
% Vertices and faces of an axis-aligned box spanning x, y, z = [min max]
[X, Y, Z] = ndgrid(x, y, z);
V = [X(:) Y(:) Z(:)];
F = [1 2 4 3; 5 6 8 7; 1 2 6 5; 3 4 8 7; 1 3 7 5; 2 4 8 6];
end

function [tt, yy] = breakWraps(t, y)
% Inserts NaN at +-180 deg wraps so the plotted angle has no vertical lines
jump = [false; abs(diff(y)) > 180];
idx = (1:numel(y))' + cumsum(jump);
tt = NaN(numel(y) + nnz(jump), 1);  yy = tt;
tt(idx) = t;  yy(idx) = y;
end

function s = coloured(str, rgb)
% TeX string str in colour rgb, for plot titles that double as legends
s = sprintf('{\\color[rgb]{%g %g %g}%s}', rgb, str);
end

function s = refText(x)
% Formats a reference value for the readout, or nothing if it is not available
if isfinite(x); s = sprintf('  (ref %.1f)', x); else; s = ''; end
end
