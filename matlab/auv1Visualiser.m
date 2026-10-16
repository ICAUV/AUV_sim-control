function fig = auv1Visualiser(simOut)
% auv1Visualiser(simOut) opens an animated 3-D replay of an SIMauv1 run: the
% travelled path, a 3-D model of the vehicle with moving rudders and stern
% planes, the waypoints and desired path (ControlFlag 3-4), a HUD readout and
% linked time plots.
%
% Usage:
%   simOut = SIMauv1(4, 2);                 % Run the simulator, keep the results
%   auv1Visualiser(simOut)                  % Replay them
%
%   save('run.mat', '-struct', 'simOut');   % Or save a run ...
%   auv1Visualiser('run.mat')               % ... and replay it later
%
%   fig = auv1Visualiser(...)               % Also return the figure handle
%
% Controls:
%   Play/Pause, step buttons and the time slider. Keyboard: Space, Left/Right
%   (1 s, Shift: 10 s), Home/End. Click a time plot to jump to that time.
%   View   Overview - whole mission, drag to rotate, scroll to zoom
%          Follow   - chase camera off the vehicle's starboard quarter
%          Top      - North-East plane seen from above
%   Scale  Vehicle size multiplier, so the 1.6 m hull is visible over km.
%          In Follow view it also sets the camera distance; 1 = true size.
%   Vert.  Vertical exaggeration of depth. The vehicle keeps its shape and is
%          tilted to line up with the exaggerated path; the HUD shows the
%          true angles.
%
% simOut fields (see the end of SIMauv1.m). Only t and eta are required:
%   t, eta, nu, ui, ref.{z_d,theta_d,psi_d}, current.{Vc,betaVc},
%   alos.{y_e,z_e}, wpt, P, ControlFlag, KinematicsFlag
%
% Plot frame: NED (x = North, y = East, z = Down). The y- and z-axes are
% reversed, which together is a rotation, so the scene is not mirrored.
%
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

% ==============================================================================
% WAYPOINTS AND SCENE EXTENT
% ==============================================================================
hasPath = ~isempty(d.wpt);
if hasPath
    W = [d.wpt.pos.x(:) d.wpt.pos.y(:) d.wpt.pos.z(:)]; % NED (m)
    nW = size(W, 1);
    kAct = activeWaypoints(pos, W, d.R_switch, d.ControlFlag);
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

% Playback state
k = 1;                 % Current sample
playing = false;
tSim0 = t(1);          % Sim time and wall clock at the last (re)start
wall0 = tic;

% ==============================================================================
% FIGURE AND 3-D SCENE
% ==============================================================================
fig = figure('Name', ['AUV1 replay - ' d.modeName], 'NumberTitle', 'off', ...
    'Color', 'w', 'Units', 'normalized', 'Position', [0.05 0.07 0.9 0.85], ...
    'KeyPressFcn', @onKey, 'DeleteFcn', @onDelete);

% The panel clips the scene; a perspective camera inside the axes box
% would otherwise draw over the HUD and time plots
scenePanel = uipanel(fig, 'Units', 'normalized', 'Position', [0 0.13 0.68 0.87], ...
    'BorderType', 'none', 'BackgroundColor', 'w');
ax = axes(scenePanel, 'Position', [0.07 0.07 0.89 0.87]);
hold(ax, 'on');  grid(ax, 'on');  box(ax, 'on');
set(ax, 'YDir', 'reverse', 'ZDir', 'reverse', 'XLim', lims(:,1)', ...
    'YLim', lims(:,2)', 'ZLim', lims(:,3)', 'DataAspectRatio', [1 1 1/ve]);
xlabel(ax, 'North (m)');  ylabel(ax, 'East (m)');  zlabel(ax, 'Down (m)');
title(ax, d.modeName, 'FontWeight', 'normal');

% Sea surface, path shadow on the floor, full path and travelled path
patch(ax, lims([1 2 2 1],1), lims([1 1 2 2],2), [0 0 0 0], [0.55 0.75 0.95], ...
    'FaceAlpha', 0.15, 'EdgeColor', 'none', 'FaceLighting', 'none');
floorPath = [pos(:,1:2), repmat(zFloor, N, 1)];
hFloorPath = plot3(ax, floorPath(:,1), floorPath(:,2), floorPath(:,3), 'Color', [0.82 0.82 0.82]);
hPath = plot3(ax, pos(:,1), pos(:,2), pos(:,3), 'Color', [0.7 0.75 0.85]);
plot3(ax, pos(1,1), pos(1,2), pos(1,3), 'o', 'MarkerSize', 6, ...
    'MarkerFaceColor', [0.2 0.7 0.3], 'MarkerEdgeColor', 'w');
hTrail = plot3(ax, pos(1,1), pos(1,2), pos(1,3), 'Color', [0 0.35 0.8], 'LineWidth', 2);
hDrop = plot3(ax, pos([1 1],1), pos([1 1],2), [pos(1,3) zFloor], ':', 'Color', [0.4 0.4 0.4]);
hShadow = plot3(ax, pos(1,1), pos(1,2), zFloor, 'o', 'MarkerSize', 7, ...
    'MarkerFaceColor', [1 0.8 0.1], 'MarkerEdgeColor', [0.3 0.3 0.3]); % Finds the vehicle

% Waypoints and desired path, with the active segment highlighted
% (densified, so clipToCamera keeps the part of each leg in front of the camera)
hActive = gobjects(0);  hDesired = gobjects(0);  Wd = zeros(0, 3);
if hasPath
    Wd = densify(W(1:nW,:), 100);
    hDesired = plot3(ax, Wd(:,1), Wd(:,2), Wd(:,3), '--', 'Color', [0.85 0.3 0.2]);
    plot3(ax, W(1:nW,1), W(1:nW,2), W(1:nW,3), 'o', 'MarkerSize', 7, ...
        'MarkerFaceColor', [0.85 0.3 0.2], 'MarkerEdgeColor', 'w');
    text(ax, W(1:nW,1), W(1:nW,2), W(1:nW,3), "  WP" + (1:nW)', ...
        'FontSize', 9, 'Color', [0.6 0.15 0.1]);
    hActive = plot3(ax, W(1:2,1), W(1:2,2), W(1:2,3), 'Color', [1 0.55 0], ...
        'LineWidth', 2);
end

% Vehicle (body frame: x forward, y starboard, z down). hExag undoes the
% vertical exaggeration, so hVehicle can hold a pure rotation and scale.
hExag = hgtransform(ax, 'Matrix', makehgtform('scale', [1 1 1/ve]));
hVehicle = hgtransform(hExag);
[hRudder, hStern] = buildVehicle(hVehicle, d.model);

light(ax, 'Style', 'infinite', 'Position', [0.3 -0.5 -1]);  % From above
light(ax, 'Style', 'infinite', 'Position', [-0.4 0.6 0.5], 'Color', [0.45 0.45 0.45]);

% ==============================================================================
% HUD AND LINKED TIME PLOTS
% ==============================================================================
hHud = uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.69 0.67 0.29 0.29], 'HorizontalAlignment', 'left', ...
    'FontName', get(groot, 'FixedWidthFontName'), 'FontSize', 10, ...
    'BackgroundColor', [0.95 0.96 0.98]);

c1 = [0 0.35 0.8];  c2 = [0.85 0.33 0.1];
tAx = gobjects(3,1);  hCursor = gobjects(3,1);
for i = 1:3
    tAx(i) = axes(fig, 'Position', [0.73 0.505 - 0.165*(i-1) 0.25 0.115], ...
        'FontSize', 8, 'Box', 'on', 'NextPlot', 'add');
    grid(tAx(i), 'on');
end

plot(tAx(1), t, eta(:,3), 'Color', c1);
if any(isfinite(d.z_d))
    plot(tAx(1), t, d.z_d, '--', 'Color', c2);
    title(tAx(1), 'Depth (m): true, desired (dashed)', 'FontWeight', 'normal');
else
    title(tAx(1), 'Depth (m)', 'FontWeight', 'normal');
end
set(tAx(1), 'YDir', 'reverse');

[tt, yy] = breakWraps(t, rad2deg(ssa(eta(:,6))));
plot(tAx(2), tt, yy, 'Color', c1);
if any(isfinite(d.psi_d))
    [tt, yy] = breakWraps(t, rad2deg(ssa(d.psi_d)));
    plot(tAx(2), tt, yy, '--', 'Color', c2);
end
title(tAx(2), 'Yaw angle (deg): true, desired (dashed)', 'FontWeight', 'normal');
set(tAx(2), 'YLim', [-180 180], 'YTick', -180:90:180);

if any(isfinite(d.y_e))
    plot(tAx(3), t, d.y_e, 'Color', c1);
    if any(isfinite(d.z_e))
        plot(tAx(3), t, d.z_e, 'Color', c2);
        title(tAx(3), 'Cross-track y_e (blue), vertical-track z_e (m)', 'FontWeight', 'normal');
    else
        title(tAx(3), 'Cross-track error y_e (m)', 'FontWeight', 'normal');
    end
else
    plot(tAx(3), t, rad2deg(d.ui(:,1)), 'Color', c1);
    plot(tAx(3), t, rad2deg(d.ui(:,2)), 'Color', c2);
    title(tAx(3), 'Rudder (blue), stern plane (deg)', 'FontWeight', 'normal');
end
xlabel(tAx(3), 'Time (s)');

for i = 1:3
    set(tAx(i).Children, 'HitTest', 'off');
    hCursor(i) = xline(tAx(i), t(1), 'Color', [0.8 0.1 0.1], 'LineWidth', 1.2, ...
        'HitTest', 'off');
    disableDefaultInteractivity(tAx(i));
    tAx(i).Toolbar.Visible = 'off';
    tAx(i).ButtonDownFcn = @onPlotClick;
end
linkaxes(tAx, 'x');
xlim(tAx(1), [t(1) t(end)]);

% ==============================================================================
% CONTROLS
% ==============================================================================
tRange = t(end) - t(1);
sliderStep = min([1 10] / tRange, 1);    % 1 s arrows, 10 s trough clicks
if sliderStep(1) >= sliderStep(2); sliderStep(1) = sliderStep(2) / 10; end
hSlider = uicontrol(fig, 'Style', 'slider', 'Units', 'normalized', ...
    'Position', [0.05 0.085 0.93 0.03], 'Min', t(1), 'Max', t(end), ...
    'Value', t(1), 'SliderStep', sliderStep, 'Callback', @(s,~) seekTime(s.Value));
addlistener(hSlider, 'ContinuousValueChange', @(s,~) seekTime(s.Value));

y = 0.025;  h = 0.04;
hPlay = uicontrol(fig, 'Style', 'pushbutton', 'String', 'Play', 'Units', 'normalized', ...
    'Position', [0.05 y 0.06 h], 'FontWeight', 'bold', 'Callback', @(~,~) togglePlay());
uicontrol(fig, 'Style', 'pushbutton', 'String', '<', 'Units', 'normalized', ...
    'Position', [0.115 y 0.03 h], 'Callback', @(~,~) seekTime(t(k) - 1));
uicontrol(fig, 'Style', 'pushbutton', 'String', '>', 'Units', 'normalized', ...
    'Position', [0.15 y 0.03 h], 'Callback', @(~,~) seekTime(t(k) + 1));
hTime = uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.19 y 0.15 h*0.7], 'HorizontalAlignment', 'left', ...
    'BackgroundColor', 'w', 'FontSize', 10);

addLabel('Speed', [0.35 y 0.04 h*0.7]);
uicontrol(fig, 'Style', 'popupmenu', 'Units', 'normalized', ...
    'Position', [0.39 y 0.06 h], 'String', compose('%gx', speedOptions), ...
    'Value', iSpeed, 'Callback', @onSpeed);
addLabel('View', [0.47 y 0.035 h*0.7]);
uicontrol(fig, 'Style', 'popupmenu', 'Units', 'normalized', ...
    'Position', [0.505 y 0.08 h], 'String', viewNames, 'Value', 1, ...
    'Callback', @onView);
uicontrol(fig, 'Style', 'pushbutton', 'String', 'Reset view', 'Units', 'normalized', ...
    'Position', [0.59 y 0.06 h], 'Callback', @(~,~) applyView());
addLabel('Vehicle scale', [0.67 y 0.07 h*0.7]);
hScale = uicontrol(fig, 'Style', 'edit', 'Units', 'normalized', ...
    'Position', [0.74 y 0.04 h], 'String', num2str(scale), 'Callback', @onScale);
addLabel('Vert. exag.', [0.80 y 0.06 h*0.7]);
uicontrol(fig, 'Style', 'popupmenu', 'Units', 'normalized', ...
    'Position', [0.86 y 0.05 h], 'String', compose('%gx', veOptions), ...
    'Value', find(veOptions == ve), 'Callback', @onVerticalExaggeration);

tmr = timer('Name', 'auv1Visualiser', 'ExecutionMode', 'fixedSpacing', ...
    'Period', 0.033, 'BusyMode', 'drop', 'TimerFcn', @onTick);

showFrame(1);
applyView();
if nargout == 0; clear fig; end

% ==============================================================================
% NESTED FUNCTIONS (share the state above)
% ==============================================================================
    function showFrame(kNew)
        k = max(1, min(N, kNew));
        p = pos(k,:);

        % Vehicle pose in exaggerated units (hExag scales back). Dk*R is the
        % attitude as seen with depth exaggerated; its nearest rotation keeps
        % the hull shape while lining the vehicle up with the exaggerated path.
        Dk = diag([1 1 ve]);
        [Us, ~, Vs] = svd(Dk * Rzyx(eta(k,4), eta(k,5), eta(k,6)));
        hVehicle.Matrix = [scale * (Us * Vs'), Dk * p'; 0 0 0 1];

        % Fin deflections: positive delta_r turns the trailing edge to
        % starboard (yaw to starboard), positive delta_s turns it down (pitch
        % nose down), matching the signs of Y_r, Z_s and tau in auv1.m
        hRudder.Matrix = finRotation(hRudder.UserData, 'zrotate', ...
            -min(max(d.ui(k,1), -deltaMax), deltaMax));
        hStern.Matrix = finRotation(hStern.UserData, 'yrotate', ...
            min(max(d.ui(k,2), -deltaMax), deltaMax));

        set(hDrop, 'XData', [p(1) p(1)], 'YData', [p(2) p(2)], 'ZData', [p(3) zFloor]);
        set(hShadow, 'XData', p(1), 'YData', p(2));
        seg = [];
        if hasPath; seg = densify(W(kAct(k):kAct(k)+1,:), 100); end
        if strcmp(viewMode, 'Follow')
            followCamera();
            setLine(hTrail, clipToCamera(pos(1:k,:)));
            setLine(hPath, clipToCamera(pos));
            setLine(hFloorPath, clipToCamera(floorPath));
            if hasPath
                setLine(hDesired, clipToCamera(Wd));
                setLine(hActive, clipToCamera(seg));
            end
        else
            setLine(hTrail, pos(1:k,:));
            if hasPath; setLine(hActive, seg); end
        end

        set(hCursor, 'Value', t(k));
        hSlider.Value = t(k);
        hTime.String = sprintf('t = %.1f / %.1f s', t(k), t(end));
        hHud.String = hudText();
    end

    function lines = hudText()
        e = eta(k,:);
        lines = {
            sprintf(' %s, %s', d.modeName, d.kinematicsName)
            ''
            sprintf(' Time          %8.1f s', t(k))
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
        if hasPath
            kA = kAct(k);
            if kA < nW
                lines{end+1} = sprintf(' Waypoint      %5d -> %d (%.0f m to go)', ...
                    kA, kA + 1, norm(W(kA+1,:) - e(1:3)));
            else
                lines{end+1} = sprintf(' Waypoint      %5d (final, holding bearing)', kA);
            end
        end
        if isfinite(d.y_e(k))
            lines{end+1} = sprintf(' Cross-track   %8.2f m', d.y_e(k));
        end
        if isfinite(d.z_e(k))
            lines{end+1} = sprintf(' Vert.-track   %8.2f m', d.z_e(k));
        end
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
                followCamera();
        end
        if ~strcmp(viewMode, 'Follow')
            % The auto fit leaves a wide margin; use more of the axes. Setting
            % the angle also freezes the zoom while rotating.
            ax.CameraViewAngle = 0.8 * ax.CameraViewAngle;
            % Undo any clipping from Follow view
            setLine(hPath, pos);
            setLine(hFloorPath, floorPath);
            if hasPath; setLine(hDesired, Wd); end
        end
        showFrame(k);
    end

    function P = clipToCamera(P)
        % Hides the points less than a fifth of the camera distance in front
        % of the camera. In perspective, a line passing behind the camera
        % may not be drawn at all, so lines are cut before they get there.
        % Distances are in display units (depth exaggerated by ve).
        c = ax.CameraPosition;
        v = (ax.CameraTarget - c) .* [1 1 ve];
        ahead = ((P - c) .* [1 1 ve]) * (v' / norm(v));
        P(ahead < 0.2 * norm(v), :) = NaN;
    end

    function seekTime(tq)
        [~, kNew] = min(abs(t - tq));
        showFrame(kNew);
        if playing; tSim0 = t(k); wall0 = tic; end % Continue from here
    end

    function togglePlay()
        if playing
            stopPlayback();
        else
            if k >= N; showFrame(1); end   % Replay from the start
            tSim0 = t(k);  wall0 = tic;  playing = true;
            hPlay.String = 'Pause';
            if strcmp(tmr.Running, 'off'); start(tmr); end
        end
    end

    function stopPlayback()
        playing = false;
        if isvalid(tmr) && strcmp(tmr.Running, 'on'); stop(tmr); end
        if isvalid(hPlay); hPlay.String = 'Play'; end
    end

    function onTick(~, ~)
        try
            if ~playing || ~isvalid(fig); return; end
            tq = tSim0 + speed * toc(wall0);   % Sim time follows the wall clock
            if tq >= t(end)
                showFrame(N);
                stopPlayback();
            else
                [~, kNew] = min(abs(t - tq));
                showFrame(kNew);
            end
            drawnow limitrate
        catch err
            stopPlayback();
            warning('auv1Visualiser:playback', 'Playback stopped: %s', err.message);
        end
    end

    function onSpeed(src, ~)
        speed = speedOptions(src.Value);
        if playing; tSim0 = t(k); wall0 = tic; end
    end

    function onView(src, ~)
        viewMode = viewNames{src.Value};
        applyView();
    end

    function onScale(src, ~)
        val = str2double(src.String);
        if isfinite(val) && val > 0
            scale = val;
            showFrame(k);
        end
        src.String = num2str(scale);
    end

    function onVerticalExaggeration(src, ~)
        ve = veOptions(src.Value);
        ax.DataAspectRatio = [1 1 1/ve];
        hExag.Matrix = makehgtform('scale', [1 1 1/ve]);
        showFrame(k);
        applyView();
    end

    function onPlotClick(src, ~)
        seekTime(src.CurrentPoint(1,1));
    end

    function onKey(~, evt)
        step = 1;
        if any(strcmp(evt.Modifier, 'shift')); step = 10; end
        switch evt.Key
            case 'space';      togglePlay();
            case 'rightarrow'; seekTime(t(k) + step);
            case 'leftarrow';  seekTime(t(k) - step);
            case 'home';       seekTime(t(1));
            case 'end';        seekTime(t(end));
        end
    end

    function onDelete(~, ~)
        playing = false;
        if isvalid(tmr)
            stop(tmr);
            delete(tmr);
        end
    end

    function addLabel(str, position)
        uicontrol(fig, 'Style', 'text', 'String', str, 'Units', 'normalized', ...
            'Position', position, 'HorizontalAlignment', 'right', ...
            'BackgroundColor', 'w');
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
% Returns s.(path{1}).(path{2})... as N-by-nCol (rows keep), or NaN if absent.
v = NaN(numel(keep), nCol);
for i = 1:numel(path)
    if ~isstruct(s) || ~isfield(s, path{i}); return; end
    s = s.(path{i});
end
if isvector(s) && numel(s) == N && nCol == 1; s = s(:); end
if size(s,1) == N && size(s,2) >= nCol
    v = double(s(keep, 1:nCol));
end
end

function kAct = activeWaypoints(p, W, R_switch, ControlFlag)
% Replays the waypoint switching of ALOSpsi (ControlFlag 3) or ALOS3D (4) on
% the logged positions; these are the positions the guidance law saw, since
% SIMauv1 logs the state before each update.
n = size(W, 1);
kAct = ones(size(p,1), 1);
k = 1;
for i = 1:size(p,1)
    if k < n
        if ControlFlag == 3 % Along-track distance left to the next waypoint
            pi_h = atan2(W(k+1,2) - W(k,2), W(k+1,1) - W(k,1));
            x_e = (p(i,1) - W(k,1)) * cos(pi_h) + (p(i,2) - W(k,2)) * sin(pi_h);
            dist = sqrt((W(k+1,1) - W(k,1))^2 + (W(k+1,2) - W(k,2))^2);
            switchNow = dist - x_e < R_switch;
        else                % Inside the circle of acceptance
            switchNow = sqrt((W(k+1,1) - p(i,1))^2 + (W(k+1,2) - p(i,2))^2 ...
                + (W(k+1,3) - p(i,3))^2) < R_switch;
        end
        if switchNow; k = k + 1; end
    end
    kAct(i) = k;
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

hRudder = hgtransform(parent, 'UserData', xHinge);   % Top and bottom
hStern = hgtransform(parent, 'UserData', xHinge);    % Port and starboard
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

function Q = densify(P, n)
% Splits each segment of the polyline P (one point per row) into n pieces,
% so clipping at the camera in Follow view drops only the part behind it.
s = (0:n-1)' / n;
Q = zeros((size(P,1) - 1) * n + 1, 3);
for i = 1:size(P,1) - 1
    Q((i-1)*n + (1:n), :) = P(i,:) + s .* (P(i+1,:) - P(i,:));
end
Q(end,:) = P(end,:);
end

function [tt, yy] = breakWraps(t, y)
% Inserts NaN at +-180 deg wraps so the plotted angle has no vertical lines
jump = [false; abs(diff(y)) > 180];
idx = (1:numel(y))' + cumsum(jump);
tt = NaN(numel(y) + nnz(jump), 1);  yy = tt;
tt(idx) = t;  yy(idx) = y;
end

function s = refText(x)
% Formats a reference value for the HUD, or nothing if it is not available
if isfinite(x); s = sprintf('  (ref %.1f)', x); else; s = ''; end
end
