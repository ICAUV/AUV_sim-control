function simOut = SIMauv1(ControlFlag, KinematicsFlag, path)
% This script simulates the Remus 100 Autonomous Underwater Vehicle (AUV)
% under depth and heading control while exposed to ocean currents. It supports
% both Euler angle and unit quaternion kinematics representations and features
% advanced control strategies including PID pole-placement, integral sliding
% mode control for heading, and Adaptive Line-of-Sight (ALOS) guidance for 2.5-D
% and 3-D path following. For 2.5-D path following the desired pitch angle is
% computed using a gradient-based outer-loop depth control law (gradient descent
% on the depth tracking error).
%
% Usage:
%   SIMauv1                              : Select options in a dialog box
%   SIMauv1(ControlFlag, KinematicsFlag) : ControlFlag = 1-4 (see dialog),
%                                          KinematicsFlag = 1 (Euler angles)
%                                          or 2 (unit quaternions)
%   SIMauv1(ControlFlag, KinematicsFlag, path) : Follow path (struct as
%                                          returned by pathParams.m) instead
%                                          of pathParams()
%   simOut = SIMauv1(...)                : Also return the logged results as a
%                                          struct, e.g. to save the run and
%                                          replay it later with auv1Visualiser
%
% Each run ends by opening its replay (auv1Visualiser), and a new run closes
% the previous replay along with the other figures.
%
% Dependencies:
%   auv1.m                - Dynamics of the AUV1 vehicle
%   auv1Params.m          - Model, control system and guidance parameters
%   pathParams.m          - Path to follow: waypoints and step commands
%   navigation/           - Navigation commands and reference propagation
%   control/              - Depth and tracking (fin) controllers, actuator inputs
%   physics/              - Measurements, ocean current and state propagation
%   integralSMCheading.m  - Integral sliding mode control for heading
%   refModel.m            - Reference model for autopilot systems
%   ALOSpsi.m             - 2-D ALOS guidance algorithm for path following
%   ALOS3D.m              - 3-D ALOS guidance algorithm for path following
%   LOSobserver.m         - Observer for LOS guidance 
%   q2euler.m, ssa.m      - Utilities for quaternion to Euler conversion
%                           and angle wrapping
%   auv1Visualiser.m      - Replay of the run
%
% Simulink Models:
%   demoAUVdepthHeadingControl.slx : Depth and heading control.
%
% References:
%   E. M. Coates and T. I. Fossen (2026). A Spherical Amplitude–Phase Formulation 
%       for 3-D Adaptive Line-of-Sight (ALOS) Guidance with USGES Stability
%       Guarantees, Automatica 194, 113149S. 
%   T. I. Fossen & P. Aguiar (2024). A Uniform Semiglobal Exponential  
%      Stable Adaptive Line-of-Sight (ALOS) Guidance Law for 3-D Path 
%      Following. Automatica, 163, 111556. 
%      https://doi.org/10.1016/j.automatica.2024.111556
%
% Author: Thor I. Fossen
% Date: 2021-06-28
% Revisions:
%   2022-02-01: Autopilots redesign.
%   2022-05-06: Tuning update for remus100 dynamics.
%   2022-05-08: Support for quaternion kinematics.
%   2024-04-02: Simulation options for 3-D ALOS path following (Fossen and 
%               Aguiar 2024).
%   2024-04-21: Extended compatibility with GNU Octave.
%   2024-06-07: Included display of vehicle data using displayVehicleData.m.
%   2024-07-10: Improved numerical accuracy by replacing Euler's method
%               with RK4 for the Euler angle representation.
%   2025-04-25: Improved logic and minor updates.
%   2025-05-13: Crab angle plots based on spherical amplitude-phase 
%               representation (Coates and Fossen 2025).
%   2025-10-06: Redesigned the depth controller using outer-loop gradient descent.
%   2025-12-07: Added control method for gradient-based depth control and 2.5-D ALOS
%               (Fossen and Coates, 2026).

clear integralSMCheading ALOS3D ALOSpsi; % Clear persistent states
close all;                               % Close all open figure windows
delete(findall(0, 'Type', 'figure', 'Tag', 'auv1Visualiser')); % And the last replay

% ==============================================================================
% SIMULATOR CONFIGURATION
% ==============================================================================
h = 0.05;                           % Sampling time (s)
T_final = 1400;	                    % Final simulation time (s)

if nargin < 2
    [ControlFlag, KinematicsFlag] = controlMethod();
end

P = auv1Params();                   % Model, control and guidance parameters
if nargin < 3; path = pathParams(); end
P.path = path;                      % Path to follow: waypoints and step commands
wpt = P.path.wpt;

% Initialize position and orientation
xn = 0; yn = 0; zn = 0;             % Initial North-East-Down positions (m)
phi = 0; theta = 0;                 % Initial Euler angles (radians)
psi = atan2(wpt.pos.y(2) - wpt.pos.y(1), ...
    wpt.pos.x(2) - wpt.pos.x(1));   % Yaw angle towards next waypoint
U = 1;                              % Initial speed (m/s)

% State vector initialization
if KinematicsFlag == 1  % Euler angles
    x = [U; zeros(5,1); xn; yn; zn; phi; theta; psi];
else % Unit quaternions
    quat = euler2q(phi, theta, psi);
    x = [U; zeros(5,1); xn; yn; zn; quat];
end

% Initialize ocean current parameters
Vc = 0.5;                      % Horizontal speed (m/s)
betaVc = deg2rad(30);          % Horizontal direction (radians)
wc = 0.1;                      % Vertical speed (m/s)

% Initialize actuator states (control/actuatorInputs.m)
act.n_p = 1000;                    % Initial propeller speed (RPM)
act.n_d = 1300;                    % Desired propeller speed (RPM)

% AUV1 actuators (P.model.actuators = 'auv1'): the controller outputs are
% mixed to four fin PWM signals with a servo slew-rate limit, the stern
% thruster runs at a constant PWM with a first-order lag, and the bow
% thruster is held at neutral (blending is not part of this scenario).
isAUV1 = strcmp(P.model.actuators, 'auv1');
if isAUV1
    pwm_centre = P.model.pwm_centre;       % Neutral PWM (us)
    pwm_range = P.model.pwm_range;         % PWM range at full effort (us)
    act.fin_angle = zeros(4,1);            % Fin angles [top bottom left right] (rad)
    stern_effort = 0.5;                    % Stern thruster effort (-1..1)
    act.pwm_stern_cmd = pwm_centre + stern_effort * pwm_range; % 1700 us
    act.pwm_stern = pwm_centre;            % Stern PWM after the thruster lag (us)
    act.pwm_bow = pwm_centre;              % Bow PWM, held at neutral (us)
end

% Time vector initialization
t = 0:h:T_final;                % Time vector from 0 to T_final          
nTimeSteps = length(t);         % Number of time steps

% ==============================================================================
% CONTROL SYSTEM CONFIGURATION
% ==============================================================================
% Parameters are defined in auv1Params.m and the path in pathParams.m

% Desired states for the tracking controller
ref.z_d = zn;  ref.zdot_d = 0;               % Initial depth references
ref.theta_d = 0;  ref.q_d = 0;               % Initial pitch references
ref.psi_d = psi;  ref.r_d = 0;  ref.a_d = 0; % Initial yaw references

% Integral states for autopilots
z_int = 0;                     % Integral state for depth control
theta_int = 0;                 % Integral state for pitch control

LP.z_d = makeLowPass(P.ctrl.wn_d_z, h); % Low-pass filter for the depth command

% ==============================================================================
% INPUT RANGE CHECK
% ==============================================================================
rangeCheck(act.n_d, 0, P.model.n_max);
rangeCheck(P.path.step.z, 0, P.model.z_max);
rangeCheck(U, 0, 5);

% ==============================================================================
% MAIN LOOP
% ==============================================================================
simData = zeros(nTimeSteps, length(x) + 10); % Preallocate table for simulation data
alosData = zeros(nTimeSteps, 4); % Preallocate table for ALOS guidance data

for i = 1:nTimeSteps

    y = measurements(x);                        % Measurements

    % Navigation: depth, heading and pitch commands at t(i)
    [cmd, alosData(i,:)] = navigation(t(i), y, ControlFlag, P.path, P.guidance, h);
    if ControlFlag <= 2
        ref.z_d = LP.z_d(cmd.z_ref);            % LP filtering the depth command
    end

    % References used by the autopilots at t(i). They are propagated to
    % t(i+1) below, so they are logged from here.
    ref_k = [ref.z_d ref.theta_d ref.psi_d ref.r_d];

    % Control: fin angles at t(i), the references at t(i+1), actuator inputs
    [delta_r, delta_s] = trackingController(y, ref, theta_int, ControlFlag, ...
        P.ctrl, P.model.delta_max, h);
    if ControlFlag <= 3
        ref.theta_d = depthController(y, ref, z_int, ControlFlag, P.ctrl, h);
    end
    ref = updateReferences(ref, cmd, ControlFlag, P.ctrl, P.guidance, h);
    [ui, ui_log, act] = actuatorInputs(delta_r, delta_s, act, P.model, h);

    % Physics: ocean current, then the state at t(i+1)
    if ControlFlag == 4
        [Vc, betaVc] = oceanCurrent(t(i), Vc, betaVc, h);
    end

    simData(i,:) = [ref_k Vc betaVc wc ui_log' x']; % Store simulation data in a table

    x = physicsStep(x, ui, Vc, betaVc, wc, P.model, h);

    % Euler's integration method (k+1)
    z_int = z_int + h * ( y.zn - ref.z_d );
    theta_int = theta_int + h * ssa( y.theta - ref.theta_d );

end

theta = y.theta; % Last measured pitch angle, used by the Euler-angle crab-angle plots

% ==============================================================================
% PLOTS
% ==============================================================================
scrSz = get(0, 'ScreenSize'); % Returns [left bottom width height]
legendLocation = 'best'; legendSize = 11;
if isoctave; legendLocation = 'northeast'; end

% simData = [z_d theta_d psi_d r_d Vc betaVc wc ui' x']
z_d     = simData(:,1);
theta_d = simData(:,2);
psi_d   = simData(:,3);
r_d     = simData(:,4);
Vc      = simData(:,5);
betaVc  = simData(:,6);
wc      = simData(:,7);
ui       = simData(:,8:10);
nu      = simData(:,11:16);
u = nu(:,1); v = nu(:,2); w = nu(:,3);
p = nu(:,4); q = nu(:,5); r = nu(:,6);

if (KinematicsFlag == 1) % Euler angle representation
    eta = simData(:,17:22);
else % Transform the unit quaternions to Euler angles
    quaternion = simData(:,20:23);
    phi = zeros(nTimeSteps,1); theta = zeros(nTimeSteps,1); psi = zeros(nTimeSteps,1);
    for i = 1:length(t)
        [phi(i,1),theta(i,1),psi(i,1)] = q2euler(quaternion(i,:));
    end
    eta = [simData(:,17:19) phi theta psi];
end

% alosData = [y_e z_e alpha_c_hat beta_c_hat]
y_e = alosData(:,1);
z_e = alosData(:,2);
alpha_c_hat = alosData(:,3);
beta_c_hat = alosData(:,4);

% Ocean current velocities 
uc = Vc .* cos(betaVc);
vc = Vc .* sin(betaVc);

% Crab angles, AoA and SSA
U = sqrt(u.^2+v.^2+w.^2); % Speed
gamma = asin( (u.*sin(theta)-(v.*sin(phi)+w.*cos(phi)).*cos(theta)) ./ U );
alpha_c = theta - gamma; % Horizontal crab angle
beta_c = atan2(v.*cos(phi)-w.*sin(phi), ...
    u.*cos(theta)+(v.*sin(phi)+w.*cos(phi)).*sin(theta)); % Vertical crab angle
alpha = asin( (w-wc) ./ (u-uc) ); % AoA
beta  = atan2( (v-vc), (u-uc) ); % SSA

% ------------------------------------------------------------------------------
% Generalized velocity
% ------------------------------------------------------------------------------
figure(1);
if ~isoctave; set(gcf,'Position',[1, 1, scrSz(3)/3, scrSz(4)]); end
subplot(611),plot(t,u)
xlabel('Time (s)'),title('Surge velocity (m/s)'),grid
subplot(612),plot(t,v)
xlabel('Time (s)'),title('Sway velocity (m/s)'),grid
subplot(613),plot(t,w)
xlabel('Time (s)'),title('Heave velocity (m/s)'),grid
subplot(614),plot(t,(180/pi)*p)
xlabel('Time (s)'),title('Roll rate (deg/s)'),grid
subplot(615),plot(t,(180/pi)*q)
xlabel('Time (s)'),title('Pitch rate (deg/s)'),grid
subplot(616),plot(t,(180/pi)*r)
xlabel('Time (s)'),title('Yaw rate (deg/s)'),grid
set(findall(gcf,'type','line'),'linewidth',1.5)
set(findall(gcf,'type','text'),'FontSize',12)
set(findall(gcf,'type','legend'),'FontSize',legendSize)

% ------------------------------------------------------------------------------
% Heave position and Euler angles
% ------------------------------------------------------------------------------
figure(2);
if ~isoctave; set(gcf,'Position',[scrSz(3)/3, 1, scrSz(3)/3, scrSz(4)]); end
if ControlFlag == 3 || ControlFlag == 4; z_d = eta(:,3); end
subplot(411),plot(t,eta(:,3),t,z_d,'-.')
xlabel('Time (s)'),title('Heave position (m)'),grid
legend('True','Desired')
subplot(412),plot(t,rad2deg(eta(:,4)))
xlabel('Time (s)'),title('Roll angle (deg)'),grid
subplot(413),plot(t,rad2deg(eta(:,5)),t,rad2deg(theta_d),'-.')
xlabel('Time (s)'),title('Pitch angle (deg)'),grid
legend('True','Desired')
subplot(414),plot(t,rad2deg(eta(:,6)),t,rad2deg(psi_d),'-.')
xlabel('Time (s)'),title('Yaw angle (deg)'),grid
legend('True','Desired')
set(findall(gcf,'type','line'),'linewidth',1.5)
set(findall(gcf,'type','text'),'FontSize',12)
set(findall(gcf,'type','legend'),'FontSize',legendSize)

% ------------------------------------------------------------------------------
% Control signals
% ------------------------------------------------------------------------------
figure(3);
if ~isoctave; set(gcf,'Position',[2*scrSz(3)/3,scrSz(4)/2,scrSz(3)/3,scrSz(4)/2]);end
subplot(311),plot(t,rad2deg(ui(:,1)))
xlabel('Time (s)'),title('Rudder command \delta_r (deg)'),grid
subplot(312),plot(t,rad2deg(ui(:,2)))
xlabel('Time (s)'),title('Stern-plane command \delta_s (deg)'),grid
subplot(313),plot(t,ui(:,3))
if isAUV1
    xlabel('Time (s)'),title('Stern thruster PWM (\mus)'),grid
else
    xlabel('Time (s)'),title('Propeller speed command n (rpm)'),grid
end
set(findall(gcf,'type','line'),'linewidth',1.5)
set(findall(gcf,'type','text'),'FontSize',12)

% ------------------------------------------------------------------------------
% Ocean currents and speed
% ------------------------------------------------------------------------------
figure(4);
if ~isoctave; set(gcf,'Position',[2*scrSz(3)/3,1,scrSz(3)/3,scrSz(4)/2]);end
subplot(311),plot(t,sqrt(nu(:,1).^2+nu(:,2).^2),t,Vc)
xlabel('Time (s)'),grid
legend('Vehicle horizontal speed (m/s)','Ocean current horizontal speed (m/s)',...
    'Location',legendLocation)
subplot(312),plot(t,nu(:,3),t,wc)
xlabel('Time (s)'),grid
legend('Vehicle heave velocity (m/s)','Ocean current heave velocity (m/s)',...
    'Location',legendLocation)
subplot(313),plot(t,rad2deg(betaVc),'r')
xlabel('Time (s)'),grid
legend('Ocean current direction (deg)','Location',legendLocation)
set(findall(gcf,'type','line'),'linewidth',1.5)
set(findall(gcf,'type','text'),'FontSize',12)
set(findall(gcf,'type','legend'),'FontSize',legendSize)

% ------------------------------------------------------------------------------
% Crab angles, SSA and AoA
% ------------------------------------------------------------------------------
if ControlFlag == 4 % 3-D ALOS
    figure(5);
    if ~isoctave; set(gcf,'Position',[100,scrSz(4)/2,scrSz(3)/3,scrSz(4)]); end
    subplot(311)
    plot(t,rad2deg(alpha),'g',t,rad2deg(alpha_c),'b',...
        t,rad2deg(alpha_c_hat),'r')
    title('Vertical crab angle and AoA (deg)')
    xlabel('Time (s)')
    grid
    legend('\alpha Angle of attack (AoA)','\alpha_c Vertical crab angle','\alpha_c ALOS estimate','Location',legendLocation)
    subplot(312)
    plot(t,rad2deg(beta),'g',t,rad2deg(beta_c),'b',...
        t,rad2deg(beta_c_hat),'r')
    title('Horizontal crab angle and SSA (deg)')
    xlabel('Time (s)')
    grid
    legend('\beta Sideslip angle (SSA)','\beta_c Horizontal crab angle','\beta_c ALOS estimate','Location',legendLocation)
    subplot(313)
    plot(t,y_e,t,z_e)
    title('Tracking errors (m)'),grid
    xlabel('Time (s)')
    legend('Cross-track error y_e^p','Vertical-track error z_e^p')
    set(findall(gcf,'type','line'),'linewidth',1.5)
    set(findall(gcf,'type','text'),'FontSize',12)
    set(findall(gcf,'type','legend'),'FontSize',legendSize)
end

% ------------------------------------------------------------------------------
% 2-D position plots with waypoints
% ------------------------------------------------------------------------------
if ControlFlag == 3 || ControlFlag == 4 % 2.5-D and 3-D ALOS
    figure(6);
    if ~isoctave;set(gcf,'Position',[300,200,scrSz(3)/3,scrSz(4)/2]);end
    subplot(211);
    plot(eta(:,2), eta(:,1));
    hold on;
    plot(wpt.pos.y, wpt.pos.x, 'rx', 'MarkerSize', 10);
    hold off;
    xlabel('East');
    ylabel('North');
    title('North-East plot (m)');
    xlim([0, 2500]);
    axis('equal');
    grid on;
    subplot(212);
    plot(eta(:,2), eta(:,3));
    hold on;
    plot(wpt.pos.y, wpt.pos.z, 'rx', 'MarkerSize', 10);
    hold off;
    xlim([0, 2500]);
    ylim([0, 150]);
    xlabel('East');
    ylabel('Down');
    title('Down-East plot (m)');
    grid on;
    legend('Actual path', 'Waypoints', 'Location', legendLocation);
    set(findall(gcf, 'type', 'line'), 'LineWidth', 1.5);
    set(findall(gcf, 'type', 'text'), 'FontSize', 12);
    set(findall(gcf, 'type', 'legend'), 'FontSize', legendSize);
end

% ------------------------------------------------------------------------------
% 3-D position plot with waypoints
% ------------------------------------------------------------------------------
if ControlFlag == 4 % 3-D ALOS
    figure(7);
    plot3(eta(:,2),eta(:,1),eta(:,3))
    hold on;
    plot3(wpt.pos.y, wpt.pos.x, wpt.pos.z, 'ro', 'MarkerSize', 15);
    hold off
    title('North-East-Down plot (m)')
    xlabel('East'); ylabel('North'); zlabel('Down');
    legend('Actual path','Waypoints','Location',legendLocation),grid
    set(gca, 'ZDir', 'reverse');
    set(findall(gcf,'type','line'),'linewidth',1.5)
    set(findall(gcf,'type','text'),'FontSize',12)
    set(findall(gcf,'type','legend'),'FontSize',legendSize)
    view(-25, 30);  % view(AZ,EL)
end

% ==============================================================================
% SIMULATION OUTPUT AND REPLAY (see auv1Visualiser.m)
% ==============================================================================
out.t = t(:);                           % Time (s)
out.h = h;                              % Sampling time (s)
out.ControlFlag = ControlFlag;
out.KinematicsFlag = KinematicsFlag;
out.eta = eta;                          % [x y z phi theta psi], NED (m, rad)
out.nu = nu;                            % [u v w p q r] (m/s, rad/s)
out.ui = ui;                            % [delta_r delta_s n] applied (rad, rad, RPM),
                                        % stern thruster PWM (us) for AUV1 actuators
out.ref.z_d = simData(:,1);             % Desired depth (m), ControlFlag 1-3
out.ref.theta_d = theta_d;              % Desired pitch angle (rad)
out.ref.psi_d = psi_d;                  % Desired yaw angle (rad)
out.ref.r_d = r_d;                      % Desired yaw rate (rad/s)
out.current.Vc = Vc;                    % Horizontal current speed (m/s)
out.current.betaVc = betaVc;            % Horizontal current direction (rad)
out.current.wc = wc;                    % Vertical current speed (m/s)
out.alos.y_e = y_e;                     % Cross-track error (m), ControlFlag 3-4
out.alos.z_e = z_e;                     % Vertical-track error (m), ControlFlag 4
out.alos.alpha_c_hat = alpha_c_hat;
out.alos.beta_c_hat = beta_c_hat;
out.wpt = wpt;                          % Waypoints (m), ControlFlag 3-4
out.P = P;                              % Model, control and guidance parameters
if nargout > 0; simOut = out; end       % Not shown as ans when not asked for

% Replay of the run. If it fails, the results above are still returned.
if ~isoctave                            % The replay needs uifigure
    try
        auv1Visualiser(out);
    catch err
        warning('SIMauv1:replay', 'The replay could not be opened: %s', err.message);
    end
end

end % SIMauv1

% ==============================================================================
% FUNCTIONS
% ==============================================================================
function [ControlFlag, KinematicsFlag] = controlMethod()

f = figure('Position', [400, 400, 900, 500], ...
    'Name', 'Control Method and Kinematic Representation', ...
    'MenuBar', 'none', ...
    'NumberTitle', 'off', ...
    'WindowStyle', 'modal');

% ------------------------------------------------------------------------------
% Control methods
% ------------------------------------------------------------------------------
bg1 = uibuttongroup('Parent', f, ...
    'Position', [0.02 0.55 1.0 0.4], ...
    'Title', 'Control Methods', ...
    'FontSize', 14, ...
    'FontWeight', 'bold');

radio1 = uicontrol(bg1, ...
    'Style', 'radiobutton', ...
    'FontSize', 13, ...
    'String', 'Gradient-based depth control and PID pole-placement for heading control', ...
    'Position', [10 125 800 30], ...
    'Tag', '1', ...
    'Value', 1);  % default

radio2 = uicontrol(bg1, ...
    'Style', 'radiobutton', ...
    'FontSize', 13, ...
    'String', 'Gradient-based depth control and integral sliding-mode control (SMC) for heading control', ...
    'Position', [10 90 800 30], ...
    'Tag', '2');

radio3 = uicontrol(bg1, ...
    'Style', 'radiobutton', ...
    'FontSize', 13, ...
    'String', 'Gradient-based depth control, SMC heading control, and ALOS guidance law for 2.5-D path following', ...
    'Position', [10 55 850 30], ...
    'Tag', '3');

radio4 = uicontrol(bg1, ...
    'Style', 'radiobutton', ...
    'FontSize', 13, ...
    'String', 'Successive-loop depth control, SMC heading control, and ALOS guidance law for 3-D path following', ...
    'Position', [10 20 800 30], ...
    'Tag', '4');

% ------------------------------------------------------------------------------
% Kinematics
% ------------------------------------------------------------------------------
bg2 = uibuttongroup('Parent', f, ...
    'Position', [0.02 0.3 0.96 0.25], ...
    'Title', 'Kinematics Representation', ...
    'FontSize', 14, ...
    'FontWeight', 'bold');

radio5 = uicontrol(bg2, ...
    'Style', 'radiobutton', ...
    'FontSize', 13, ...
    'String', 'Euler angles', ...
    'Position', [10 50 500 30], ...
    'Tag', '1', ...
    'Value', 0);

radio6 = uicontrol(bg2, ...
    'Style', 'radiobutton', ...
    'FontSize', 13, ...
    'String', 'Unit quaternions', ...
    'Position', [10 15 500 30], ...
    'Tag', '2', ...
    'Value', 1);  % default: quaternions

% ------------------------------------------------------------------------------
% OK button
% ------------------------------------------------------------------------------
uicontrol('Style', 'pushbutton', ...
    'String', 'OK', ...
    'FontSize', 13, ...
    'Position', [20 50 100 40], ...
    'Callback', @(src, evt) uiresume(f));

uiwait(f); % Wait until OK is pressed

% ------------------------------------------------------------------------------
% Read control method
% ------------------------------------------------------------------------------
if get(radio1, 'Value') == 1
    ControlFlag = str2double(get(radio1, 'Tag'));
elseif get(radio2, 'Value') == 1
    ControlFlag = str2double(get(radio2, 'Tag'));
elseif get(radio3, 'Value') == 1
    ControlFlag = str2double(get(radio3, 'Tag'));
else
    ControlFlag = str2double(get(radio4, 'Tag'));
end

% ------------------------------------------------------------------------------
% Read kinematics option 
% ------------------------------------------------------------------------------
if get(radio5, 'Value') == 1
    % Euler angles
    KinematicsFlag = str2double(get(radio5, 'Tag'));
else
    % Unit quaternions
    KinematicsFlag = str2double(get(radio6, 'Tag'));
end

close(f);  % Close the figure

% ------------------------------------------------------------------------------
% Summary text
% ------------------------------------------------------------------------------
disp('-----------------------------------------------------------------------');
disp('MSS toolbox: Remus 100 AUV');

if KinematicsFlag == 1
    disp('Kinematics:          Euler angle representation (12 states)');
else
    disp('Kinematics:          Unit quaternion representation (13 states)');
end

switch ControlFlag
    case 1
        % Method 1
        disp('Heading autopilot:   PID pole-placement control');
        disp('Depth autopilot:     Gradient-based depth control');

    case 2
        % Method 2
        disp('Heading autopilot:   Integral sliding-mode control (SMC)');
        disp('Depth autopilot:     Gradient-based depth control');

    case 3
        % Method 3: 2-D ALOS in the horizontal plane
        disp('Heading autopilot:   Integral sliding-mode control (SMC)');
        disp('Depth autopilot:     Gradient-based depth control');
        disp('Path-following:      ALOS guidance law for 2.5-D path following');

    case 4
        % Method 4: 3-D ALOS with spherical representation
        disp('Heading autopilot:   Integral sliding-mode control (SMC)');
        disp('Depth autopilot:     Successive-loop closure (PI-controller)');
        disp('Path-following:      ALOS guidance law for 3-D path following');

    otherwise
        disp('Unknown ControlFlag setting');
end

disp('------------------------------------------------------------------------');
disp('Simulating...');

end
