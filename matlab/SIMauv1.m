function SIMauv1(ControlFlag, KinematicsFlag)
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
%
% Dependencies:
%   auv1.m                - Dynamics of the AUV1 vehicle
%   auv1Params.m          - Model, control system and guidance parameters
%   integralSMCheading.m  - Integral sliding mode control for heading
%   refModel.m            - Reference model for autopilot systems
%   ALOSpsi.m             - 2-D ALOS guidance algorithm for path following
%   ALOS3D.m              - 3-D ALOS guidance algorithm for path following
%   LOSobserver.m         - Observer for LOS guidance 
%   q2euler.m, ssa.m      - Utilities for quaternion to Euler conversion 
%                           and angle wrapping
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

% ==============================================================================
% SIMULATOR CONFIGURATION
% ==============================================================================
h = 0.05;                           % Sampling time (s)
T_final = 1400;	                    % Final simulation time (s)

if nargin < 2
    [ControlFlag, KinematicsFlag] = controlMethod();
end

P = auv1Params();                   % Model, control and guidance parameters

% Define waypoints for 3D path following
wpt.pos.x = [0  -20 -100   0  200, 200  400];
wpt.pos.y = [0  200  600 950 1300 1800 2200];
wpt.pos.z = [0   10  100 100   50   50   50];

% Initialize position and orientation
xn = 0; yn = 0; zn = 0;             % Initial North-East-Down positions (m)
phi = 0; theta = 0;                 % Initial Euler angles (radians)
psi = atan2(wpt.pos.y(2) - wpt.pos.y(1), ...
    wpt.pos.x(2) - wpt.pos.x(1));   % Yaw angle towards next waypoint
U = 1;                              % Initial speed (m/s)

% Initial control and state setup
theta_d = 0; q_d = 0;               % Initial pitch references
psi_d = psi; r_d = 0; a_d = 0;      % Initial yaw references

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

% Initialize propeller dynamics
n_max = P.model.n_max;         % Maximum propeller speed (RPM)
n_rate = P.model.n_rate;       % Rate limit (RPM per update)
n_p = 1000;                   % Initial propeller speed (RPM)
n_d = 1300;                    % Desired propeller speed (RPM)

% AUV1 actuators (P.model.actuators = 'auv1'): the controller outputs are
% mixed to four fin PWM signals with a servo slew-rate limit, the stern
% thruster runs at a constant PWM with a first-order lag, and the bow
% thruster is held at neutral (blending is not part of this scenario).
isAUV1 = strcmp(P.model.actuators, 'auv1');
if isAUV1
    pwm_centre = P.model.pwm_centre;       % Neutral PWM (us)
    pwm_range = P.model.pwm_range;         % PWM range at full effort (us)
    fin_angle = zeros(4,1);                % Fin angles [top bottom left right] (rad)
    stern_effort = 0.5;                    % Stern thruster effort (-1..1)
    pwm_stern_cmd = pwm_centre + stern_effort * pwm_range; % 1700 us
    pwm_stern = pwm_centre;                % Stern PWM after the thruster lag (us)
    pwm_bow = pwm_centre;                  % Bow PWM, held at neutral (us)
    a_thruster = exp(-h / P.model.thruster_tau); % Discrete first-order lag
end

% Time vector initialization
t = 0:h:T_final;                % Time vector from 0 to T_final          
nTimeSteps = length(t);         % Number of time steps

% ==============================================================================
% CONTROL SYSTEM CONFIGURATION
% ==============================================================================
% Parameters are defined in auv1Params.m
delta_max = P.model.delta_max; % Maximum rudder and stern angle (rad)

% Setup for depth and heading control
z_max = P.model.z_max;         % Maximum depth (m)
psi_step = deg2rad(-60);       % Step change in heading angle (rad)
z_step = 30;                   % Step change in depth, max 100 m

% Integral states for autopilots
z_int = 0;                     % Integral state for depth control
theta_int = 0;                 % Integral state for pitch control
psi_int = 0;                   % Integral state for yaw control

% Depth controller (successive-loop closure)
z_d = zn;                      % Initial desired depth (m)
zdot_d = 0;                    % Initial desired vertical velocity (m/s)
Kp_z = P.ctrl.Kp_z;  T_z = P.ctrl.T_z;  k_grad = P.ctrl.k_grad;
Kp_theta = P.ctrl.Kp_theta;  Kd_theta = P.ctrl.Kd_theta;  Ki_theta = P.ctrl.Ki_theta;
wn_d_z = P.ctrl.wn_d_z;  zeta_d_z = P.ctrl.zeta_d_z;
w_max = P.ctrl.w_max;  theta_max = P.ctrl.theta_max;
LP.z_d = makeLowPass(wn_d_z, h); % Low-pass filter object

% Heading control (Nomoto model) and heading autopilot
K_yaw = P.ctrl.K_yaw;  T_yaw = P.ctrl.T_yaw;
zeta_d_psi = P.ctrl.zeta_d_psi;  wn_d_psi = P.ctrl.wn_d_psi;  r_max = P.ctrl.r_max;
lambda = P.ctrl.lambda;  lambda_alos = P.ctrl.lambda_alos;  phi_b = P.ctrl.phi_b;

if ControlFlag == 1 % PID control parameters
    K_d = P.ctrl.pid.K_d;  K_sigma = P.ctrl.pid.K_sigma;
else                % SMC control parameters
    K_d = P.ctrl.smc.K_d;  K_sigma = P.ctrl.smc.K_sigma;
end

% ==============================================================================
% ALOS PATH-FOLLOWING PARAMETERS
% ==============================================================================
Delta_h = P.guidance.Delta_h;  Delta_v = P.guidance.Delta_v;
gamma_h = P.guidance.gamma_h;  gamma_v = P.guidance.gamma_v;
M_theta = P.guidance.M_theta;
R_switch = P.guidance.R_switch;  K_f = P.guidance.K_f;

% ==============================================================================
% INPUT RANGE CHECK
% ==============================================================================
rangeCheck(n_d, 0, n_max); 
rangeCheck(z_step, 0, z_max);
rangeCheck(U, 0, 5);

% ==============================================================================
% MAIN LOOP
% ==============================================================================
simData = zeros(nTimeSteps, length(x) + 10); % Preallocate table for simulation data
alosData = zeros(nTimeSteps, 4); % Preallocate table for ALOS guidance data

for i = 1:nTimeSteps

    % Measurements
    u = x(1);                  % Surge velocity (m/s)
    v = x(2);                  % Sway veloiciy (m/s)
    w = x(3);                  % Heave velocity (m/s)
    q = x(5);                  % Pitch rate (rad/s)
    r = x(6);                  % Yaw rate (rad/s)
    xn = x(7);                 % North position (m)
    yn = x(8);                 % East position (m)
    zn = x(9);                 % Down position (m), depth

    % Kinematic representation
    switch KinematicsFlag
        case 1
            theta = x(11); psi = x(12); % Euler angles
        otherwise
            [~,theta,psi] = q2euler(x(10:13)); % Quaternion to Euler angles
    end

    % Control systems 
    switch ControlFlag

        case {1, 2}  % Gradient-based depth control and heading autopilot
            % Depth command, z_ref, and heading angle command, psi_ref
            if t(i) > 200
                z_ref = z_step;
                psi_ref = psi_step;
            else
                z_ref = 10;
                psi_ref = deg2rad(0);
            end

           % Depth autopilot using the stern planes 
            delta_s = -Kp_theta * ssa(theta - theta_d)...   % PID
               - Kd_theta * q - Ki_theta * theta_int;
            delta_s = sat(delta_s, delta_max);

            % Depth kinematics: 
            %   z_dot = -u * sin(theta) + w * cos(theta) := alpha_z + sigma
            % Virtual control:
            %   alpha_z = z_d_dot + Kp_z * (e_z + (1/T_z) * z_int)
            %
            % Outer-loop depth control residual sigma = 0 implies e_z = 0
            z_d = LP.z_d(z_ref); % LP filtering the depth command

            sigma = -u * sin(theta) + w * cos(theta) ...
                + Kp_z * ((zn - z_d) + (1/T_z) * z_int);

            % Gradient-descent update using J = 1/2 * sigma^2 to drive sigma to 0:            
            % Residual sensitivity: d(sigma)/d(theta)
            % Gradient: dJ/d(theta) = d(sigma)/d(theta) * sigma
            if abs(w/u) > 0.176 % abs(w/u) > 10 deg
                gradJ = -(u * cos(theta) + w * sin(theta));
            else % Simplified gradient when abs(w/u) < 10 deg
                gradJ = -sign(u);
            end

            % Propagate gradient-based depth command (k+1)
            theta_d = theta_d - h * k_grad * gradJ * sigma;
            theta_d = sat(theta_d, theta_max);

            % Heading autopilot using the tail rudder
            delta_r = integralSMCheading(psi, r, psi_d, r_d, a_d, ...
                K_d, K_sigma, lambda, phi_b, K_yaw, T_yaw, h);
            delta_r = sat(delta_r, delta_max);

            % Propagate reference model for heading angle (k+1)
            [psi_d, r_d, a_d] = refModel(psi_d, r_d, a_d, psi_ref, r_max,...
                zeta_d_psi, wn_d_psi, h, 1);

        case 3 % 2-D ALOS guidance law
           % Depth autopilot using the stern planes 
            delta_s = -Kp_theta * ssa(theta - theta_d)...   % PID
               - Kd_theta * q - Ki_theta * theta_int;
            delta_s = sat(delta_s, delta_max);

            % Heading autopilot using the tail rudder (integral SMC)
            delta_r = integralSMCheading(psi, r, psi_d, r_d, a_d, ...
                K_d, K_sigma, lambda_alos, phi_b, K_yaw, T_yaw, h);
            delta_r = sat(delta_r, delta_max);

            % 2-D ALOS guidance law
            [psi_ref,~,~,k_active] = ALOSpsi(xn,yn,Delta_h,gamma_h,h,R_switch,wpt);
            z_ref = wpt.pos.z(k_active); % Use z-coordinate as depth reference

            % Gradient-based depth command (down position)
            sigma = -u * sin(theta) + w * cos(theta) ...
                + Kp_z * ((zn - z_d) + (1/T_z) * z_int);

            gradJ = -(u * cos(theta) + w * sin(theta));

            % Propagate gradient-based depth command (k+1)
            theta_d = theta_d - h * k_grad * gradJ * sigma;
            theta_d = sat(theta_d, theta_max);
            
            % Propagate ALOS observer (k+1)
            [psi_d, r_d] = LOSobserver(psi_d, r_d, psi_ref, h, K_f);
            r_d = sat(r_d, r_max);

            % Propagate reference model for desired depth (k+1)
            [z_d,zdot_d,a_d] = refModel(z_d,zdot_d,a_d,z_ref,w_max,zeta_d_z,wn_d_z,h,0);

        otherwise % Control flag 4 (3-D ALOS path-following)
            % Heading autopilot using the tail rudder (integral SMC)
            delta_r = integralSMCheading(psi, r, psi_d, r_d, a_d, ...
                K_d, K_sigma, lambda_alos, phi_b, K_yaw, T_yaw, h);
            delta_r = sat(delta_r, delta_max);

            % Depth autopilot using the stern planes (PID)
            delta_s = -Kp_theta * ssa(theta - theta_d)...
                - Kd_theta * q - Ki_theta * theta_int;
            delta_s = sat(delta_s, delta_max);

            % 3-D ALOS guidance law
            [psi_ref, theta_ref, y_e, z_e, alpha_c_hat, beta_c_hat] = ...
                ALOS3D(xn, yn, zn, Delta_h, Delta_v, gamma_h, gamma_v,...
                M_theta, h, R_switch, wpt);

            % Propagate ALOS observers
            [theta_d, q_d] = LOSobserver(theta_d, q_d, theta_ref, h, K_f);
            [psi_d, r_d] = LOSobserver(psi_d, r_d, psi_ref, h, K_f);
            r_d = sat(r_d, r_max);

            % Ocean current dynamics
            if t(i) > 800
                Vc_d = 0.65;
                w_V = 0.05;
                Vc = exp(-h*w_V) * Vc + (1 - exp(-h*w_V)) * Vc_d;
            else
                Vc = 0.5;
            end

            if t(i) > 500
                betaVc_d = deg2rad(160);
                w_beta = 0.1;
                betaVc = exp(-h*w_beta) * betaVc + (1 - exp(-h*w_beta)) * betaVc_d;
            else
                betaVc = deg2rad(150);
            end

            betaVc = betaVc + randn / 1000;
            Vc = Vc + 0.002 * randn;

            % Store ALOS data in table
            alosData(i,:) = [y_e z_e alpha_c_hat beta_c_hat];
    end

   if isAUV1
       % Normalised yaw and pitch commands with the same moment sign as the
       % REMUS rudder and stern planes (+yaw_cmd: N > 0, +pitch_cmd: M > 0),
       % mixed to the four fins as on the vehicle (fin_mixer.m)
       yaw_cmd = delta_r / delta_max;
       pitch_cmd = delta_s / delta_max;
       roll_cmd = 0;                       % No roll loop yet
       [~, deflection] = fin_mixer(pitch_cmd, roll_cmd, yaw_cmd);

       % Servo slew-rate limit on the fin angles
       d_max = P.model.servo_slew * h;
       fin_angle = fin_angle + max(-d_max, min(d_max, deflection * delta_max - fin_angle));
       pwm_fins = deflection_to_pwm(fin_angle / delta_max);

       % Stern thruster PWM with first-order lag, bow thruster at neutral
       pwm_stern = a_thruster * pwm_stern + (1 - a_thruster) * pwm_stern_cmd;

       % Control input vector [top bottom left right stern bow] (us)
       ui = [pwm_fins; pwm_stern; pwm_bow];
       ui_log = [delta_r -delta_s pwm_stern]';
   else
       % Propeller speed (RPM)
       if n_p < n_d
           n_p = n_p + n_rate;
       elseif n_p > n_d
           n_p = n_p - n_rate;
       end
       n_p = sat(n_p, n_max);

       % Control input vector
       ui = [delta_r -delta_s n_p]';
       ui_log = ui;
   end

   % Store simulation data in a table
   simData(i,:) = [z_d theta_d psi_d r_d Vc betaVc wc ui_log' x'];

   if (KinematicsFlag == 1)
       % Euler angles x = [ u v w p q r x y z phi theta psi ]'
       x = rk4(@auv1, h, x, ui, Vc, betaVc, wc, P.model);  % RK4 method x(k+1)
   else
       % Unit quaternions x = [ u v w p q r x y z eta eps1 eps2 eps3 ]'
       xdot = auv1(x, ui, Vc, betaVc, wc, P.model);
       quat = x(10:13);                            % Unit quaternion
       x(1:6) = x(1:6) + h * xdot(1:6);            % Forward Euler
       x(7:9) = x(7:9) + h * Rquat(quat) * x(1:3); % Backward Euler
       quat = expm(Tquat(x(4:6)) * h) * quat;  % Exact quat. discretization
       x(10:13) = quat / norm(quat);           % Normalization
   end

   % Euler's integration method (k+1)
   z_int = z_int + h * ( zn - z_d );
   theta_int = theta_int + h * ssa( theta - theta_d );
   psi_int = psi_int + h * ssa( psi - psi_d );

end

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
