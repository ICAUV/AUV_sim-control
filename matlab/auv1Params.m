function P = auv1Params()
% P = auv1Params() returns the model, control system and guidance parameters
% of the AUV1 simulator as a struct:
%
%   P.model     - Vehicle model and actuator parameters (passed to auv1.m)
%   P.ctrl      - Depth, pitch and heading autopilot parameters
%   P.guidance  - ALOS path-following parameters
%
% Field names match the variable names used in auv1.m and SIMauv1.m.
% Derived quantities (planform area, spheroid semi-axes, parasitic drag
% coefficient, fin areas and pitch PID gains) are computed here, so changing
% a primary parameter updates everything that depends on it.
%
% Example usage:
%   P = auv1Params();
%   xdot = auv1(x, ui, Vc, betaVc, w_c, P.model);
%   x = rk4(@auv1, h, x, ui, Vc, betaVc, w_c, P.model);

% ==============================================================================
% MODEL PARAMETERS (auv1.m)
% ==============================================================================
% Constants
mu = deg2rad(63.446827);           % Latitude for Trondheim, Norway (rad)
model.g_mu = gravity(mu);          % Gravity (m/s2)
constant = mssConstants();
model.rho = constant.rho_water;    % Density of water (kg/m3)

% Actuator and operating limits
model.delta_max = deg2rad(20);     % Maximum rudder and stern angles (rad)
model.n_max = 1525;                % Maximum propeller speed (RPM)
model.n_rate = 0.1;                % Propeller rate limit (RPM per update)
model.z_max = 100;                 % Maximum depth (m)

% Hull; Fossen (2027, Chapter 8) and Allen et al. (2000)
model.L_auv = 1.6;                 % AUV length (m)
model.D_auv = 0.19;                % AUV diameter (m)
model.S = 0.7 * model.L_auv * model.D_auv; % Planform area S = 70% of L_auv * D_auv
model.a = 1.0096 * model.L_auv/2;  % Scaled spheroid semi-axes a and b to obtain m = 31.9 kg
model.b = 1.0096 * model.D_auv/2;
model.r44 = 0.3;                   % Added moment of inertia in roll: A44 = r44 * Ix
model.r_bG = [ 0 0 0.02 ]';        % CG w.r.t. to the CO
model.r_bB = [ 0 0 0 ]';           % CB w.r.t. to the CO

% Parasitic drag coefficient CD_0, i.e. zero lift and alpha = 0
% F_drag = 0.5 * rho * Cd * (pi * b^2)
% F_drag = 0.5 * rho * CD_0 * S
model.Cd = 0.42;                   % From Allen et al. (2000)
model.CD_0 = model.Cd * pi * model.b^2 / model.S;

% Propeller coeffs. KT and KQ are computed as a function of advance no.
% Ja = Va/(n*D_prop) where Va = (1-w)*U_r = 0.944 * U_r; Allen et al. (2000)
model.D_prop = 0.14;               % Propeller diameter corresponding to 5.5 inches
model.t_prop = 0.1;                % Thrust deduction number
model.Va_ratio = 0.944;            % Advance speed ratio (1-w), Va = Va_ratio * U_r

% Ja_max = 0.944 * 2.5 / (0.14 * 1525/60) = 0.6632
model.Ja_max = 0.6632;

% Single-screw propeller with 3 blades and blade-area ratio = 0.718.
% >> [KT_0, KQ_0] = wageningen(0,1,0.718,3)
model.KT_0 = 0.4566;
model.KQ_0 = 0.0700;
% >> [KT_max, KQ_max] = wageningen(0.6632,1,0.718,3)
model.KT_max = 0.1798;
model.KQ_max = 0.0312;

model.S_fin = 0.00665;             % Fin area (m2)

% Tail rudder
model.CL_delta_r = 0.5;            % Rudder lift coefficient (-)
model.A_r = 2 * model.S_fin;       % Rudder area (m2)
model.x_r = -model.a;              % Rudder x-position (m)

% Stern plane (double)
model.CL_delta_s = 0.7;            % Stern-plane lift coefficient (-)
model.A_s = 2 * model.S_fin;       % Stern-plane area (m2)
model.x_s = -model.a;              % Stern-plane x-position (m)

% Low-speed linear damping matrix parameters
model.T1 = 20;                     % Time constant in surge (s)
model.T2 = 20;                     % Time constant in sway (s)
model.zeta4 = 0.3;                 % Relative damping ratio in roll
model.zeta5 = 0.8;                 % Relative damping ratio in pitch
model.T6 = 1;                      % Time constant in yaw (s)

P.model = model;

% ==============================================================================
% CONTROL SYSTEM PARAMETERS
% ==============================================================================
% Depth controller (gradient-based outer loop)
ctrl.Kp_z = 0.1;                   % Proportional gain for depth
ctrl.T_z = 100;                    % Time constant for integral action in depth control
ctrl.k_grad = 0.1;                 % Gain for computation of theta_d
ctrl.wn_d_z = 0.05;                % Natural frequency for depth control
ctrl.zeta_d_z = 1.0;               % Damping ratio, depth reference model (2.5-D ALOS)
ctrl.w_max = 3.0;                  % Maximum dive speed (m/s)
ctrl.theta_max = deg2rad(30.0);    % Maximum pitch angle (rad)

% Pitch controller (PID pole placement using the model mass matrix)
[~,~,M] = auv1(zeros(12,1), zeros(3,1), 0, 0, 0, model);
ctrl.w_theta = 0.8;                % Natural frequency in pitch (rad/s)
ctrl.zeta_theta = 0.8;             % Relative damping ratio in pitch
ctrl.Kp_theta = M(5,5) * ctrl.w_theta^2;                   % Proportional gain
ctrl.Kd_theta = M(5,5) * 2 * ctrl.zeta_theta * ctrl.w_theta; % Derivative gain
ctrl.Ki_theta = ctrl.Kp_theta * ctrl.w_theta / 10;         % Integral gain

% Heading control parameters (using Nomoto model)
ctrl.K_yaw = 5 / 20;               % Gain, max rate of turn over max. rudder angle
ctrl.T_yaw = 1;                    % Time constant for yaw dynamics
ctrl.zeta_d_psi = 1.0;             % Desired damping ratio for yaw control
ctrl.wn_d_psi = 0.1;               % Natural frequency for yaw control
ctrl.r_max = deg2rad(5.0);         % Maximum allowable rate of turn (rad/s)

% Heading autopilot (Equation 16.479 in Fossen 2027)
% sigma = r-r_d + 2*lambda*ssa(psi-psi_d) + lambda^2 * integral(ssa(psi-psi_d))
% delta = (T_yaw*r_r_dot + r_r - K_d*sigma - K_sigma*(sigma/phi_b)) / K_yaw
ctrl.lambda = 0.1;                 % Depth and heading autopilot modes
ctrl.lambda_alos = 1;              % ALOS path-following modes
ctrl.phi_b = 0.1;                  % Boundary layer thickness

ctrl.pid.K_d = 0.5;                % Derivative gain for PID controller
ctrl.pid.K_sigma = 0;              % Gain for SMC (inactive when using PID)
ctrl.smc.K_d = 0;                  % Derivative gain inactive in SMC mode
ctrl.smc.K_sigma = 0.05;           % Sliding mode control gain

P.ctrl = ctrl;

% ==============================================================================
% ALOS PATH-FOLLOWING PARAMETERS
% ==============================================================================
guidance.Delta_h = 20;             % Horizontal look-ahead distance (m)
guidance.Delta_v = 20;             % Vertical look-ahead distance (m)
guidance.gamma_h = 0.001;          % Adaptive gain, horizontal plane
guidance.gamma_v = 0.001;          % Adaptive gain, vertical plane
guidance.M_theta = deg2rad(20);    % Maximum value of estimates, alpha_c, beta_c

% Additional parameters for straight-line path following
guidance.R_switch = 5;             % Radius of switching circle
guidance.K_f = 0.5;                % LOS observer gain

P.guidance = guidance;

end
