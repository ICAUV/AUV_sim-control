function P = auv1Params(vehicle)
% P = auv1Params(vehicle) returns the model, control system and guidance
% parameters of the simulator as a struct:
%
%   P.model     - Vehicle model and actuator parameters (passed to auv1.m)
%   P.ctrl      - Depth, pitch and heading autopilot parameters
%   P.guidance  - ALOS path-following parameters
%
% vehicle selects the model parameter set:
%   'auv1'      - AUV1 hull, mass, buoyancy and fins (default). The propeller
%                 is still the REMUS 100 one (pass B replaces it).
%   'remus100'  - Original MSS REMUS 100 values, used as the reference in
%                 tests/test_auv1_matches_remus100.m.
%
% Field names match the variable names used in auv1.m and SIMauv1.m.
% Derived quantities (planform area, spheroid semi-axes, parasitic drag
% coefficient, fin areas and pitch PID gains) are computed here, so changing
% a primary parameter updates everything that depends on it.
%
% Every AUV1 value is documented as: unit | source | uncertainty. Values
% without a measurement are marked ESTIMATE. The history of changes from the
% REMUS 100 values is kept in PARAMS_CHANGELOG.md.
%
% Example usage:
%   P = auv1Params();
%   xdot = auv1(x, ui, Vc, betaVc, w_c, P.model);
%   x = rk4(@auv1, h, x, ui, Vc, betaVc, w_c, P.model);

if nargin < 1 || isempty(vehicle), vehicle = 'auv1'; end

constant = mssConstants();
rho_mss = constant.rho_water;      % Density used inside the MSS helpers (kg/m3)

% ==============================================================================
% MODEL PARAMETERS (auv1.m)
% ==============================================================================
switch lower(vehicle)

    case 'auv1'
        % --- Environment --------------------------------------------------
        % Gravity latitude (rad) | London, 51.5 deg N | negligible
        model.g_mu = gravity(deg2rad(51.5));

        % Water density (kg/m3) | fresh water, pool tests | 996-1000 for
        % 10-30 deg C. KNOWN GAP: the MSS helpers (spheroid, imlay61,
        % crossFlowDrag) use rho_water = 1025 internally, so the rigid-body
        % mass, added mass and hull drag are ~2.5% off relative to fresh
        % water. Only the propeller and fin forces in auv1.m use this value.
        model.rho = 1000;

        % --- Hull ---------------------------------------------------------
        % Length (m) | orientation deck 13 Oct 2026, slide 4; current CAD
        % spine rod 1030 mm | +-0.05
        model.L_auv = 1.1;

        % Diameter (m) | orientation deck 13 Oct 2026, slide 4 | +-0.01
        model.D_auv = 0.16;

        % Mass moving with the hull (kg) | ESTIMATE, +-20%. Includes the water
        % inside the free-flooded sections (~ rho*pi*(D/2)^2*L). The deck
        % gives ~20 kg dry.
        m_target = 22;

        % Spheroid semi-axes (m) | derived from m_target. spheroid() computes
        % m = 4/3*pi*rho_mss*a*b^2, so the hull spheroid is scaled by k to
        % return m_target. a, b are therefore slightly larger than L/2, D/2.
        k = (m_target / (rho_mss * 4/3 * pi * (model.L_auv/2) * (model.D_auv/2)^2))^(1/3);
        model.a = k * model.L_auv/2;
        model.b = k * model.D_auv/2;

        % Centre of gravity w.r.t. the body origin CO (m) | ESTIMATE, +-0.01
        % in z, until the TG2 tilt test
        model.r_bG = [ 0 0 0.02 ]';

        % Centre of buoyancy w.r.t. the body origin CO (m) | ESTIMATE,
        % body origin placed at the CB
        model.r_bB = [ 0 0 0 ]';

        % Net buoyancy B - W (N) | ESTIMATE, slightly positive as required by
        % TG2; to be measured at the float test
        model.dB = 2;

        % --- Fins ---------------------------------------------------------
        % Area per fin (m2) | TG5 brief, real blade | uncertainty not stated
        model.S_fin = 0.0096;

        % Fin x-position from mid-length, rudder and stern plane (m) |
        % ESTIMATE, +-0.05, until the hinge is measured
        model.x_r = -0.45;
        model.x_s = -0.45;

        % Maximum rudder and stern-plane angle (rad) | 25 deg,
        % auv1-sim/config/vehicle.json | unverified
        model.delta_max = deg2rad(25);

    case 'remus100'
        % Original MSS REMUS 100 values; Fossen (2027, Chapter 8) and
        % Allen et al. (2000)
        mu = deg2rad(63.446827);       % Latitude for Trondheim, Norway (rad)
        model.g_mu = gravity(mu);      % Gravity (m/s2)
        model.rho = rho_mss;           % Density of water (kg/m3)

        model.L_auv = 1.6;             % AUV length (m)
        model.D_auv = 0.19;            % AUV diameter (m)
        model.a = 1.0096 * model.L_auv/2;  % Scaled spheroid semi-axes a and b to obtain m = 31.9 kg
        model.b = 1.0096 * model.D_auv/2;
        model.r_bG = [ 0 0 0.02 ]';    % CG w.r.t. to the CO
        model.r_bB = [ 0 0 0 ]';       % CB w.r.t. to the CO
        model.dB = 0;                  % Neutrally buoyant, B = W (N)

        model.S_fin = 0.00665;         % Fin area (m2)
        model.x_r = -model.a;          % Rudder x-position (m)
        model.x_s = -model.a;          % Stern-plane x-position (m)

        model.delta_max = deg2rad(20); % Maximum rudder and stern angles (rad)

    otherwise
        error('auv1Params:vehicle', ...
            'Unknown vehicle ''%s''. Use ''auv1'' or ''remus100''.', vehicle);
end

% Planform area S = 70% of L_auv * D_auv (m2) | derived
model.S = 0.7 * model.L_auv * model.D_auv;

% Parasitic drag coefficient CD_0, i.e. zero lift and alpha = 0 | derived
% F_drag = 0.5 * rho * Cd * (pi * b^2)
% F_drag = 0.5 * rho * CD_0 * S
% Cd (-) | ESTIMATE (REMUS value), from Allen et al. (2000)
model.Cd = 0.42;
model.CD_0 = model.Cd * pi * model.b^2 / model.S;

% Added moment of inertia in roll: A44 = r44 * Ix (-) | ESTIMATE (REMUS value)
model.r44 = 0.3;

% Tail rudder
% Lift coefficient (-) | ESTIMATE (REMUS value), REMUS calibrated effective value
model.CL_delta_r = 0.5;
model.A_r = 2 * model.S_fin;       % Rudder area (m2) | derived from S_fin

% Stern plane (double)
% Lift coefficient (-) | ESTIMATE (REMUS value), REMUS calibrated effective value
model.CL_delta_s = 0.7;
model.A_s = 2 * model.S_fin;       % Stern-plane area (m2) | derived from S_fin

% Low-speed linear damping matrix parameters | ESTIMATE (REMUS value)
model.T1 = 20;                     % Time constant in surge (s)
model.T2 = 20;                     % Time constant in sway (s)
model.zeta4 = 0.3;                 % Relative damping ratio in roll
model.zeta5 = 0.8;                 % Relative damping ratio in pitch
model.T6 = 1;                      % Time constant in yaw (s)

% Operating limits | ESTIMATE (REMUS value)
model.z_max = 100;                 % Maximum depth (m)

% ------------------------------------------------------------------------------
% REMUS PLACEHOLDER: replaced by T500 in pass B (both parameter sets)
% ------------------------------------------------------------------------------
model.n_max = 1525;                % Maximum propeller speed (RPM)
model.n_rate = 0.1;                % Propeller rate limit (RPM per update)

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

% The AUV1 hull spheroid must reproduce the target mass (spheroid() uses rho_mss)
if strcmpi(vehicle, 'auv1')
    MRB = spheroid(model.a, model.b, zeros(3,1), model.r_bG);
    assert(abs(MRB(1,1) - m_target) / m_target < 1e-3, ...
        'auv1Params:mass', 'Spheroid mass %.3f kg differs from m_target %.3f kg', ...
        MRB(1,1), m_target);
end

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
