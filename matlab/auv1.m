function [xdot,U,M,tau] = auv1(x,ui,Vc,betaVc,w_c,model)

% The length of the AUV1 is 1.1 m, the cylinder diameter is 16 cm and the
% mass of the vehicle (including the water in the free-flooded sections) is
% 22 kg. All values are documented in auv1Params.m.
%
% The function returns the time derivative xdot of the state vector: 
%
%   x = [ u v w p q r x y z phi theta psi ]', alternatively 
%   x = [ u v w p q r x y z eta eps1 eps2 eps3 ]' 
%
% in addition to the speed U in m/s, the 6x6 mass matrix M and the generalized
% actuator force vector tau (optionally). The state vector can be
% of dimension 12 (Euler angles) or 13 (unit quaternions):
%
%   u:       Surge velocity          (m/s)
%   v:       Sway velocity           (m/s)
%   w:       Heave velocity          (m/s)
%   p:       Roll rate               (rad/s)
%   q:       Pitch rate              (rad/s)
%   r:       Yaw rate                (rad/s)
%   x:       North position          (m)
%   y:       East position           (m)
%   z:       Downwards position      (m)
%   phi:     Roll angle              (rad)       
%   theta:   Pitch angle             (rad)
%   psi:     Yaw angle               (rad)
% 
% For the unit quaternion representation, the last three arguments of the 
% x-vector, the Euler angles (phi, theta, psi), are replaced by the unit 
% quaternion quat = [eta, eps1, eps2, eps3]'. This increases the dimension of
% the state vector from 12 to 13.
%
% The control inputs depend on the actuator set, model.actuators:
%
% 'remus100': one tail rudder, two stern planes and a single-screw propeller
%
%   ui = [ delta_r delta_s n ]'  where
%
%    delta_r:   Rudder angle (rad)
%    delta_s:   Stern plane angle (rad)
%    n_p:       Propeller revolution (RPM)
%
% 'auv1': four independent fins in a '+' arrangement, a T500 stern thruster
% and a T200 bow tunnel thruster, driven by the six PWM signals sent to the
% autopilot (us):
%
%   ui = [ pwm_top pwm_bottom pwm_left pwm_right pwm_stern pwm_bow ]'
%
%    Fin PWM:    1500 + 400 * deflection, deflection in -1..1; +-400 us maps
%                linearly to +-delta_max (fin_mixer.m, deflection_to_pwm.m)
%    Stern PWM:  T500 thrust along +x on the centreline, positive for PWM > 1500
%    Bow PWM:    T200 thrust along +y at x_bow, positive for PWM > 1500
%
%    Each fin produces lift L_i = 0.5*rho*U^2*S_fin*CL_delta*delta_i (U_rh and
%    CL_delta_r for top/bottom, U_rv and CL_delta_s for left/right) along body y
%    (top, bottom) or body z (left, right), multiplied by the fin force sign,
%    and drag X_i = -0.5*rho*U^2*S_fin*CL_delta*delta_i^2. Moments are r x F
%    with the fin centre of pressure at (x_fin, 0, -+r_fin) for top/bottom and
%    (x_fin, -+r_fin, 0) for left/right.
%
%    KNOWN GAPS of the AUV1 actuator model: the thrust is bollard (static)
%    data, with no loss with forward speed, so it overpredicts at speed; no
%    thrust deduction; no propeller torque in roll; no fin stall; no local flow
%    angle at the fins (fins see the free-stream relative velocity).
%
% The arguments Vc (m/s), betaVc (rad), w_c (m/s) are optional arguments for 
% ocean currents
%
%    v_c = [ Vc * cos(betaVc - psi), Vc * sin( betaVc - psi), w_c ]
%
% The optional argument model is the struct P.model from auv1Params.m. If it
% is omitted, the default parameters are loaded on every call, so pass it
% explicitly in simulation loops. The default model is the AUV1 set.
%
% Example usage:
%   [~,~,M] = auv1()                             : Return the 6x6 mass matrix M
%   [xdot,U,M,tau] = auv1(x,ui,Vc,betaVc,w_c,P.model) : Also return actuator forces
%   P = auv1Params();
%   [xdot,U] = auv1(x,ui,Vc,betaVc,w_c,P.model)  : 3-D ocean currents
%   [xdot,U] = auv1(x,ui,Vc,betaVc,w_c)          : 3-D ocean currents, default model
%   [xdot,U] = auv1(x,ui,Vc,betaVc)              : 2-D ocean currents, default model
%   [xdot,U] = auv1(x,ui)                        : No ocean currents, default model
%
% Author:    Jaime López Ruiz, Léo Sutherby, Thor I. Fossen
% 
% Date:      2026-10-16
% Revisions:
%   2026-10-16  
%   2021-08-24  Ocean currents are now expressed in NED
%   2021-10-21  imlay61.m is called using the relative velocity
%   2021-12-30  Added the time derivative of the current velocity
%   2022-02-01  Updated lift and drag forces
%   2022-05-06  Calibration of drag and propulsion forces using data from 
%               Allen et al. (2000)
%   2022-06-08  Added compatibility for unit quaternions in addition to the 
%               Euler angle representation
%   2022-10-16  Added vertical currents
%   2023-05-02  Corrected the rudder area A_r
%   2023-10-07  Scaled down the propeller roll-induced moment
%   2024-02-09  Updated rudder and stern-plane areas
%   2024-02-13  Calibration of the model parameters
%   2024-06-23  Corrected sign of tau(5). A positive delta_s will result in a
%               negative pitch and negative Z_s. Hence, tau(5) = -x_s * Z_s 
%               when x_s < 0.
%   2025-04-25 Added empty call: [~,~,M] = remus100(), and minor bug fixes.
%   2025-06-09 Change cross-flow drag to cylinder model (M. Seidl).
%   2026-08-26 Use through-water speed for propulsion and damping (J. Harvey).
%
% References: 
%   B. Allen, W. S. Vorus and T. Prestero, "Propulsion system 
%       performance enhancements on REMUS AUVs," OCEANS 2000 MTS/IEEE 
%       Conference and Exhibition. Conference Proceedings, 2000, 
%       pp. 1869-1873 vol.3, doi: 10.1109/OCEANS.2000.882209.
%   T. I. Fossen (2027). Handbook of Marine Craft Hydrodynamics and
%       Motion Control. 3rd. Edition, Wiley. URL: www.fossen.biz/wiley   

if nargin == 0  % [~,~,M] = auv1()
    x = zeros(12,1); ui = []; Vc = 0; betaVc = 0; w_c = 0;
end

if (nargin == 2), Vc = 0; betaVc = 0; w_c = 0; end % No ocean currents
if (nargin == 4), w_c = 0; end % No vertical ocean currents
if (nargin < 6 || isempty(model)) % Default model parameters
    P = auv1Params();
    model = P.model;
end

if isempty(ui), ui = model.ui0; end % Neutral inputs of the actuator set
isAUV1 = strcmp(model.actuators, 'auv1');
if isAUV1
    if (length(ui) ~= 6), error('u-vector must have dimension 6 for the auv1 actuators!'); end
else
    if (length(ui) ~= 3), error('u-vector must have dimension 3!'); end
end
if (length(x) ~= 12 && length(x) ~= 13)
    error('x-vector must have dimension 12 or 13'); 
end

% Constants (auv1Params.m)
g_mu = model.g_mu;       % Gravity (m/s2)
rho = model.rho;         % Density of water (kg/m3)

% 6x1 velocity vector, yaw angle and control inputs
nu = x(1:6); 
if length(x) == 12 % Euler angles
    psi = x(12);
else % Convert unit quaternion to yaw angle
    psi = atan2(2*(x(10)*x(13) + x(11)*x(12)), 1 - 2*(x(12)^2 + x(13)^2));
end

% Amplitude saturation of the control signals (REMUS actuators)
if ~isAUV1
    delta_r = sat(ui(1), model.delta_max);  % Saturated tail rudder (rad)
    delta_s = sat(ui(2), model.delta_max);  % Saturated Stern plane (rad)
    n_p = sat(ui(3), model.n_max) / 60;     % Saturated propeller speed (rps)
end

% Ocean currents expressed in BODY
u_c = Vc * cos( betaVc - psi );                               
v_c = Vc * sin( betaVc - psi );   

nu_c = [u_c v_c w_c 0 0 0]'; % Ocean current velocities
Dnu_c = [nu(6)*v_c -nu(6)*u_c 0 0 0 0]'; % Time derivative of nu_c

% Relative velocities/speed, angle of attack and vehicle speed
nu_r = nu - nu_c;                                 % Relative velocity
alpha = atan2( nu_r(3), nu_r(1) );                % Angle of attack (rad)
U_r = sqrt( nu_r(1)^2 + nu_r(2)^2 + nu_r(3)^2 );  % Relative speed (m/s)
U  = sqrt( nu(1)^2 + nu(2)^2 + nu(3)^2 );         % Speed (m/s)

% AUV model parameters (auv1Params.m); Fossen (2027, Chapter 8) and
% Allen et al. (2000)
L_auv = model.L_auv;  D_auv = model.D_auv;  S = model.S;
a = model.a;  b = model.b;  r44 = model.r44;
r_bG = model.r_bG;  r_bB = model.r_bB;
CD_0 = model.CD_0;                       % Parasitic drag coefficient

% Propeller coeffs. KT and KQ are computed as a function of advance no.
% Ja = Va/(n*D_prop) where Va = (1-w)*U_r; Allen et al. (2000)
if ~isAUV1
D_prop = model.D_prop;  t_prop = model.t_prop;  Ja_max = model.Ja_max;
KT_0 = model.KT_0;  KQ_0 = model.KQ_0;
KT_max = model.KT_max;  KQ_max = model.KQ_max;
Va = model.Va_ratio * U_r;               % Advance speed (m/s)

% Propeller thrust and propeller-induced roll moment
% Linear approximations for positive Ja values
% KT ~= KT_0 + (KT_max-KT_0)/Ja_max * Ja   
% KQ ~= KQ_0 + (KQ_max-KQ_0)/Ja_max * Ja        
if n_p > 0   
    % Forward thrust
    X_prop = rho * D_prop^4 * ( ... 
        KT_0 * abs(n_p) * n_p + (KT_max-KT_0)/Ja_max * (Va/D_prop) * abs(n_p) );        
    K_prop = rho * D_prop^5 * ( ...
        KQ_0 * abs(n_p) * n_p + (KQ_max-KQ_0)/Ja_max * (Va/D_prop) * abs(n_p) );                 
else      
    % Reverse thrust (braking)   
    X_prop = rho * D_prop^4 * KT_0 * abs(n_p) * n_p; 
    K_prop = rho * D_prop^5 * KQ_0 * abs(n_p) * n_p;            
end            

% Tail rudder and stern plane (double)
CL_delta_r = model.CL_delta_r;  A_r = model.A_r;  x_r = model.x_r;
CL_delta_s = model.CL_delta_s;  A_s = model.A_s;  x_s = model.x_s;
end

% Low-speed linear damping matrix parameters
T1 = model.T1;  T2 = model.T2;  T6 = model.T6;
zeta4 = model.zeta4;  zeta5 = model.zeta5;

% Rigid-body mass and hydrodynamic added mass
[MRB,CRB] = spheroid(a,b,nu(4:6),r_bG);
[MA,CA] = imlay61(a, b, nu_r, r44);

% The reduced-order REMUS model does not retain a complete set of measured
% maneuvering derivatives for the hull, fins, and appendages. The selected
% added-mass Coriolis couplings are therefore omitted to avoid retaining an
% unbalanced Munk-moment model. This is a modeling assumption; the
% corresponding physical terms are not generally zero for a bare body.
CA(5,3) = 0; CA(3,5) = 0;  % Heave-pitch coupling
CA(5,1) = 0; CA(1,5) = 0;  % Surge-pitch Munk coupling
CA(6,1) = 0; CA(1,6) = 0;  % Yaw-related Munk couplings
CA(6,2) = 0; CA(2,6) = 0;

M = MRB + MA;
C = CRB + CA;
m = MRB(1,1); W = m * g_mu; B = W + model.dB; % Net buoyancy dB (N)

% Dissipative forces and moments
D = Dmtrx([T1 T2 T6],[zeta4 zeta5],MRB,MA,[W r_bG' r_bB']);
D(1,1) = D(1,1) * exp(-3 * U_r); % Vanish at high through-water speed

tau_liftdrag = forceLiftDrag(D_auv,S,CD_0,alpha,U_r);
tau_crossflow = crossFlowDrag(L_auv,D_auv,D_auv,nu_r,'cylinder');

% Kinematics
if (length(x) == 13)
    [J,R] = quatern(x(10:13));
else
    [J,R] = eulerang(x(10),x(11),x(12));
end

% Restoring forces and moments
g = gRvect(W,B,R,r_bG,r_bB);

% Horizontal- and vertical-plane relative speed
U_rh = sqrt( nu_r(1)^2 + nu_r(2)^2 );  
U_rv = sqrt( nu_r(1)^2 + nu_r(3)^2 );  

% Generalized propulsion force vector
if isAUV1
    tau = auv1Actuators(ui, rho, U_rh, U_rv, model);
else
    % Rudder and stern-plane drag
    X_r = -0.5 * rho * U_rh^2 * A_r * CL_delta_r * delta_r^2;
    X_s = -0.5 * rho * U_rv^2 * A_s * CL_delta_s * delta_s^2;

    % Rudder sway force
    Y_r = -0.5 * rho * U_rh^2 * A_r * CL_delta_r * delta_r;

    % Stern-plane heave force
    Z_s = -0.5 * rho * U_rv^2 * A_s * CL_delta_s * delta_s;

    tau = zeros(6,1);
    tau(1) = (1-t_prop) * X_prop + X_r + X_s;
    tau(2) = Y_r;
    tau(3) = Z_s;
    tau(4) = K_prop / 10; % Scaled down by a factor of 10 to match exp. results
    tau(5) = -x_s * Z_s;
    tau(6) = x_r * Y_r;
end

% State-space model
xdot = [ Dnu_c + M \ ...
            (tau + tau_liftdrag + tau_crossflow - C * nu_r - D * nu_r  - g)
         J * nu ];

function tau = auv1Actuators(ui, rho, U_rh, U_rv, model)
% Generalized force vector of the AUV1 actuators: four independent fins, the
% T500 stern thruster and the T200 bow thruster. ui is the six PWM signals (us).

% Fin deflection angles (rad) from PWM: +-pwm_range maps to +-delta_max
defl = max(-1, min(1, (ui(1:4) - model.pwm_centre) / model.pwm_range));
delta = defl * model.delta_max;                 % [top bottom left right]'

% Lift (top, bottom: horizontal-plane flow; left, right: vertical-plane flow)
Uf2 = [U_rh^2; U_rh^2; U_rv^2; U_rv^2];
CL = [model.CL_delta_r; model.CL_delta_r; model.CL_delta_s; model.CL_delta_s];
lift = 0.5 * rho * Uf2 .* model.S_fin .* CL .* delta;
drag = -0.5 * rho * Uf2 .* model.S_fin .* CL .* delta.^2;

% Fin forces and positions in the body frame. Top and bottom push along y,
% left and right along z.
x_f = model.x_fin;  r_f = model.r_fin;
s = model.fin_sign(:);
F = [ drag(1) 0 0;                              % Top
      drag(2) 0 0;                              % Bottom
      drag(3) 0 0;                              % Left
      drag(4) 0 0 ];                            % Right
F(1,2) = s(1) * lift(1);  F(2,2) = s(2) * lift(2);
F(3,3) = s(3) * lift(3);  F(4,3) = s(4) * lift(4);
pos = [ x_f  0   -r_f;                          % Top
        x_f  0    r_f;                          % Bottom
        x_f -r_f  0;                            % Left
        x_f  r_f  0 ];                          % Right

% Stern (T500, +x) and bow (T200, +y) thrusters, interpolated from the 14.8 V
% bollard tables and clamped to the table range
T_stern = model.stern_dir * interpTable(model.T500, ui(5));
T_bow = model.bow_dir * interpTable(model.T200, ui(6));
F = [F; T_stern 0 0; 0 T_bow 0];
pos = [pos; 0 0 0; model.x_bow 0 0];

tau = zeros(6,1);
tau(1:3) = sum(F, 1)';
tau(4:6) = sum(cross(pos, F, 2), 1)';

function f = interpTable(tbl, pwm)
% Linear interpolation of a thrust table (pwm_us, force_N) on a uniform PWM grid,
% clamped to its range (same result as interp1, but fast enough for RK4 loops)
p = max(tbl.pwm(1), min(tbl.pwm(end), pwm));
s = (p - tbl.pwm(1)) / tbl.dpwm;
k = min(floor(s) + 1, numel(tbl.pwm) - 1);
f = tbl.force(k) + (s - (k - 1)) * (tbl.force(k+1) - tbl.force(k));
