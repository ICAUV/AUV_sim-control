function [delta_r, delta_s] = trackingController(y, ref, theta_int, ControlFlag, ctrl, delta_max, h)
% [delta_r, delta_s] = trackingController(y, ref, theta_int, ControlFlag, ctrl,
% delta_max, h) is the inner loop: the fin angles that track the desired
% pitch and heading in ref.
%
%   delta_s : Stern planes, pitch PID with integral state theta_int and the
%             feedforward Kff_theta*theta_d against the restoring moment
%   delta_r : Tail rudder, integral SMC heading autopilot. ControlFlag 1 uses
%             the PID gains, 2-4 the SMC gains; 3-4 use the ALOS modes.

% Depth autopilot using the stern planes (PID with restoring-moment feedforward)
delta_s = ctrl.Kff_theta * ref.theta_d - ctrl.Kp_theta * ssa(y.theta - ref.theta_d) ...
    - ctrl.Kd_theta * y.q - ctrl.Ki_theta * theta_int;
delta_s = sat(delta_s, delta_max);

% Heading autopilot using the tail rudder
if ControlFlag == 1, gains = ctrl.pid; else, gains = ctrl.smc; end
if ControlFlag <= 2, lambda = ctrl.lambda; else, lambda = ctrl.lambda_alos; end

delta_r = integralSMCheading(y.psi, y.r, ref.psi_d, ref.r_d, ref.a_d, ...
    gains.K_d, gains.K_sigma, lambda, ctrl.phi_b, ctrl.K_yaw, ctrl.T_yaw, h);
delta_r = sat(delta_r, delta_max);

end
