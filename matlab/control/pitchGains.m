function [Kp, Kd, Ki, Kff] = pitchGains(m_th, d_th, k_th, M_delta, w, zeta)
% [Kp, Kd, Ki, Kff] = pitchGains(m_th, d_th, k_th, M_delta, w, zeta) returns
% the pitch PID gains of trackingController.m (rad of stern plane per rad, per
% rad/s and per rad s) and its restoring-moment feedforward (rad per rad) by
% pole placement on the linearised pitch model
%
%   m_th * theta_dd + d_th * q + k_th * theta = M_delta * delta_s
%
%   m_th    : Pitch inertia including added mass, M(5,5)       (kg m2)
%   d_th    : Linear pitch damping at the design speed         (N m s/rad)
%   k_th    : Restoring stiffness, W*BG_z plus any speed-
%             dependent theta moment                           (N m/rad)
%   M_delta : Pitch moment per rad of stern-plane command
%             delta_s at the design speed                      (N m/rad)
%   w, zeta : Desired natural frequency (rad/s) and relative damping
%
% With delta_s = Kff*theta_d - Kp*(theta - theta_d) - Kd*q
%                - Ki*integral(theta - theta_d)
% the proportional and derivative terms give the closed-loop polynomial
% m_th*s^2 + 2*zeta*w*m_th*s + m_th*w^2, so
%
%   Kp  = (m_th*w^2 - k_th) / M_delta
%   Kd  = (2*zeta*w*m_th - d_th) / M_delta
%   Ki  = Kp * w / 10
%   Kff = k_th / M_delta
%
% Kff is the stern angle whose moment balances the restoring moment at
% theta = theta_d, so the response to a theta_d step reaches the full step
% without waiting for the integral (DC gain 1 instead of 1 - k_th/(m_th*w^2)).
% It leaves the poles unchanged.
%
% The plant already provides k_th and d_th, so the gains are positive only
% when w and zeta ask for more stiffness and damping than the hull has: the
% function errors out otherwise. Theory: successive loop closure, pitch
% inner loop (Ch 15.2-15.3); restoring moment (Ch 4.1).
%
% Example usage:
%   P = auv1Params();
%   [Kp, Kd, Ki, Kff] = pitchGains(P.ctrl.m_theta, P.ctrl.d_theta, P.ctrl.k_theta, ...
%       P.ctrl.M_delta_theta, P.ctrl.w_theta, P.ctrl.zeta_theta);

assert(M_delta > 0, 'pitchGains:effectiveness', ...
    'M_delta = %.4g must be positive (+delta_s gives nose up)', M_delta);
assert(w > 0 && zeta > 0, 'pitchGains:design', ...
    'w and zeta must be positive');

Kp = (m_th * w^2 - k_th) / M_delta;
Kd = (2 * zeta * w * m_th - d_th) / M_delta;
Ki = Kp * w / 10;
Kff = k_th / M_delta;

assert(Kp > 0 && Kd > 0, 'pitchGains:lowBandwidth', ...
    ['w = %.3g rad/s, zeta = %.3g give Kp = %.4g, Kd = %.4g: the hull alone ' ...
    'has w = %.3g rad/s, zeta = %.3g. Choose a higher w or zeta.'], ...
    w, zeta, Kp, Kd, sqrt(k_th / m_th), d_th / (2 * sqrt(k_th * m_th)));

end
