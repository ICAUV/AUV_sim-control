function [cmd, alos] = navigation(t, y, ControlFlag, path, guidance, h)
% [cmd, alos] = navigation(t, y, ControlFlag, path, guidance, h) is the
% high-level navigation: the depth, heading and pitch commands at time t.
%
%   ControlFlag 1-2 : Step command schedule path.step (cmd.z_ref, cmd.psi_ref)
%   ControlFlag 3   : 2-D ALOS on path.wpt, depth of the active waypoint
%                     (cmd.psi_ref, cmd.z_ref)
%   ControlFlag 4   : 3-D ALOS on path.wpt (cmd.psi_ref, cmd.theta_ref)
%
% y is the measurement struct from measurements.m and alos = [y_e z_e
% alpha_c_hat beta_c_hat] the ALOS data to log (zeros for ControlFlag 1-2).

alos = zeros(1, 4);

switch ControlFlag

    case {1, 2}  % Step commands, the k-th once t is past k-1 step times
        k = 1 + sum(t > path.step.t);
        cmd.z_ref = path.step.z(k);
        cmd.psi_ref = path.step.psi(k);

    case 3  % 2-D ALOS guidance law
        [cmd.psi_ref, y_e, beta_c_hat, k_active] = ALOSpsi(y.xn, y.yn, ...
            guidance.Delta_h, guidance.gamma_h, h, guidance.R_switch, path.wpt);
        cmd.z_ref = path.wpt.pos.z(k_active); % Use z-coordinate as depth reference
        alos = [y_e NaN NaN beta_c_hat];      % No vertical-track error in 2.5-D

    otherwise  % 3-D ALOS guidance law
        [cmd.psi_ref, cmd.theta_ref, y_e, z_e, alpha_c_hat, beta_c_hat] = ...
            ALOS3D(y.xn, y.yn, y.zn, guidance.Delta_h, guidance.Delta_v, ...
            guidance.gamma_h, guidance.gamma_v, guidance.M_theta, h, ...
            guidance.R_switch, path.wpt);
        alos = [y_e z_e alpha_c_hat beta_c_hat];
end

end
