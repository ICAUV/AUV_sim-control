function theta_d = depthController(y, ref, z_int, ControlFlag, ctrl, h)
% theta_d = depthController(y, ref, z_int, ControlFlag, ctrl, h) is the
% gradient-based outer depth loop: it propagates the desired pitch angle
% ref.theta_d to the next sample so that the depth tracks ref.z_d.
%
% Depth kinematics:
%   z_dot = -u * sin(theta) + w * cos(theta) := alpha_z + sigma
% Virtual control:
%   alpha_z = z_d_dot + Kp_z * (e_z + (1/T_z) * z_int)
%
% Outer-loop depth control residual sigma = 0 implies e_z = 0. theta_d is
% updated by gradient descent on J = 1/2 * sigma^2.

sigma = -y.u * sin(y.theta) + y.w * cos(y.theta) ...
    + ctrl.Kp_z * ((y.zn - ref.z_d) + (1/ctrl.T_z) * z_int);

% Gradient dJ/d(theta) = d(sigma)/d(theta) * sigma. ControlFlag 1-2 use a
% simplified gradient when abs(w/u) < 10 deg.
if ControlFlag == 3 || abs(y.w/y.u) > 0.176
    gradJ = -(y.u * cos(y.theta) + y.w * sin(y.theta));
else
    gradJ = -sign(y.u);
end

theta_d = ref.theta_d - h * ctrl.k_grad * gradJ * sigma;
theta_d = sat(theta_d, ctrl.theta_max);

end
