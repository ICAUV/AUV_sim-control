function x = physicsStep(x, ui, Vc, betaVc, wc, model, h)
% x = physicsStep(x, ui, Vc, betaVc, wc, model, h) propagates the AUV1 state
% one sample, x(k) -> x(k+1), with the control inputs ui and ocean current
% (Vc, betaVc, wc).

if length(x) == 12
    % Euler angles x = [ u v w p q r x y z phi theta psi ]'
    x = rk4(@auv1, h, x, ui, Vc, betaVc, wc, model);  % RK4 method x(k+1)
else
    % Unit quaternions x = [ u v w p q r x y z eta eps1 eps2 eps3 ]'
    xdot = auv1(x, ui, Vc, betaVc, wc, model);
    quat = x(10:13);                            % Unit quaternion
    x(1:6) = x(1:6) + h * xdot(1:6);            % Forward Euler
    x(7:9) = x(7:9) + h * Rquat(quat) * x(1:3); % Backward Euler
    quat = expm(Tquat(x(4:6)) * h) * quat;      % Exact quat. discretization
    x(10:13) = quat / norm(quat);               % Normalization
end

end
