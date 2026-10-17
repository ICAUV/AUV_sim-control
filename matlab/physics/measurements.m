function y = measurements(x)
% y = measurements(x) returns the states used by navigation and control from
% the state vector x (12 states, Euler angles, or 13, unit quaternions).

y.u = x(1);                 % Surge velocity (m/s)
y.w = x(3);                 % Heave velocity (m/s)
y.q = x(5);                 % Pitch rate (rad/s)
y.r = x(6);                 % Yaw rate (rad/s)
y.xn = x(7);                % North position (m)
y.yn = x(8);                % East position (m)
y.zn = x(9);                % Down position (m), depth

if length(x) == 12          % Euler angles
    y.theta = x(11); y.psi = x(12);
else                        % Quaternion to Euler angles
    [~, y.theta, y.psi] = q2euler(x(10:13));
end

end
