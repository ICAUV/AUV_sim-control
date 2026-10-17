function [fins, deflections] = fin_mixer(pitch_cmd, roll_cmd, yaw_cmd)
% [fins, deflections] = fin_mixer(pitch_cmd, roll_cmd, yaw_cmd) maps normalised
% commands (-1..+1) to fin deflections (-1..+1) for the "+" stern fin
% configuration.
%
% MATLAB port of auv1/fin_mixer.py (auv1-software), mix(). The SIGNS table and
% the clamp to -1..1 are identical. Do not change SIGNS here without changing
% it in auv1-software after re-verifying on the vehicle.
%
% Fin layout viewed from behind the vehicle:
%
%         TOP
%    LEFT  +  RIGHT
%        BOTTOM
%
%   TOP and BOTTOM act as rudders (yaw), LEFT and RIGHT as stern planes
%   (pitch), and all four differentially give roll.
%
% Conventions (vehicle_control.py): +pitch_cmd means nose up, +yaw_cmd means
% heading increasing (turn to starboard).
%
% Outputs:
%   fins:         struct with fields top, bottom, left, right
%   deflections:  [top bottom left right]' as a column vector

% Bench-verified direction facts, auv1-software 2026-09-04
SIGNS.top    = struct('yaw', +1, 'roll', -1);
SIGNS.bottom = struct('yaw', -1, 'roll', -1);
SIGNS.left   = struct('pitch', -1, 'roll', -1);
SIGNS.right  = struct('pitch', +1, 'roll', -1);

fins.top    = clamp(SIGNS.top.yaw * yaw_cmd + SIGNS.top.roll * roll_cmd);
fins.bottom = clamp(SIGNS.bottom.yaw * yaw_cmd + SIGNS.bottom.roll * roll_cmd);
fins.left   = clamp(SIGNS.left.pitch * pitch_cmd + SIGNS.left.roll * roll_cmd);
fins.right  = clamp(SIGNS.right.pitch * pitch_cmd + SIGNS.right.roll * roll_cmd);

deflections = [fins.top; fins.bottom; fins.left; fins.right];
end

function y = clamp(x)
y = max(-1, min(1, x));
end
