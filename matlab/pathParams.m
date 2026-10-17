function p = pathParams()
% p = pathParams() returns the path for SIMauv1 to follow:
%
%   p.wpt   - Waypoints for 2.5-D and 3-D ALOS path following (ControlFlag 3-4)
%   p.step  - Depth and heading step commands for the autopilots (ControlFlag 1-2)
%
% Both are test manoeuvres: sharp turns both ways, yaw crossing +-180 deg,
% dives and climbs alone and while turning. At about 2.2-2.5 m/s the vehicle
% finishes the waypoints within the 1400 s run.
%
% Example usage:
%   P = auv1Params();
%   P.path = pathParams();

% Waypoints, NED (m). Legs must be longer than R_switch (auv1Params.m) and
% long enough after a sharp turn to converge back onto the path.
p.wpt.pos.x = [0  250  250 -100 -100  250  550  200  200];
p.wpt.pos.y = [0    0  150  120  300  300  700 1050 1300];
p.wpt.pos.z = [0   20   20   40   40   25   85   15   10];
%   1 -> 2   North, dive to 20 m
%   2 -> 3   Right 90 deg, short leg
%   3 -> 4   Right 95 deg to -175 deg (yaw crosses +-180 deg), dive to 40 m
%   4 -> 5   Left 95 deg, short leg
%   5 -> 6   Left 90 deg, climb to 25 m
%   6 -> 7   Right 53 deg, steep dive to 85 m
%   7 -> 8   Right 82 deg, steep climb to 15 m
%   8 -> 9   Left 45 deg, shallow final leg

% Step commands: z(1) and psi(1) until t(1), then z(k+1) and psi(k+1) after t(k)
p.step.t = [200 400 600 800 1000];             % Step times (s)
p.step.z = [10 30 30 60 60 15];                % Depth commands (m), max 100 m
p.step.psi = deg2rad([0 -60 170 170 -150 90]); % Heading commands (rad)
%   0-200 s      Dive to 10 m
%   200-400 s    Dive to 30 m and turn left 60 deg
%   400-600 s    Turn left 130 deg across +-180 deg
%   600-800 s    Dive to 60 m
%   800-1000 s   Turn right 40 deg across +-180 deg
%   1000-1400 s  Climb to 15 m and turn left 120 deg across +-180 deg

end
