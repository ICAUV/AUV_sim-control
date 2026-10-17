function [Vc, betaVc] = oceanCurrent(t, Vc, betaVc, h)
% [Vc, betaVc] = oceanCurrent(t, Vc, betaVc, h) is the time-varying ocean
% current of the 3-D ALOS scenario (ControlFlag 4): the speed Vc (m/s) changes
% after t = 800 s and the direction betaVc (rad) after t = 500 s, both with
% small random disturbances.

if t > 800
    Vc_d = 0.65;
    w_V = 0.05;
    Vc = exp(-h*w_V) * Vc + (1 - exp(-h*w_V)) * Vc_d;
else
    Vc = 0.5;
end

if t > 500
    betaVc_d = deg2rad(160);
    w_beta = 0.1;
    betaVc = exp(-h*w_beta) * betaVc + (1 - exp(-h*w_beta)) * betaVc_d;
else
    betaVc = deg2rad(150);
end

betaVc = betaVc + randn / 1000;
Vc = Vc + 0.002 * randn;

end
