function [ui, ui_log, act] = actuatorInputs(delta_r, delta_s, act, model, h)
% [ui, ui_log, act] = actuatorInputs(delta_r, delta_s, act, model, h) turns the
% autopilot fin commands into the inputs of auv1.m for the actuator set
% model.actuators, and steps the actuator states in act:
%
%   'auv1'      ui = [top bottom left right stern bow]' PWM (us). Fins through
%               fin_mixer.m with a servo slew-rate limit, stern thruster at
%               act.pwm_stern_cmd through a first-order lag, bow at neutral.
%   'remus100'  ui = [delta_r -delta_s n_p]'. Rudder and stern planes (rad),
%               propeller speed (RPM) rate limited by propellerSpeed.m.
%
% ui_log = [delta_r -delta_s stern]' is what SIMauv1 logs, with the stern
% thruster PWM (us) or the propeller speed (RPM) last.

if strcmp(model.actuators, 'auv1')
    % Normalised yaw and pitch commands with the same moment sign as the
    % REMUS rudder and stern planes (+yaw_cmd: N > 0, +pitch_cmd: M > 0),
    % mixed to the four fins as on the vehicle
    yaw_cmd = delta_r / model.delta_max;
    pitch_cmd = delta_s / model.delta_max;
    roll_cmd = 0;                           % No roll loop yet
    [~, deflection] = fin_mixer(pitch_cmd, roll_cmd, yaw_cmd);

    % Servo slew-rate limit on the fin angles
    d_max = model.servo_slew * h;
    act.fin_angle = act.fin_angle ...
        + max(-d_max, min(d_max, deflection * model.delta_max - act.fin_angle));
    pwm_fins = deflection_to_pwm(act.fin_angle / model.delta_max);

    % Stern thruster PWM with first-order lag, bow thruster at neutral
    a_thruster = exp(-h / model.thruster_tau);
    act.pwm_stern = a_thruster * act.pwm_stern + (1 - a_thruster) * act.pwm_stern_cmd;

    ui = [pwm_fins; act.pwm_stern; act.pwm_bow];
    ui_log = [delta_r -delta_s act.pwm_stern]';
else
    act.n_p = propellerSpeed(act.n_p, act.n_d, model);
    ui = [delta_r -delta_s act.n_p]';
    ui_log = ui;
end

end
