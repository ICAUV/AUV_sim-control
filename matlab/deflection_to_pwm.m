function pwm = deflection_to_pwm(deflection)
% pwm = deflection_to_pwm(deflection) converts a -1..+1 deflection to a servo
% PWM value in microseconds, clamped to 1100..1900.
%
% MATLAB port of deflection_to_pwm() in auv1/fin_mixer.py (auv1-software):
% PWM_CENTRE = 1500, PWM_RANGE = 400, and int() truncation toward zero.

PWM_CENTRE = 1500;
PWM_RANGE = 400;
pwm = fix(PWM_CENTRE + max(-1, min(1, deflection)) * PWM_RANGE);
end
