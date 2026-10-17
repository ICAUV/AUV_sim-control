# control

Feedback controllers, and the step from their outputs to the actuators. The
controllers take the measurements `y` (`physics/measurements.m`) and the desired
states `ref` (`navigation/`), with gains from `P.ctrl` (`auv1Params.m`). They
output REMUS-style rudder and stern-plane angles; for AUV1 these are mixed to
the four fins in `actuatorInputs.m`.

## trackingController.m (inner loop)

`[delta_r, delta_s] = trackingController(y, ref, theta_int, ControlFlag, ctrl, delta_max, h)`

Stern planes, pitch PID:

```
delta_s = -Kp_theta * ssa(theta - theta_d) - Kd_theta * q - Ki_theta * theta_int
```

The gains come from pole placement on the pitch inertia `M(5,5)` with
`w_theta` and `zeta_theta` (theory Ch 15.3).

Rudder, integral sliding mode heading autopilot `integralSMCheading` on a
first-order Nomoto model (`K_yaw`, `T_yaw`), tracking `psi_d`, `r_d`, `a_d`:

| Flag | Gains | Mode |
|---|---|---|
| 1 | `ctrl.pid` (`K_sigma = 0`, so plain PID) | `lambda` |
| 2 | `ctrl.smc` | `lambda` |
| 3-4 | `ctrl.smc` | `lambda_alos` |

Both outputs are saturated at `delta_max`. Theory: SMC heading autopilot
Ch 16.4.4, Nomoto model Ch 7.2.

## depthController.m (outer loop, flags 1-3)

`theta_d = depthController(y, ref, z_int, ControlFlag, ctrl, h)`

Finds the pitch angle that brings the depth to `z_d`. With the residual

```
sigma = -u*sin(theta) + w*cos(theta) + Kp_z*((z - z_d) + z_int/T_z)
```

`theta_d` takes one gradient-descent step on `J = sigma^2/2` per sample
(gain `k_grad`), limited to `theta_max`. Flags 1-2 use the simplified gradient
`-sign(u)` when `|w/u| < tan(10 deg)`.

The pitch PID above then tracks `theta_d`: successive loop closure, depth
outside pitch (theory Ch 15.2). The gradient law itself is from Fossen and
Coates (2026), cited in `SIMauv1.m`.

## actuatorInputs.m

`[ui, ui_log, act] = actuatorInputs(delta_r, delta_s, act, model, h)` turns
the rudder and stern-plane commands into the inputs of `auv1.m`, for the
actuator set in `model.actuators`. `act` carries the actuator states between
samples (set up in `SIMauv1.m`).

AUV1 (`'auv1'`, the default), `ui` = six PWM signals (us):

1. `yaw_cmd = delta_r/delta_max`, `pitch_cmd = delta_s/delta_max`, no roll
   command, mixed to `[top bottom left right]` deflections by `fin_mixer.m`
   (same sign table as the vehicle software)
2. Fin angles follow the deflections at most `servo_slew` per second, then
   `deflection_to_pwm.m` gives 1500 +- 400 us
3. Stern thruster PWM goes to `act.pwm_stern_cmd` (1700 us) through a
   first-order lag (`thruster_tau`); the bow thruster stays at 1500 us

Mixing commands onto several actuators is control allocation (theory Ch 11.2).

REMUS 100 (`'remus100'`): `ui = [delta_r; -delta_s; n_p]`, with `n_p` from
`propellerSpeed.m`.

`ui_log = [delta_r; -delta_s; stern]` is what `SIMauv1` logs, with the stern
thruster PWM or the propeller speed last.

## propellerSpeed.m

`n_p = propellerSpeed(n_p, n_d, model)` moves the propeller speed towards
`n_d` by `n_rate` RPM per sample, capped at `n_max`. Used by the REMUS
actuators only; `n_p` starts at 1000 RPM and `n_d` is 1300 RPM.
