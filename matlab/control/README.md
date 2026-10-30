# control

Feedback controllers, and the step from their outputs to the actuators. The
controllers take the measurements `y` (`physics/measurements.m`) and the desired
states `ref` (`navigation/`), with gains from `P.ctrl` (`auv1Params.m`). They
output REMUS-style rudder and stern-plane angles; for AUV1 these are mixed to
the four fins in `actuatorInputs.m`.

## trackingController.m (inner loop)

`[delta_r, delta_s] = trackingController(y, ref, theta_int, ControlFlag, ctrl, delta_max, h)`

Stern planes, pitch PID with restoring-moment feedforward:

```
delta_s = Kff_theta * theta_d - Kp_theta * ssa(theta - theta_d) - Kd_theta * q - Ki_theta * theta_int
```

`delta_s` is in rad, so the gains are rad per rad, rad per rad/s and rad per
rad s. They come from `pitchGains.m`, pole placement (theory Ch 15.2-15.3) on
the linearised pitch model at one design speed `U0`:

```
m_th * theta_dd + d_th * q + k_th * theta = M_delta * delta_s
Kp  = (m_th*w^2 - k_th) / M_delta
Kd  = (2*zeta*w*m_th - d_th) / M_delta
Ki  = Kp * w / 10
Kff = k_th / M_delta
```

`Kff_theta` is the stern angle whose moment balances the restoring moment at
`theta_d`. Without it the proportional term has to hold the restoring moment
with a pitch error. `auv1Params('remus100')` sets `Kff_theta = 0`.

| Term | AUV1 value | Meaning |
|---|---|---|
| `U0_theta` | 2.25 m/s | Through-water speed `SIMauv1(1,1)` reaches with the stern command (1700 us) |
| `m_theta` | 3.146 kg m2 | Pitch inertia incl. added mass (`M(5,5)` = 3.154) |
| `d_theta` | 6.350 N m s/rad | Linear pitch damping at `U0` (hull; the fins add none in `auv1.m`) |
| `k_theta` | 4.356 N m/rad | Restoring stiffness (`W*BG_z` = 4.317) |
| `M_delta_theta` | 15.31 N m/rad | Pitch moment per rad of `delta_s` at `U0` |

The four values are central finite differences of `auv1.m` about level flight at
`U0` (fins neutral, other states at trim). They are stored in `auv1Params.m`,
not recomputed, so redo them after changing `P.model`:
`validation/validate_pitch_autopilot.m` repeats the differences and stops if the
stored values are out of date. `M_delta` scales with `U0^2`, so the gains hold
at `U0` only. `auv1Params('remus100')` keeps the original gains from `M(5,5)`.

`w_theta` = 2.0 rad/s, `zeta_theta` = 0.8 give `Kp` = 0.537, `Kd` = 0.243,
`Ki` = 0.108, `Kff` = 0.284 (before: 2.019, 4.038, 0.162, none). The hull alone
has 1.18 rad/s and `zeta` = 0.86, so `Kp` and `Kd` are positive only above
`w` = 1.26 rad/s. A 10 deg step in `theta_d` takes `delta_s` to 8.2 deg (33% of
`delta_max`, a change of 8.2 deg per sample against the 15 deg per sample servo
slew limit): 5.4 deg from `Kp` and 2.8 deg from `Kff`. A step of `theta_max`
(30 deg) asks for 24.7 deg, just inside `delta_max` and above the slew limit.
Peak pitch rate 8.9 deg/s.

The feedforward does not move the poles (`-1.53 +- 1.10j`, `w` = 1.88 rad/s,
`zeta` = 0.81, and `-0.15` rad/s from `Ki`). It gives the proportional and
feedforward terms together a DC gain of 1. Without it they reached only
`1 - k_th/(m_th*w^2)` = 65% of the step and the integral cleared the rest
slowly (2% settling time 18.3 s). Step response in the nonlinear simulation now:
10-90% rise time 1.1 s, 11% overshoot, 2% settling time 13.6 s (ideal second
order: 1.2 s, 1.5%, 1.9 s). The overshoot comes from the integral, which
collects error while the pitch rises and then needs time to unwind.

`Kff` is only as good as `M_delta`, which is an ESTIMATE (+-25%). With the plant
`M_delta` at 0.75 and 1.25 times the stored value and the controller unchanged,
the proportional and feedforward terms alone would settle at 89.7% and 107.4% of
the step, and the integral has to clear that. In the nonlinear simulation:
overshoot 2.2% and 17.2%, 2% settling time 4.3 s and 17.5 s
(`validation/validate_pitch_autopilot.m`).

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
