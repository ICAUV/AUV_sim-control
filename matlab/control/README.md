# control

Feedback controllers. They take the measurements `y` (`physics/measurements.m`)
and the desired states `ref` (`navigation/`), with gains from `P.ctrl`
(`auv1Params.m`).

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

## propellerSpeed.m

`n_p = propellerSpeed(n_p, n_d, model)` moves the propeller speed towards
`n_d` by `n_rate` RPM per sample, capped at `n_max`. In `SIMauv1.m`,
`n_p` starts at 1000 RPM and `n_d` is 1300 RPM.
