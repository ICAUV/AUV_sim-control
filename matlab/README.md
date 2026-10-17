# matlab

Run `startup_auv1.m` first. It adds this folder, `mss/`, `navigation/`,
`control/` and `physics/` to the path.

## Files

| File | What it does |
|---|---|
| `SIMauv1.m` | Entry point. Sets up the run, steps the main loop, plots, returns `simOut`. An optional third argument replaces `pathParams()` |
| `auv1.m` | Vehicle dynamics, `xdot = auv1(x, ui, Vc, betaVc, wc, P.model)` |
| `auv1Params.m` | `P.model`, `P.ctrl` and `P.guidance`. `auv1Params('remus100')` gives the original REMUS 100 values and actuators |
| `pathParams.m` | `P.path`: waypoints (`wpt`) and the step command schedule (`step`) |
| `fin_mixer.m`, `deflection_to_pwm.m` | Fin mixing and PWM conversion, ports of the vehicle's `auv1-software` |
| `data/thrusters/` | T500 and T200 thrust tables (N vs PWM) |
| `auv1Visualiser.m` | 3-D replay of a run, `auv1Visualiser(simOut)` |
| `PARAMS_CHANGELOG.md` | AUV1 parameters that differ from REMUS 100, with sources and known gaps |
| `startup_auv1.m` | Path setup |

## Main loop (SIMauv1.m)

One pass per sample, `h = 0.05` s, for 1400 s:

```
y   = measurements(x)                                       physics/
[cmd, alos] = navigation(t, y, ControlFlag, P.path, P.guidance, h)  navigation/
ref.z_d = LP.z_d(cmd.z_ref)                   flags 1-2: low-pass filtered depth command
[delta_r, delta_s] = trackingController(y, ref, theta_int, ...)   control/
ref.theta_d = depthController(y, ref, z_int, ...)     flags 1-3   control/
ref = updateReferences(ref, cmd, ...)                       navigation/
[ui, ui_log, act] = actuatorInputs(delta_r, delta_s, act, P.model, h)   control/
[Vc, betaVc] = oceanCurrent(t, Vc, betaVc, h)         flag 4      physics/
x   = physicsStep(x, ui, Vc, betaVc, wc, P.model, h)        physics/
z_int, theta_int += h * error                               integral states
```

The fin commands at t(i) are computed from the references at t(i); the
references are then moved on to t(i+1). `actuatorInputs` turns the rudder and
stern-plane commands into what `auv1.m` takes, depending on
`P.model.actuators`:

- `'auv1'` (default): six PWM signals `[top bottom left right stern bow]` (us).
  The fin commands go through `fin_mixer.m` with a servo slew limit, the stern
  thruster runs at 1700 us through a first-order lag, the bow thruster is held
  at neutral (1500 us).
- `'remus100'`: `[delta_r; -delta_s; n_p]`, rudder and stern planes (rad) and
  propeller speed (RPM).

| Variable | Meaning |
|---|---|
| `x` | State `[u v w p q r x y z phi theta psi]'`, or with the quaternion `[eta eps1 eps2 eps3]` in place of the Euler angles |
| `y` | Measurements: `u w q r xn yn zn theta psi` |
| `cmd` | Navigation commands: `z_ref`, `psi_ref`, `theta_ref` (flag 4) |
| `ref` | Desired states: `z_d zdot_d theta_d q_d psi_d r_d a_d` |
| `ui` | Model inputs, see above |
| `ui_log` | Logged inputs `[delta_r -delta_s stern]`, stern = thruster PWM (us) or propeller speed (RPM) |
| `act` | Actuator states: fin angles, stern PWM (AUV1) or propeller speed (REMUS) |
| `Vc`, `betaVc`, `wc` | Ocean current speed, direction, vertical speed |
| `P` | `P.model`, `P.ctrl`, `P.guidance`, `P.path` |

Ocean current: constant 0.5 m/s at 30 deg for flags 1-3; time-varying for
flag 4 (`physics/oceanCurrent.m`). Vertical current 0.1 m/s throughout.

## Output (simOut)

`t`, `h`, `ControlFlag`, `KinematicsFlag`, `eta` (position and Euler angles,
also for quaternion runs), `nu`, `ui` (= `ui_log`), `ref.{z_d, theta_d, psi_d, r_d}`,
`current.{Vc, betaVc, wc}`, `alos.{y_e, z_e, alpha_c_hat, beta_c_hat}`, `wpt`, `P`.

## Vehicle model (auv1.m)

```
xdot = [ Dnu_c + M \ (tau + tau_liftdrag + tau_crossflow - C*nu_r - D*nu_r - g)
         J * nu ]
```

| Term | From | Theory (theory/Slides) |
|---|---|---|
| `M = MRB + MA`, `C` | Spheroid hull (`spheroid`) and added mass (`imlay61`) | Ch 3, 6, 8.4 |
| `D` | Linear damping (`Dmtrx`), surge term fades with speed | Ch 6 |
| `tau_liftdrag`, `tau_crossflow` | Hull lift/drag and cross-flow drag | Ch 6, 8 |
| `g` | Restoring forces (`gRvect`), buoyancy `B = W + dB` | Ch 4.1 |
| `tau`, AUV1 | Lift and drag of each of the four fins; T500 stern and T200 bow thrust from the PWM tables | Ch 9 |
| `tau`, REMUS | Propeller thrust/torque from KT, KQ vs advance number; rudder and stern-plane lift | Ch 9 |
| `nu_r = nu - nu_c` | Ocean current in BODY | Ch 10.3 |
| `J` | Kinematics, Euler angles (`eulerang`) or quaternion (`quatern`) | Ch 2 |

The AUV1 thrust tables are static bollard data, so thrust is overpredicted at
speed; the other known gaps are listed in `PARAMS_CHANGELOG.md`.
