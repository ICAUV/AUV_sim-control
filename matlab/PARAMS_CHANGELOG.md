# AUV1 parameter change log

History of the `auv1Params('auv1')` values that differ from the REMUS 100 set
(`auv1Params('remus100')`). The current values and their sources are in the
comments of `auv1Params.m`; this file records what changed, when and why.

## Pass A: hull, mass, buoyancy, fins (2026-10-16)

| Date | Parameter | Old (REMUS 100) → New (AUV1) | Unit | Source | Uncertainty | Reason |
|------|-----------|------------------------------|------|--------|-------------|--------|
| 2026-10-16 | `L_auv` | 1.6 → 1.1 | m | Orientation deck 13 Oct 2026, slide 4; CAD spine rod 1030 mm | ±0.05 | AUV1 hull length |
| 2026-10-16 | `D_auv` | 0.19 → 0.16 | m | Orientation deck 13 Oct 2026, slide 4 | ±0.01 | AUV1 hull diameter |
| 2026-10-16 | `m_target` (new, not stored in `P.model`) | 31.9 → 22 | kg | Deck: ~20 kg dry, plus water in free-flooded sections (≈ ρ·π(D/2)²·L) | ESTIMATE, ±20% | Mass moving with the hull |
| 2026-10-16 | `a` (derived) | 0.80768 → 0.62333 | m | Computed from `m_target`, `L_auv`, `D_auv` (spheroid scale factor k ≈ 1.133, replaces the hard-coded 1.0096) | follows `m_target` | Spheroid returns `m_target` (checked to 0.1% in `auv1Params`) |
| 2026-10-16 | `b` (derived) | 0.095912 → 0.090666 | m | As for `a` | follows `m_target` | As for `a` |
| 2026-10-16 | `S` (derived) | 0.2128 → 0.1232 | m² | 0.7·L_auv·D_auv | follows `L_auv`, `D_auv` | Follows hull size |
| 2026-10-16 | `CD_0` (derived) | 0.05704 → 0.08804 | – | Cd·π·b²/S, with Cd unchanged | follows `a`, `b`, `S` | Follows hull size |
| 2026-10-16 | `rho` | 1025 → 1000 | kg/m³ | Fresh water (pool) | 996–1000 for 10–30 °C | Pool tests. The MSS helpers still use 1025 internally (~2.5% on mass, added mass and hull drag). Known gap, `mss/` not edited |
| 2026-10-16 | `g_mu` | 9.82178 → 9.81203 | m/s² | Latitude 63.4468° (Trondheim) → 51.5° (London) | negligible | Vehicle operates in London |
| 2026-10-16 | `dB` (new) | 0 → 2 | N | Slightly positive net buoyancy, TG2 requirement | ESTIMATE, to be measured at the float test | `B = W + dB` in `auv1.m` |
| 2026-10-16 | `S_fin` | 0.00665 → 0.0096 | m² | TG5 brief, real blade | not stated | AUV1 fin geometry |
| 2026-10-16 | `A_r`, `A_s` (derived) | 0.0133 → 0.0192 | m² | 2·`S_fin` | follows `S_fin` | Follows `S_fin` |
| 2026-10-16 | `x_r`, `x_s` | −0.80768 → −0.45 | m | Position from mid-length | ESTIMATE, ±0.05, until the hinge is measured | No longer tied to `a` |
| 2026-10-16 | `delta_max` | 20° → 25° (0.3491 → 0.4363) | rad | `auv1-sim/config/vehicle.json` | unverified | AUV1 fin deflection limit |

Values documented as ESTIMATE in `auv1Params.m` but unchanged from REMUS 100:
`r_bG` (±0.01 m until the TG2 tilt test), `r_bB`, `CL_delta_r`, `CL_delta_s`,
`Cd`, `r44`, `T1`, `T2`, `T6`, `zeta4`, `zeta5`, `z_max`. The propeller block
(`n_max`, `n_rate`, `D_prop`, `t_prop`, `Va_ratio`, `Ja_max`, `KT_*`, `KQ_*`)
is a REMUS placeholder until pass B.

Knock-on effect, not a parameter change: the pitch PID gains in `P.ctrl` are
derived from `M(5,5)` of the model, so they change with the new hull
(`Kp_theta` 5.007 → 2.019, `Kd_theta` 10.01 → 4.038, `Ki_theta` 0.4006 → 0.1615).

## Pass B: actuators, T500 stern thruster, four fins, T200 bow thruster (2026-10-16)

The AUV1 parameter set now uses `model.actuators = 'auv1'`: `ui` is the six PWM
signals `[top bottom left right stern bow]` (us) of the autopilot. The
`'remus100'` set keeps `model.actuators = 'remus100'` and its original code
path in `auv1.m`.

| Date | Parameter | Old (REMUS 100) → New (AUV1) | Unit | Source | Uncertainty | Reason |
|------|-----------|------------------------------|------|--------|-------------|--------|
| 2026-10-16 | `actuators` (new) | `'remus100'` → `'auv1'` | – | Model structure, pass B | – | Selects the actuator model in `auv1.m` and the input vector (3 → 6 elements) |
| 2026-10-16 | `ui0` (new) | `[0 0 0]'` → `[1500 1500 1500 1500 1500 1500]'` | rad, rad, rpm → us | Neutral PWM, `auv1/fin_mixer.py`, `vehicle_daemon.py` | – | Neutral inputs, used for the empty `auv1()` call and the mass-matrix call in `auv1Params` |
| 2026-10-16 | `m_target` (now stored in `P.model`) | 31.9 → 22 | kg | See pass A | ESTIMATE, ±20% | Stored so the mass target is available outside `auv1Params` (REMUS set stores 31.9) |
| 2026-10-16 | `x_fin` (new) | −0.80768 (`x_r`, `x_s` = −`a`) → −0.45 | m | Pass A fin position | ESTIMATE, ±0.05 | One fin position for all four fins; `x_r` and `x_s` now equal `x_fin` |
| 2026-10-16 | `r_fin` (new) | – → 0.13 | m | Radial position of the fin centre of pressure | ESTIMATE, ±0.02 | Fin lever arm for roll and for the fin positions |
| 2026-10-16 | `fin_sign` (new) | – → [−1 +1 −1 +1] (top, bottom, left, right) | – | Consistent with the pitch and yaw columns of `fin_mixer.SIGNS` under the `vehicle_control.py` conventions; matches `auv1-sim/config/vehicle.json` | ESTIMATE, until the per-fin bench test (one fin at a time to PWM 1900) | Direction of each fin's force for a positive deflection. With these signs the mixer's roll column does not add up to a clean roll moment (see `tests/test_auv1_actuator_signs.m`) |
| 2026-10-16 | `pwm_centre`, `pwm_range` (new) | – → 1500, 400 | us | `auv1/fin_mixer.py` | – | Fin PWM = 1500 + 400·deflection |
| 2026-10-16 | Fin PWM to angle (new) | – → ±400 us = ±`delta_max` (25°), linear | – | Assumed: full servo range ±400 us = `delta_max` (25°, `auv1-sim/config/vehicle.json`) | ESTIMATE, until the bench test measures the angle at PWM 1900 | Fin angle from PWM |
| 2026-10-16 | `x_bow` (new) | – → +0.40 | m | Bow tunnel position | ESTIMATE, ±0.05 | Yaw moment arm of the bow thruster |
| 2026-10-16 | `stern_dir`, `bow_dir` (new) | – → +1, +1 | – | Stern thrust along +x for PWM > 1500, bow thrust along +y for PWM > 1500 | ESTIMATE, until a props-on bench check | Thrust directions |
| 2026-10-17 | `T500` (new, replaces the REMUS propeller in the AUV1 set) | REMUS propeller (`KT_*`, `KQ_*`, rpm) → table `data/thrusters/T500_14v8.csv` | N vs PWM | Blue Robotics `T500-Public-Performance-Data-12-24V-August-2022.xlsx`, sheets "14 V" and "16 V" (the sheets bracketing 14.8 V), force (kgf) × 9.80665, linear in voltage to 14.8 V; built by `data/thrusters/make_thrust_tables.m`. Stern thruster | Manufacturer uncertainty not stated. Static bollard data | Stern thrust from PWM. Interpolated between 14 V and 16 V (first version used 12 V and 16 V) |
| 2026-10-17 | `T200` (new) | – → table `data/thrusters/T200_14v8.csv` | N vs PWM | Blue Robotics `T200-Public-Performance-Data-10-20V-September-2019.xlsx`, sheets "14 V" and "16 V", same method. Bow tunnel thruster | As for `T500` | Bow thrust from PWM. Interpolated between 14 V and 16 V (first version used 12 V and 16 V) |
| 2026-10-16 | `servo_slew` (new) | – → 300 °/s (5.236 rad/s) | rad/s | `auv1-sim/config/vehicle.json` | Unverified | Fin servo rate limit in `SIMauv1` |
| 2026-10-16 | `thruster_tau` (new) | – → 0.15 | s | `auv1-sim/config/vehicle.json` | Unverified | Thruster first-order lag in `SIMauv1` (applied to the stern PWM) |

Unchanged but no longer used by the AUV1 actuators: the REMUS propeller block
(`n_max`, `n_rate`, `D_prop`, `t_prop`, `Va_ratio`, `Ja_max`, `KT_*`, `KQ_*`),
`A_r`, `A_s`. They stay in the struct because the `'remus100'` code path
needs them (`n_max` and `n_rate` are also read by `SIMauv1`).
`CL_delta_r` = 0.5 and `CL_delta_s` = 0.7 (REMUS effective values, ESTIMATE)
are now used per fin with `S_fin`.

Known gaps of the AUV1 actuator model (also in the `auv1.m` header): the thrust
is bollard data with no loss with forward speed (overpredicts at speed), no
thrust deduction, no propeller torque in roll, no fin stall, no local flow
angle at the fins.

## Pitch autopilot gains (2026-10-27)

The gains were `M(5,5)*w^2` and `M(5,5)*2*zeta*w`, in torque units, while
`trackingController.m` outputs `delta_s` in rad. They now come from
`control/pitchGains.m`, pole placement on the linearised pitch model
`m_th*theta_dd + d_th*q + k_th*theta = M_delta*delta_s` at one design speed
`U0`. The four model terms are central finite differences of `auv1.m` about level
flight at `U0` (fins neutral, other states at trim), repeated and checked by
`validation/validate_pitch_autopilot.m`. They are not recomputed in
`auv1Params.m`: after changing any `P.model` value, redo them. The REMUS 100
set keeps the old gains.

| Date | Parameter | Old → New | Unit | Source | Uncertainty | Reason |
|------|-----------|-----------|------|--------|-------------|--------|
| 2026-10-27 | `U0_theta` (new) | – → 2.25 | m/s | Through-water speed of `SIMauv1(1,1)` with the stern command 1700 us, steady from t = 100 s (2.2487–2.2499) | ESTIMATE, ±0.01 in the simulator. Real speed is lower: bollard thrust table | Single design speed of the pitch gains |
| 2026-10-27 | `m_theta` (new) | – → 3.146 | kg m² | Finite difference of `auv1.m`; `M(5,5)` = 3.1544 (finite difference −0.27%: surge coupling `M(1,5)` = m·zg) | ESTIMATE, follows `m_target` (±20%) | Pitch inertia incl. added mass |
| 2026-10-27 | `d_theta` (new) | – → 6.350 | N m s/rad | Finite difference of `auv1.m` at `U0`: `D(5,5)`, hull lift, cross-flow drag. Fins add none (no local flow angle at the fins in `auv1.m`) | ESTIMATE, ±0.45 (5.907 with w0 = 0 instead of the trim value) | Linear pitch damping |
| 2026-10-27 | `k_theta` (new) | – → 4.356 | N m/rad | Finite difference of `auv1.m`; `W*BG_z` = 4.317 (finite difference +0.9%: `dB*sin(theta)` in surge through `M(1,5)`) | ESTIMATE, follows `r_bG` (±0.01 m in z, i.e. ±50%) | Restoring stiffness |
| 2026-10-27 | `M_delta_theta` (new) | – → 15.31 | N m/rad of `delta_s` | Finite difference of pitch command → `fin_mixer` → `deflection_to_pwm` → `auv1.m` at `U0`; sign checked against `test_auv1_actuator_signs.m` | ESTIMATE, ±25% (`x_fin` ±0.05 m is ±11%, `CL_delta_s` is the REMUS value) | Pitch moment per rad of `delta_s`; scales with `U0²` |
| 2026-10-27 | `w_theta` | 0.8 → 2.0 | rad/s | Design choice | – | Above 1.18 rad/s (`Kp` > 0) and 1.26 rad/s (`Kd` > 0). A 10° step in `theta_d` gives a peak `delta_s` of 5.4° (22% of `delta_max`, 5.4° of 15° per sample servo slew); a 30° step (`theta_max`) gives 16° |
| 2026-10-27 | `zeta_theta` | 0.8 → 0.8 (unchanged) | – | Design choice | – | Kept near 0.8 |
| 2026-10-27 | `Kp_theta` | 2.0188 → 0.5374 | rad/rad | `pitchGains.m`: (m_th·w² − k_th)/M_delta | follows the model terms | Old value had torque units, no restoring stiffness or fin effectiveness |
| 2026-10-27 | `Kd_theta` | 4.0376 → 0.2428 | rad/(rad/s) | `pitchGains.m`: (2·zeta·w·m_th − d_th)/M_delta | follows the model terms | As for `Kp_theta`. Old value was 16 times too large |
| 2026-10-27 | `Ki_theta` | 0.1615 → 0.1075 | rad/(rad s) | `pitchGains.m`: Kp·w/10 | follows `Kp_theta` | Same rule as before, new `Kp` and `w` |

Effect on the step response (`validate_pitch_autopilot.m`, 10° `theta_d` step at
`U0`): 10–90% rise time 6.8 s, no overshoot, 2% settling time 18 s, against 1.2 s,
1.5% and 1.9 s for the ideal second order system with `w` and `zeta`. The
proportional and derivative terms alone reach `1 − k_th/(m_th·w²)` = 65% of the
step and `Ki = Kp·w/10` supplies the rest through a slow pole at −0.15 rad/s.

Regression baseline `tests/reference/SIMauv1_baseline.mat` rewritten: depth
differs by at most 0.65 m and pitch by at most 2.65° from the old gains in the
8 runs (largest in ControlFlag 1-2), no change in stability or oscillation.

## Pitch restoring-moment feedforward (2026-10-28)

With `k_th` > 0 the pitch PD alone reaches only `1 − k_th/(m_th·w²)` = 65% of a
`theta_d` step and the integral cleared the rest slowly. `trackingController.m`
now adds `Kff_theta*theta_d` to `delta_s`, with `Kff_theta = k_th/M_delta`
from `control/pitchGains.m` (fourth output). The poles do not change. `Ki`
stays `Kp·w/10`. The REMUS 100 set has `Kff_theta = 0`, so its behaviour is
unchanged.

| Date | Parameter | Old → New | Unit | Source | Uncertainty | Reason |
|------|-----------|-----------|------|--------|-------------|--------|
| 2026-10-28 | `Kff_theta` (new) | – → 0.2845 | rad/rad | `pitchGains.m`: k_th/M_delta = 4.356/15.31 | ESTIMATE, ±56% (`k_theta` ±50%, `M_delta_theta` ±25%, in quadrature) | Stern angle that balances the restoring moment at `theta_d` |

Effect on the 10° `theta_d` step at `U0` (`validate_pitch_autopilot.m`,
nonlinear simulation): 10–90% rise time 6.8 s → 1.1 s, overshoot 0% → 11%,
2% settling time 18.3 s → 13.6 s; peak `delta_s` 5.4° → 8.2° (33% of
`delta_max`, 8.2° of 15° per sample servo slew). The overshoot comes from the
integral term. With the plant `M_delta` at 0.75 and 1.25 times the stored value
and the controller unchanged: overshoot 2.2% and 17.2%, 2% settling time 4.3 s
and 17.5 s.

Regression baseline `tests/reference/SIMauv1_baseline.mat` rewritten: against
the previous baseline depth differs by at most 1.83 m and pitch by at most 6.7°
(ControlFlag 1-2, which reach the `theta_max` limit), 0.16 m and 0.71° in
ControlFlag 3, 0.94 m and 3.3° in ControlFlag 4. `alpha_c_hat` in ControlFlag 4
changes by +0.009° at its maximum and +0.0003° at the end. No depth response
became oscillatory or unstable.
