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
