# validation

Checks of the AUV1 parameter set, one script per pass (see
`../PARAMS_CHANGELOG.md`). Each writes a text log and plots into this folder.

## validate_auv1_params_a.m (hull, mass, buoyancy, fins)

| Step | Check | Output |
|---|---|---|
| 1 | Equivalence test, `auv1` vs `remus100` | log |
| 2 | At rest, neutral buoyancy (`dB = 0`), 60 s: velocities stay below 1e-6 | `at_rest_dB0.png` |
| 3 | At rest, `dB = 2` N from 20 m: steady rise speed, compared with a hand calculation (cross-flow drag balance) | `at_rest_dB2_rise.png` |
| 4 | Free decay from 10 deg pitch and roll: linearised modes, settling time, period and damping | `free_decay_pitch_roll.png` |
| 5 | `SIMauv1(1, 1)` runs end to end with no non-finite values | `SIMauv1_opt1_euler_fig*.png` |

Log: `validation_results.txt`.

## validate_auv1_params_b.m (T500 stern thruster, four fins, T200 bow thruster)

| Step | Check | Output |
|---|---|---|
| 0 | Thrust tables at 14.8 V | `thrust_tables_14v8.png` |
| 1, 2 | Equivalence test and actuator sign test | log |
| 3 | Pass A results again, with neutral PWM | log |
| 4 | TG5 case 2: top speed for stern PWM 1600-1900 | `tg5_case2_top_speed.png` |
| 5 | TG5 case 3: turning circle at 1 m/s, 20 deg rudder | `tg5_case3_turning_circle.png` |
| 6 | TG5 case 4: 10 deg stern-plane step at 1 m/s | `tg5_case4_stern_plane_step.png` |
| 7 | `SIMauv1(1, 1)` runs end to end | `passB_SIMauv1_opt1_euler_fig*.png` |

Log: `validation_results_pass_b.txt`.

## validate_pitch_autopilot.m (pitch PID gains and feedforward)

| Step | Check | Output |
|---|---|---|
| 1 | Finite differences of `auv1.m` for `m_th`, `d_th`, `k_th`, `M_delta` at `U0`, against the values in `auv1Params.m` (within 0.5%), `M(5,5)` and `W*BG_z` | console |
| 2 | `pitchGains.m` on the finite differences against `Kp`, `Kd`, `Ki`, `Kff` in `auv1Params.m` | console |
| 3 | 10 deg `theta_d` step from trim at `U0`: nonlinear, linearised and ideal second-order response (rise time, overshoot, settling time), no fin saturation or servo slew limit; repeated with the plant `M_delta` scaled by 0.75 and 1.25 and the plant `k_th` scaled by 0.5 and 1.5 (via `r_bG(3)`; `auv1.m` takes the pitch damping from `W*BG_z` too, so `d_th` moves by about the square root of the scale), controller unchanged | `validate_pitch_autopilot.png` |

No text log; the script prints the tables and stops on a failed check.

The files here are from the last time each script was run; the `SIMauv1` step
used the path in `pathParams.m` at that time.
