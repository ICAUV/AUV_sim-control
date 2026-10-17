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

The files here are from the last time each script was run; the `SIMauv1` step
used the path in `pathParams.m` at that time.
