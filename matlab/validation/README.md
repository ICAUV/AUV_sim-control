# validation

`validate_auv1_params_a.m` checks the AUV1 parameter set from pass A (hull,
mass, buoyancy, fins; see `../PARAMS_CHANGELOG.md`). Run with
`run validate_auv1_params_a`.

| Step | Check | Output |
|---|---|---|
| 1 | Equivalence test, `auv1` vs `remus100` | log |
| 2 | At rest, neutral buoyancy (`dB = 0`), 60 s: velocities stay below 1e-6 | `at_rest_dB0.png` |
| 3 | At rest, `dB = 2` N from 20 m: steady rise speed, compared with a hand calculation (cross-flow drag balance) | `at_rest_dB2_rise.png` |
| 4 | Free decay from 10 deg pitch and roll: linearised modes, settling time, period and damping | `free_decay_pitch_roll.png` |
| 5 | `SIMauv1(1, 1)` runs end to end with no non-finite values | `SIMauv1_opt1_euler_fig*.png` |

The text log goes to `validation_results.txt`. The files here are from the last
time the script was run; step 5 used the path in `pathParams.m` at that time.
