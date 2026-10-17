# tests

Each test runs `startup_auv1.m` itself, so they can be run from anywhere, e.g.
`run test_SIMauv1_regression`.

| Test | Checks | Time |
|---|---|---|
| `test_SIMauv1_regression.m` | All 8 runs (ControlFlag 1-4, Euler and quaternion) match `reference/SIMauv1_baseline.mat`, every 100th sample, ocean-current noise seeded | A few minutes |
| `test_auv1_matches_remus100.m` | `auv1.m` with `auv1Params('remus100')` gives the same `xdot` as the original MSS `remus100.m`, within 1e-12 | Seconds |
| `test_auv1_actuator_signs.m` | AUV1 fins and thrusters push the right way through `fin_mixer.m` and `deflection_to_pwm.m` (+yaw command turns to starboard, +pitch noses up, PWM > 1500 is forward/starboard thrust) | Seconds |
| `test_auv1Visualiser.m` | The replay draws the logged data and its controls work | About a minute |

## Regression baseline

The baseline stores the path it was made with and the test runs that path,
so editing `pathParams.m` does not break it. It was last rewritten after
merging pass B (AUV1 PWM actuators). Before that, the merged code was checked
bit for bit against the unsplit `SIMauv1.m` of pass B.

After an intended change in behaviour, rewrite it with the current
`pathParams()`:

```matlab
test_SIMauv1_regression(true)
```

## reference/

- `remus100.m`: unmodified MSS REMUS 100 model. On the path only while
  `test_auv1_matches_remus100` runs, so it never shadows anything.
- `SIMauv1_baseline.mat`: regression baseline (`runs`, `step`, `path`).
