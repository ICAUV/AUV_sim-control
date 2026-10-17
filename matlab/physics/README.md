# physics

The vehicle side of the loop: what the controllers measure, the ocean current,
and the state update. The dynamics themselves are in `../auv1.m` (see
`matlab/README.md`).

## measurements.m

`y = measurements(x)` returns `u w q r xn yn zn theta psi` from the state.
For quaternion states, `theta` and `psi` come from `q2euler`. The measurements
are exact (no noise, no estimator).

## oceanCurrent.m (flag 4 only)

`[Vc, betaVc] = oceanCurrent(t, Vc, betaVc, h)`

- Speed 0.5 m/s, rising towards 0.65 m/s after 800 s
- Direction 150 deg, turning towards 160 deg after 500 s
- Both changes through first-order filters, plus small `randn` noise (the
  regression test seeds it)

Flags 1-3 keep the constant current set in `SIMauv1.m`. Theory Ch 10.3.

## physicsStep.m

`x = physicsStep(x, ui, Vc, betaVc, wc, model, h)` moves the state one sample.

- 12 states (Euler angles): RK4 on `auv1.m`
- 13 states (quaternion): forward Euler on the velocities, NED positions from
  the new velocities, exact quaternion step `expm(Tquat([p q r])*h) * quat`,
  then normalised. Theory Ch 2.2.
