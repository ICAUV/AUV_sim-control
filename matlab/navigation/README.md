# navigation

What the vehicle should follow, and the smooth desired states (references)
handed to the controllers. In Fossen's terms this is the guidance system
(theory Ch 12); there is no state estimator, the controllers see the true state.

## navigation.m

`[cmd, alos] = navigation(t, y, ControlFlag, path, guidance, h)`

| Flag | Output | How |
|---|---|---|
| 1-2 | `z_ref`, `psi_ref` | Step schedule `path.step`: entry `k = 1 + (number of step times passed)` |
| 3 | `psi_ref`, `z_ref` | `ALOSpsi` (2-D ALOS) on `path.wpt`. `z_ref` is the depth of the waypoint the current leg starts from |
| 4 | `psi_ref`, `theta_ref` | `ALOS3D` on `path.wpt` |

- Inputs: `y.xn, y.yn, y.zn` from `physics/measurements.m`; `guidance = P.guidance`
  (look-ahead `Delta_h`, `Delta_v`, adaptive gains `gamma_h`, `gamma_v`, estimate
  limit `M_theta`, waypoint switching radius `R_switch`).
- `alos = [y_e z_e alpha_c_hat beta_c_hat]`: cross-track and vertical-track
  errors, and the crab-angle estimates. Logged only; zeros for flags 1-2.
- ALOS steers along the line between waypoints with a look-ahead point, and
  adapts an estimate of the crab angle so ocean currents do not leave a
  steady offset. The next waypoint becomes active within `R_switch` of it.
- Theory: Ch 12.5 (2-D ALOS), Ch 12 3-D ALOS (Fossen and Aguiar 2024).

## updateReferences.m

`ref = updateReferences(ref, cmd, ControlFlag, ctrl, guidance, h)` moves the
desired states from t(i) to t(i+1).

| Flag | Updates | Using |
|---|---|---|
| 1-2 | `psi_d r_d a_d` | `refModel` on `psi_ref` (`wn_d_psi`, `zeta_d_psi`, rate limit `r_max`) |
| 3 | `psi_d r_d` | `LOSobserver` on `psi_ref` (gain `K_f`), `r_d` limited to `r_max` |
| 3 | `z_d zdot_d a_d` | `refModel` on `z_ref` (`wn_d_z`, `zeta_d_z`, `w_max`) |
| 4 | `theta_d q_d`, `psi_d r_d` | `LOSobserver` on `theta_ref` and `psi_ref` |

- `refModel`: third-order reference model, gives smooth angle, rate and
  acceleration. Theory Ch 12.1.1.
- `LOSobserver`: filters the ALOS angle and estimates its rate, so the
  autopilot gets `r_d` as well as `psi_d` (Fossen 2027, eqs. B.46-B.47).
- Flags 1-2 depth reference: the step command low-pass filtered in `SIMauv1.m`
  (`makeLowPass`, `wn_d_z`).
- The desired pitch for flags 1-3 comes from `control/depthController.m`.
- Flag 3 shares `a_d` between the depth reference model and the heading
  autopilot. This is kept from the original MSS code.
