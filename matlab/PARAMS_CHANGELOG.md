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
