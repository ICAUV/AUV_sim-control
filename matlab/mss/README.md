# Vendored MSS toolbox files

The `.m` files in this folder are unmodified copies from Fossen's Marine Systems
Simulator (MSS) toolbox: https://github.com/cybergalactic/MSS (MIT licence, see
`LICENSE`). They were copied on 2026-10-16.

Do not edit these files in place. To refresh them, re-copy them from the MSS
repository.

## Where they are used

| Function | Called from | Does |
|---|---|---|
| `ALOSpsi`, `ALOS3D` | `navigation/navigation.m` | 2-D and 3-D ALOS path following |
| `refModel`, `LOSobserver` | `navigation/updateReferences.m` | Reference model; LOS angle and rate filter |
| `makeLowPass` | `SIMauv1.m` | Low-pass filter for the depth command |
| `integralSMCheading` | `control/trackingController.m` | Integral SMC heading autopilot |
| `sat` | `control/`, `navigation/updateReferences.m`, `auv1.m` | Saturation |
| `ssa` | `control/trackingController.m`, `SIMauv1.m`, `auv1Visualiser.m` | Smallest signed angle |
| `rk4`, `Rquat`, `Tquat` | `physics/physicsStep.m` | RK4 step; quaternion kinematics |
| `q2euler`, `euler2q` | `physics/measurements.m`, `SIMauv1.m` | Quaternion to Euler angles and back |
| `spheroid`, `imlay61`, `Dmtrx`, `forceLiftDrag`, `crossFlowDrag`, `gRvect`, `eulerang`, `quatern` | `auv1.m` | Mass, added mass, damping, hull forces, restoring forces, kinematics |
| `gravity`, `mssConstants` | `auv1Params.m` | Gravity at a latitude; water density |
| `rangeCheck`, `isoctave` | `SIMauv1.m` | Input checks; Octave detection |
| `Rzyx` | `auv1Visualiser.m` | Rotation matrix for the replay |

The rest (`Hmtrx`, `Smtrx`, `m2c`, `Tzyx`, `Hoerner`, `coeffLiftDrag`,
`cylinderDrag`) are only called by the functions above.
