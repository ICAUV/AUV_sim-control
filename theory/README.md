# theory

Lecture notes for T. I. Fossen, *Handbook of Marine Craft Hydrodynamics and
Motion Control*: `Slides/Ch1.pdf` to `Ch16.pdf`, and extra notes on motion
control in `Notes/Ch15_Motion_Control.pdf`.

Where each chapter is used in the code:

| Ch | Topic | Used in |
|---|---|---|
| 2 | Kinematics: Euler angles, unit quaternions (2.2), crab angles (2.5) | `auv1.m`, `physics/`, crab-angle plots in `SIMauv1.m` |
| 3 | Rigid-body kinetics | `spheroid.m` (MRB, CRB) |
| 4 | Hydrostatics, restoring forces for underwater vehicles (4.1) | `gRvect.m` in `auv1.m` |
| 6 | Maneuvering models: added mass, damping, cross-flow drag, currents | `imlay61.m`, `Dmtrx.m`, `crossFlowDrag.m` in `auv1.m` |
| 7 | Nomoto models for heading (7.2) | `K_yaw`, `T_yaw` of the heading autopilot |
| 8 | Models for underwater vehicles: 6-DOF AUV (8.1), spheroid hulls (8.4), REMUS 100 | `auv1.m`, `auv1Params.m` |
| 9 | Propellers and thrusters (9.1, 9.3), control surfaces | Thruster, fin, rudder and stern-plane forces in `auv1.m` |
| 10 | Ocean current forces (10.3) | Relative velocity in `auv1.m`, `physics/oceanCurrent.m` |
| 11 | GNC architecture (11.1), control allocation (11.2) | The split into `navigation/`, `control/`, `physics/`; fin mixing in `control/actuatorInputs.m` |
| 12 | Guidance: reference models (12.1.1), 2-D ALOS (12.5), 3-D ALOS | `refModel.m`, `makeLowPass.m`, `ALOSpsi.m`, `ALOS3D.m`, `LOSobserver.m` |
| 15 | Successive loop closure (15.2), PID pole placement (15.3) | `control/depthController.m`, pitch PID gains |
| 16 | Integral sliding mode heading autopilot (16.4.4) | `integralSMCheading.m` |

Chapters 1, 5, 13 and 14 (introduction, seakeeping, navigation filters, INS)
are not used. Note that the code's `navigation/` folder is what the book calls
guidance (Ch 12); there is no state estimator.

Not in the slides: the gradient-based depth control (Fossen and Coates 2026)
and the 3-D ALOS papers (Fossen and Aguiar 2024; Coates and Fossen 2026),
cited in `matlab/SIMauv1.m`.
