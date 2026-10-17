# AUV_sim-control

MATLAB simulator for the AUV1 underwater vehicle: a 6-DOF model with depth and
heading autopilots and ALOS path following. It is built on Fossen's MSS toolbox
(the REMUS 100 example), with the AUV1 hull, mass and fins, and its actuators:
four PWM fins, a T500 stern thruster and a T200 bow thruster.

## Running it

```matlab
run matlab/startup_auv1.m    % Adds the code folders to the path
SIMauv1                      % Pick the control method and kinematics in a dialog
simOut = SIMauv1(4, 2);      % 3-D ALOS with unit quaternions, keep the results
```

Each run plots the results and opens a 3-D replay (`auv1Visualiser`).

## Folders

| Folder | Contents |
|---|---|
| [matlab/](matlab/) | Simulator entry point, vehicle model, parameters, path, replay |
| [matlab/navigation/](matlab/navigation/) | Commands from the path, and the desired states they lead to |
| [matlab/control/](matlab/control/) | Depth loop, fin autopilots, actuator inputs (fin mixing, thrusters) |
| [matlab/physics/](matlab/physics/) | Measurements, ocean current, state update |
| [matlab/mss/](matlab/mss/) | Unmodified MSS toolbox functions used by the above |
| [matlab/data/thrusters/](matlab/data/thrusters/) | T500 and T200 thrust tables and the script that builds them |
| [matlab/tests/](matlab/tests/) | Regression and equivalence tests |
| [matlab/validation/](matlab/validation/) | Parameter validation script and its results |
| [theory/](theory/) | Fossen's lecture slides, and which chapter each part of the code uses |

## Control methods

`SIMauv1(ControlFlag, KinematicsFlag)`

| ControlFlag | Navigation | Depth | Heading |
|---|---|---|---|
| 1 | Step commands | Gradient depth loop + pitch PID | PID |
| 2 | Step commands | Gradient depth loop + pitch PID | Integral sliding mode (SMC) |
| 3 | 2-D ALOS (2.5-D path following) | Gradient depth loop + pitch PID | Integral SMC |
| 4 | 3-D ALOS | ALOS pitch command + pitch PID | Integral SMC |

KinematicsFlag 1 uses Euler angles (12 states, RK4), 2 uses unit quaternions
(13 states).

The waypoints and step commands are in `matlab/pathParams.m`; the vehicle,
control and guidance parameters in `matlab/auv1Params.m`.
