# data/thrusters

Thrust tables for the AUV1 thrusters, read by `auv1Params.m`:

| File | Thruster | Used as |
|---|---|---|
| `T500_14v8.csv` | Blue Robotics T500 | Stern thruster |
| `T200_14v8.csv` | Blue Robotics T200 | Bow tunnel thruster |

Each table is force (N) against PWM (us) at 14.8 V. `make_thrust_tables.m`
builds them from the manufacturer spreadsheets in `raw/`: it takes the 14 V and
16 V sheets, converts kgf to N and interpolates linearly to 14.8 V. Positive
force is forward thrust (PWM above 1500).

The data is static (bollard) thrust, with no loss at forward speed.
