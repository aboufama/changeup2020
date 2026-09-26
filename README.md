# Change Up 2020–21 — VEX V5 robot code

VEXcode Pro V5 (C++) code for a VEX Robotics Competition robot from the 2020–21 *Change Up* season. The core of it is tracking-wheel odometry: two V5 rotation sensors and the inertial sensor keep a running estimate of where the robot is on the field, and the drive functions use the heading for field-oriented moves.

## Hardware

| Device | Port | Notes |
|---|---|---|
| `TR`, `TL`, `BR`, `BL` drive motors | 1–4 | 18:1 (green) cartridges; holonomic drive, diagonal wheel pairs driven together |
| `RIntake`, `LIntake` motors | 5, 6 | configured, not used in code yet |
| `yRot` rotation sensor | 14 | y (forward) tracking wheel, reversed |
| `xRot` rotation sensor | 15 | x (sideways) tracking wheel |
| `Inertial16` inertial sensor | 16 | heading |

## How it works

- **Startup** (`Init` in `src/Definitions.cpp`): stops the drive, calibrates the inertial sensor (blocks until done), zeroes both tracking wheels, and sets the tuning constants and the starting position (0, 0).
- **Background thread** (`BckGround` in `src/Util.cpp`, about every 20 ms): corrects the heading, runs one odometry step, and prints x, y and heading to the Brain screen.
- **Odometry** (`src/Odom.cpp`): takes the change in each tracking wheel and in heading since the last step, uses the arc-chord approximation to get the local displacement, rotates it into field coordinates by the heading, and adds it to `xPos` / `yPos`.
- **Heading correction** (`Correct` in `src/Util.cpp`): adds a drift correction proportional to how far the robot has turned in total (`inertError`, about 2.4 %).

## Drive functions (`src/MovementFuncs.cpp`)

- `TurnInPlace(deg, slowDown, minSpeed)` turns to an absolute heading. Speed is the remaining error divided by `slowDown`, plus `minSpeed`, and the error is wrapped to ±180° so the robot turns the short way round.
- `StrafeXY(x, y, speed, decel)` drives toward an (x, y) offset in tracking-wheel degrees, with the direction corrected for the robot's heading. It stops once both tracking wheels are within 5° of the target.
- `SetVelocityTheta(theta, speed, state)` sets the drive moving at an angle: robot-relative when `state` is `"local"`, field-relative otherwise. It does not stop the drive on its own.

## Building

Open `idk.v5code` in VEXcode Pro V5, build, and download to the V5 Brain. `main()` currently initializes the sensors and starts the odometry thread; the `StrafeXY` test call is commented out.

## Notes

- Positions are in tracking-wheel degrees, not inches.
- `Print()` in `src/Util.cpp` takes a `char`, so the values it shows on the Brain screen are truncated to a single character.
- `yTurnOffset`, `xTurnOffset` and the `decel` argument of `StrafeXY` are set but not used yet.
