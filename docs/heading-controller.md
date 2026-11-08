# Heading controller

`bobot/controller/HeadingController.java` calculates chassis turn power from a
measured heading. It does not read sensors, reset Pinpoint, or command motors.
All headings are radians: zero along +X and positive counterclockwise.

Construct it with robot-tuned proportional and derivative gains:

```java
HeadingController heading = new HeadingController(kP, kD);
heading.setMaxPower(0.3); // Example test limit, not a calibrated setting.
heading.setTolerance(Math.toRadians(2)); // Example acceptable error.
heading.resetHeading(measuredHeadingRadians);
```

After updating your heading sensor, call once per control loop:

```java
double turnCCW = heading.calculate(measuredHeadingRadians, dtSeconds);
```

`dtSeconds` is the measured positive loop duration, not milliseconds. Output is
limited to the configured maximum power; positive means a CCW correction.
Confirm the drivetrain's rotation convention before passing this value to its
turn argument. Do not copy the old TeleOp's negation without verifying it.

- `setTargetHeading(radians)` selects an absolute heading by the shortest turn.
- `adjustTargetHeading(deltaRadians)` shifts that target.
- `snapTargetHeading(Direction.ANTICLOCKWISE)` adds π/2; CLOCKWISE subtracts π/2.
  Call once per button edge, not every loop while held. Steps are relative to the
  existing target, not rounding to a cardinal direction.
- `resetHeading(currentHeading)` captures the current heading and clears the
  derivative history. Use it after manual turning or a localization reset.
- `isAtTarget(currentHeading)` checks angular tolerance only, not settled speed.

Gains use power/radian and power/(radian/second); historical degree-based gains
are not interchangeable. Derivative uses measured angular velocity so a target
change does not create derivative kick. Consecutive samples must be less than
π apart for wrapped velocity to identify the rotation direction unambiguously.

`RegionalATeleOp` now reads heading from `GoBildaPinpointDriver` with hardware
name `pinpoint`. G1 D-pad Right resets its pose explicitly, as described below. It reads heading once
per loop and uses the measured loop duration for the controller. No Pedro
follower or X/Y localization is required for this heading-only mode.

## Button-enabled hold with manual override

`updateHold(currentHeading, dtSeconds, lockPressed, manualOverride)` manages an
initially disabled hold mode. Pass a one-loop button edge for `lockPressed` and
an explicit right-stick movement/deadband check for `manualOverride`.

- A new lock press captures the measured heading and enables PID hold.
- While holding, call `snapTargetHeading()` once on each left/right snap-button
  edge to move the target by ±90 degrees.
- Manual override disables hold immediately and wins over a simultaneous lock
  press. `updateHold()` returns zero PID power while disabled; the caller uses
  the driver's normal manual turn command instead.
- Centering the stick does not re-enable hold. A fresh lock press is required
  and captures the new measured heading, not the old snap target.
- `isHoldingHeading()` reports the mode; `disableHold()` cancels it on stop.

`calculate()` remains the raw PD math API and does not apply the hold-mode
latch. Use `updateHold()` for the requested button/manual override behavior.

## Regional A TeleOp controls

Driver Station name remains **Regional A - Snap** (the turret still uses snap
aiming). Controls are:

| Gamepad | Input | Action |
|---|---|---|
| 2 | D-pad Up, new press | Capture current measured chassis heading and enable hold |
| 1 | X / B, new press | While holding, shift heading target 90° CCW / CW |
| 1 | Right stick movement | Cancel heading hold; normal manual rotation uses right X |
| 1 | RT held above 0.2 | Intake |
| 1 | LT held above 0.2 | Eject; takes priority over shooting and intake |
| 1 | Y held | Spin shooter and feed at speed; takes priority over intake |
| 1 | LB held | Slow drive mode (moved from RT to avoid the intake conflict) |
| 1 | A held | Stop drive/mechanisms and cancel heading hold |
| 2 | LB / RB, new press | Existing turret CCW / CW 90° snap |
| 2 | X, new press | Existing turret center |

Intake and eject are hold-to-run rather than toggle controls. Both X/B together
are ignored for heading steps. A fresh lock press captures the actual heading
with priority over a simultaneous step. Right stick magnitude across both axes
above `RIGHT_STICK_DEADBAND` (initially 0.05) cancels hold, including when the lock
button is pressed simultaneously. A held lock button cannot re-arm it after
manual override or A-stop; release and press it again.

Dashboard exposes `HEADING_KP`, `HEADING_KD`, `HEADING_MAX_POWER`,
`HEADING_TOLERANCE_RADIANS` and `RIGHT_STICK_DEADBAND`. Initial P=1, D=0,
maximum power=0.35 and tolerance=2° are starting values for tuning, not measured
robot calibration. SolversLib's nominal positive turn direction is clockwise,
so the TeleOp negates the controller's CCW output. Confirm physical correction
direction and tune on the robot. The manual drive's confirmed right-side motor
inversion is preserved.

## Standalone regression checks

The test has a `main()` entry point and does not need JUnit. Run from the
repository root; compiled files go into a temporary directory:

```sh
rtk proxy python3 - <<'PY'
import subprocess
import tempfile
package = "org/firstinspires/ftc/teamcode/bobot/controller/"
with tempfile.TemporaryDirectory() as output:
    subprocess.run(["rtk", "proxy", "javac", "-d", output,
                    "TeamCode/src/main/java/" + package + "HeadingController.java",
                    "TeamCode/src/test/java/" + package + "HeadingControllerTest.java"], check=True)
    subprocess.run(["rtk", "proxy", "java", "-cp", output,
                    "org.firstinspires.ftc.teamcode.bobot.controller.HeadingControllerTest"], check=True)
PY
```

These check angle wrapping, output signs/limits, snaps, tolerance, derivative
behavior, reset, invalid inputs and an idealized rotation model. They do not
validate real drivetrain dynamics, sensor measurements or gain tuning.

## Pinpoint position reset (2026-11-08)

Both Regional A - Snap and Regional A - Tracking use a new G1 D-pad Right press
to write `START_X`, `START_Y`, `START_HEADING` directly to Pinpoint. Defaults are
0 inches, 0 inches, 0 radians; configure these Dashboard values for the robot's
known physical reset location. Heading is CCW from +X. This sets field pose,
without recalibrating the IMU or changing measured pod offsets.

The handler cancels heading hold and runs before the current sensor update and
any aiming. Holding the button does not repeatedly reset. G1 A suppresses reset.
Telemetry increments `Pose resets` when the handler runs. The old Pedro follower
is not used. The direct SDK API and units were checked against installed SDK
12.0.0 sources and both TeleOps compile; physical reset behavior needs a robot
check. The Snap mode continues to read only heading for its normal operation.
