# TeleOp commands and scoring

NextFTC calls the Ivy scheduler once in each active loop. Competition OpModes
schedule explicit sensor, drive, scoring and telemetry phases. In Point Track,
the driver controls and scoring update share a command so the readiness check,
drive write and shooter update keep their original order. SolversLib handles
the Mecanum mix and gamepad edges.

`ShootingStateMachine` coordinates the shared shooter, intake, and turret.
Snap rotates the turret in 90-degree steps. Tracking aims at the inner target
of the current field quarter. Point Track holds the configured field target and
lets the driver revise it by aiming manually. Positions use inches and headings
use radians from +X, increasing counterclockwise.

The flywheels use the configured base TPS. Feeding starts only after the
shooter reports a positive target and readiness. The stopper stays open once
feeding starts, and the transfer waits for the stopper opening delay before
applying its feed power. Cancel closes the stopper and stops the intake.
OpMode shutdown also stops the drivetrain and both flywheels.

## Pedro Pathing 3

A path-running OpMode can compose `PedroDrive.follow(path)` with Ivy
`Groups.sequential` and `Groups.parallel`. Construct `PedroDrive` during INIT
with this robot's measured `ForesightConfig`, set the calibrated start pose,
call `update()` in `beforeCommands()`, and call `stop()` in `onStop()`.
A completed path keeps Pedro's final pose hold active until the next movement
or STOP. Put an indefinite `hold(pose)` at the end of a sequence or cancel it
explicitly.

The drive wrapper owns the Mecanum motors and Pinpoint for that OpMode.
The existing TeleOps own those devices directly and the Blue Auto placeholder
only publishes telemetry.
