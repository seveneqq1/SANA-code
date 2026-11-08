# Minimal TeleOp state machines

Create one variation using your existing hardware controller instances:

```java
SnapStateMachine machine = new SnapStateMachine(shooter, intake, turret, null);
// Or: TrackingStateMachine machine = new TrackingStateMachine(shooter, intake, turret, null);
```

A null LUT uses `shooterController_var2.baseVelocity`. To use measured shot data,
pass `new ShotTable(distancesInches, speedsTPS)` with at least two matching arrays,
strictly increasing distances, and positive TPS. Interpolation is linear and
out-of-range distances use the closest endpoint speed.

```java
// Snap only: one step per press, safe to call every loop.
machine.handleRotationButtons(gamepad2.right_bumper, gamepad2.left_bumper);
if (gamepad2.a) machine.spinUp();
if (gamepad2.b) machine.cancelAll();
if (gamepad2.x) machine.startIntaking();
if (gamepad2.y) machine.reverseIntake();
machine.requestFeed(gamepad2.right_trigger > 0.5);
machine.update(x, y, heading);
machine.debug(telemetry);
telemetry.update();
```

Update localization first. Positions are inches from the bottom-left field corner.
Heading is radians from +X, increasing counterclockwise. Quarter boundaries are
x=72 and y=72; tracking targets are (60,60), (84,60), (60,84), and (84,84).
Distance for shooter speed uses the same tile target, including in snap mode.
The robot pose is treated as the turret position.

Configure `turretForwardRadians` and `turretDirection` (+1 for increasing servo
command turning CCW, -1 otherwise). Existing mechanical turret limits remain.
Snap methods `rotateClockwise90()` and `rotateCounterclockwise90()` return false
at a physical limit. Call these directly once per press, or use the button handler.
`centerTurret()` returns to center; tracking mode also disables tracking until
`setTrackingEnabled(true)` is called.

Helpers include `setManualTPS(tps)`, `useAutomaticTPS()`, `setSpeedTrim(tps)`,
`stopIntaking()`, `reverseIntake()`, `stopShooting()`, and `cancelAll()`.
Tracking also supports `adjustAimTrim(radians)`.

Feeding requires a positive target and `shooter.isAtTarget()`. There is no pose
validation, aiming gate, turret settling timer, speed stability timer, or quarter
hysteresis. The driver decides when the turret is aligned. Call `cancelAll()` on
OpMode stop. Only the helper calls shooter/turret update.


## Regional A - Tracking (2026-11-08)

`RegionalATrackingTeleOp` refreshes the full Pinpoint pose once per loop before
passing inches and radians into the tracking coordinator. It retains heading hold,
driver mechanism priorities, fixed shooter TPS, right-side drive inversion, and
stop cleanup. G2 X resumes tracking, Y centers and pauses tracking, and LB/RB
apply +5/-5 degree aim trim once per press (both together do nothing).

Tracking still selects the existing quarter targets (60/84 inches on each axis).
Pinpoint must already be calibrated and positioned in that field coordinate frame;
this OpMode does not invent pod offsets. G1 D-pad Right sets the pose to the
configured START_X/Y/HEADING (inches/radians), and cancels heading hold. Verify those targets
match the intended goal before shooting. Module-to-module CAD spacing is not a
signed pod offset from the robot center.

Transfer power is reduced only in FEEDING: 0.8 times the Dashboard setting
`intakeController.TRANSFER_SHOOTING_SPEED_SCALE` (default 0.5) gives 0.4.
FULL_POWER stays 1.0 and BALLS_OUT stays -0.8. Spin-up without shooter readiness
stops transfer; stop/cancel sets it to zero.

Fresh Java compilation and a disposable JVM harness with simulated hardware passed
for transfer transitions, readiness, position/heading aiming, center/resume, trim,
and cancel. These checks do not establish physical odometry or aiming calibration.
