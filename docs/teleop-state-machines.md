# Minimal TeleOp state machines

Create one variation using your existing hardware controller instances:

```java
SnapStateMachine machine = new SnapStateMachine(shooter, intake, turret, null);
// Or: TrackingStateMachine machine = new TrackingStateMachine(shooter, intake, turret, null);
```

A null LUT uses `shooterController.baseVelocity`. To use measured shot data,
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
