# Start here when debugging

The turret is currently locked at servo position **0.5** on both `turret` and `turret1`.
The controller commands this at initialization and on every update, including while aiming,
holding, or using manual controls. The tracking calculations below still run for distance and
aim telemetry, but their angles do not move the servos. Servo command telemetry always shows 0.5.

The competition control path is three methods, read in this order:

1. `compOpModes/RegionalTeleOp.loop()` reads buttons, maintains the intake toggle, drives, and selects the action.
2. `controllers/StateController.update()` updates turret, flywheels, intake/transfer, then stopper in one pass.
3. `controllers/TurretController.update()` shows the complete tracking calculation in four numbered steps.

Turret angles are measured from the backward-facing center. Both servos at 0.5 means backward.
The calculation is `field bearing - robot heading - 180 + offset + trim`, then mechanical limits,
then `0.5 + angle / 495` for both servos (using the configurable center and travel values).

Dashboard telemetry now shows target X/Y, field bearing, robot heading, requested turret angle,
limited target angle, and the servo command. Check these in order:
- Wrong target X/Y: inspect alliance and target coordinates in `RobotConfig`.
- Wrong robot position/heading: inspect odometry and the start/reset pose.
- Correct requested angle but wrong physical direction: inspect servo direction and mechanical calibration.
- Correct servo command but delayed physical motion: the estimate is not servo feedback.

`TurretAim` contains the field geometry and mechanical-angle helpers. The moving-shot flight-time
setting is optional; setting it to zero removes velocity lead while debugging static aiming.

# BIOBUZZ robot controls

Driver Station entries:

- **Blue** and **Red** share `RegionalTeleOp`.
- **BIOBUZZ Blue Preload Auto** and **BIOBUZZ Red Preload Auto** share `RegionalAuto`.
  The displayed names are retained; `routine()` contains the current multi-path Ivy autonomous chain.
- **Timed Forward Auto** is a fallback for either alliance when odometry is unavailable.
  It drives robot-forward at 30% power for 1 second, then brakes. Change `DRIVE_SECONDS`
  and `DRIVE_POWER` in `autonomous/TimedForwardAuto.java` or its FTC Dashboard config before START.
  Power must be 0..1 and duration must be nonnegative; zero duration skips driving.
  It uses the drive motor names/directions from `pedro/Constants.java` and runs without encoders,
  Pinpoint, an IMU, or a path follower. STOP ends the drive early. Travel distance and drift depend
  on battery charge and the surface, so tune the duration on the robot.

## Project layout (Quickstart 2 style)

```text
teamcode/
├── bobot/
│   ├── controllers/       StateController + ShootingSystem + Drive/Turret/FlowerTakerController
│   ├── compOpModes/       RedTeleOp, BlueTeleOp, shared RegionalTeleOp
│   ├── autonomous/        RedAuto, BlueAuto, shared RegionalAuto
│   │   ├── commands/      RobotCommands: Ivy mechanism commands
│   │   └── paths/         AutoPaths: Pedro 3 paths
│   ├── TestOpModes/       Beka Tuning, Simple, Base, servo/debug OpModes
│   ├── states/            Alliance, PoseStorage (start/reset/handoff poses)
│   ├── config/            RobotConfig, ControlBindings (Dashboard names preserved)
│   └── utilities/         Target geometry, angle limits, drive voltage compensation
└── pedro/                 Pedro 3 constants and tuners
```

Start with `compOpModes/RegionalTeleOp.loop()`. Buttons call `stateController.intake()`,
`shoot()`, `eject()`, or `idle()`, followed by one `stateController.update(pose)`.
`controllers/StateController.update()` coordinates mechanisms in order; each mechanism controller
owns its hardware. Edit `controllers/ShootingSystem.java` for shooting power/delay/feed calibration, `config/RobotConfig.java` for other calibration, and `config/ControlBindings.java` for buttons.

For aiming, `TurretController.setTargetPosition(angle)` uses Quickstart's convention:
0 degrees points robot-forward, 180 degrees points backward. Both servos get the same position.
Tracking still selects your alliance's LOWER/UPPER target using `TARGET_SPLIT_Y` and hysteresis,
then computes `atan2(targetY - pivotY, targetX - pivotX) - robotHeading`.
The current center, 495-degree travel, limits, trim, pivot offset, and moving-shot compensation remain configured.
Manual tuning angles remain relative to the backward center (manual 0 = both servos at 0.5).

Autonomous uses **Pedro Pathing 3.0.1 and Ivy 1.1.1**, including `sequential(...)`, `parallel(...)`,
`follow(follower, path)`, `waitUntilReady(...)`, and timed `shoot(...)` commands.
Edit `autonomous/RegionalAuto.routine()` for the chain and `autonomous/paths/AutoPaths.java` for paths.
The existing path sequence and commands are preserved; Quickstart 2's older Pedro API is not used.

## Hardware configuration

| Device | Configuration name | Type |
| --- | --- | --- |
| Front left wheel | `leftFront` | DC motor, no encoder |
| Rear left wheel | `leftBack` | DC motor, no encoder |
| Front right wheel | `rightFront` | DC motor, no encoder |
| Rear right wheel | `rightBack` | DC motor, no encoder |
| Odometry | `pinpoint` | goBILDA Pinpoint, 4-bar pods |
| Intake | `intake` | DC motor, no encoder |
| Transfer | `transfer` | DC motor, no encoder |
| Shooter motor 1 | `shooter1` | DC motor with encoder |
| Shooter motor 2 | `shooter2` | DC motor with encoder |
| Turret (two servos) | `turret`, `turret1` | Servo (330°), 3:2 gearing to the turret |

Change drive names, directions, and Pinpoint pod settings in `pedro/Constants.java`.
Existing drivetrain directions and Pinpoint offsets are preserved. Validate them against your stateController.
Change mechanism names/directions in `bobot/config/RobotConfig.java`. Restart the OpMode after changing names/directions.

The turret turns 1.5 degrees per servo degree, so its travel is 330 × 1.5 = 495 degrees
(±247.5 from the backward-facing center). Servos report no position, so "aimed" uses an estimate driven by
`TURRET_SPEED_DEG_PER_SEC`; feeding waits until that estimate reaches the target.

Teleop drive uses voltage compensation. ShootingSystem sends direct power to the shooter,
intake, and transfer: a setting of 0.75 always commands 75%. Actual speed and shot distance
still depend on battery voltage and mechanical load. Autonomous driving uses Pedro.

## Shooting setup

All shooting outputs are owned by `controllers/ShootingSystem.java`. In FTC Dashboard open
**Configuration -> ShootingSystem**:

| Setting | Default | Meaning |
| --- | --- | --- |
| `SHOOTER_POWER` | 0.75 | Direct power to both flywheel motors, 0..1 |
| `SPINUP_SECONDS` | 1.0 | Delay from starting the flywheels to feeding; 0 feeds immediately |
| `INTAKE_FEED_POWER` | 0.8 | Intake power while feeding |
| `TRANSFER_FEED_POWER` | 1.0 | Transfer power while feeding |
| `STOPPER_OPEN_POSITION` | 0.5 | Gate position while feeding |
| `STOPPER_CLOSED_POSITION` | 0.0 | Gate position in every other state |

Settings apply live. Hold Y: constant flywheel power, wait once, then feed and open the stopper
for the rest of the hold. Release, eject, or stop cancels the shot and resets the timer.
The flywheel motors coast down after power is removed. Zero/invalid shooter power blocks feeding.
There is no PIDF, startup boost, encoder readiness gate, voltage scaling, distance LUT, or aim gate.
The spin-up timer being ready does not mean wheel speed was measured.
Hardware names/directions and collection/ejection powers remain in RobotConfig.

## Default controls

All actions are in `bobot/config/ControlBindings.java`. Each action has a driver number (1/2) and button enum.
Bindings can also be edited in FTC Dashboard. Buttons marked hold release their action when released.
On a PS controller: A = Cross, B = Circle, X = Square, Y = Triangle.

| Driver | Control | Action |
| --- | --- | --- |
| 1 | Left stick | Translate |
| 1 | Right stick X | Rotate |
| 1 | Right trigger, hold | Slow to 30% **and bypass pillar avoidance** |
| 1 | Triangle (Y), hold | Spin up shooter, wait Dashboard delay, then feed; release stops shooter and leaves collection on |
| 1 | X (Square), press | Toggle intake and transfer on/off |
| 1 | B (Circle), hold | Reverse intake and transfer to spit balls out |
| 1 | A (Cross), hold | Stop everything: drive, intake, shooter; turret back to its backward-facing center, trim cleared |
| 1 | Left / right bumper, press | Nudge turret 2° left / right (kept as trim for later shots) |
| 1 | D-pad right, press | Reset pose: red = bottom-left corner, blue = top-right corner |

Mechanism priority: stop, eject, shoot, intake, idle.
Red/Blue teleop continuously aims the turret while driving, idling, intaking, and ejecting.
Hold A to return it to its backward-facing center and clear trim; releasing A resumes tracking. Triangle controls shooting/feed.
Pressing Triangle enables the intake toggle in Red/Blue, Beka Tuning, and Simple teleop.
In Red/Blue, Beka Tuning, and Simple, the shooter runs only while Triangle is held. Intake/transfer wait
for `ShootingSystem.SPINUP_SECONDS` (default 1.0 second), and the stopper stays closed until feeding.
Edit the delay live in FTC Dashboard under Configuration -> ShootingSystem. Set 0 for immediate feeding.
After the delay, intake/transfer run at feed power and the stopper opens for the rest of the hold,
regardless of encoder speed or aim. Release, eject, or stop resets the timer for the next hold.
Intake/transfer return to collection power after release until X or A turns them off.
Beka Tuning also starts in continuous auto aim; D-pad Down selects the manual angle override.
Driving is always robot-centric: stick forward moves the robot forward, and stick left moves it left,
regardless of its field heading. Odometry supplies the field pose for turret aiming and support avoidance.

## Odometry and field geometry

Use one coordinate frame in every OpMode: origin at the **bottom-left corner of your attached image**,
+X right, +Y up, distances in **inches**, headings in **radians**, 0 heading facing +X, positive counterclockwise.
This is our chosen Pedro frame; do not paste centered FTC/AprilTag coordinates without converting them.

Pedro 3.0.1 calls used here are `localizer.update()`, `localizer.pose()`, `pose.x()`, `pose.y()`,
`pose.heading()`, `localizer.setPose(...)`, `drivetrain.drive(new DrivePowers(...), true)`;
autonomous uses `follower.update()`, `follower.follow(path)`, `follower.pose()`, and `follower.setPose(...)`.
Teleop directly uses Pedro's Mecanum mixer and Pinpoint localizer; a second Follower must not command the same wheels.
[Pedro TeleOp documentation](https://pedropathing.com/docs/pathing/guide/teleop-usage)

Geometry sources, checked October 14, 2026:

- [FIRST BIOBUZZ TU04 arena manual](https://ftc-resources.firstinspires.org/ftc/game/manual-09),
  Figures 9-8 to 9-10: nominal frame 49.46 x 38.95 inches, pivot height 43.95 inches,
  hive spacing 25.5 inches, cell inner gap 18.84 inches, cell depth 12.04 inches, and 30-degree tilt.
- [FIRST event setup guide](https://ftc-resources.firstinspires.org/ftc/archive/2027/field/eventfieldguide),
  Section 9: center the frame on the field.
- [Official field resources and CAD](https://ftc-resources.firstinspires.org/ftc/archive/2027/field).

`RobotConfig` uses nominal outer frame bounds centered on (72,72), not exact bolt-center coordinates.
The image orientation places blue hive at Y=84.75 and red at Y=59.25.
Cell target X coordinates are a nominal projection of the cell centers at 30-degree tilt:
72 +/- ((18.84 + 12.04)/2)*cos(30 degrees), approximately 58.63 and 85.37.
Tune these aim points and turret offset for your actual shots: they are derived approximations, not surveyed opening coordinates.
Both alliances automatically choose their nearest left/right cell, with 2 inches of switching hysteresis to reduce chatter.

**Hive cells tip during play.** Odometry alone cannot determine which cell currently faces upward.
The requested nearest-half behavior selects by position; it does not sense hive tilt or compute shot elevation.
The hood servo is not driven (fixed hood angle). Flywheel power is set manually in ShootingSystem;
field geometry does not change its power or feeding timer.

Set the physical starting pose in `RobotConfig` before INIT. The defaults are examples, not automatically measured positions.
Autonomous saves its latest pose; teleop consumes a matching-alliance handoff once if it is under 120 seconds old.
An app restart clears this handoff. `TELEOP_USE_AUTO_POSE=false` always uses the configured start pose.
Do not move the robot between auto and teleop without updating its known field pose.

## Pillar avoidance

At normal speed, stick translation is converted to the field frame and redirected tangentially around supports.
Approach power is constrained by lookahead distance and nominal stopping deceleration. Reverse/outward escape remains available.
The model covers both foot bars and the four inclined leg segments up to `ROBOT_HEIGHT_IN`.
Each segment is expanded by `ROBOT_RADIUS_IN + PILLAR_RADIUS_IN + AVOIDANCE_MARGIN_IN`.
Robot radius should be the half-diagonal of the largest active robot footprint, including protruding intake/turret parts.
This circular footprint accounts for rotation. No translation input produces no automatic translation.

Holding the slow binding bypasses avoidance completely while scaling all stick commands by `DRIVE_SLOW_SCALE`.
Telemetry shows the bypass clearly. Releasing slow re-enables assistance.
Avoidance depends on calibrated odometry, geometry, footprint, speed, and braking values; wheel slip or bad localization can still cause contact.
It models fixed hive supports, not other robots, loose balls, flowers, or walls.
It applies to manual teleop driving. Autonomous paths must be planned clear of the expanded supports.

## Dashboard installation and connection

FTC Dashboard **0.6.0** is installed in the shared `build.dependencies.gradle`, using
`https://maven.brott.dev/`. Pedro Pathing **3.0.1** uses the separate Dairy repository.
This follows the [official Dashboard installation tutorial](https://acmerobotics.github.io/ftc-dashboard/gettingstarted.html)
and [Pedro 3 installation guide](https://pedropathing.com/docs/pathing/installation).
The SDK handles Dashboard startup; no manual edits to `FtcRobotControllerActivity` are required.
Teleop and autonomous telemetry is sent to both the Driver Station and Dashboard.
Pose coordinates, shooter target/individual motor velocities, and turret error are separate numeric graph channels.

The project retains AGP 8.13.2 and selects Lint 9.4.1 in `gradle.properties` using Android's
[documented lint version override](https://developer.android.com/develop/ui/compose/tooling/lint).
This resolves the Kotlin 2.4 metadata errors from AutoTune's transitive Sloth dependency during release lint.
Lint checks remain enabled; Gradle labels the override experimental.
Gradle uses a 2 GB heap and at most two workers to avoid APK packaging running out of heap on a clean build.

After installing the APK, connect your computer to the robot's Wi-Fi and open:

- Control Hub: <http://192.168.43.1:8080/dash>
- Phone Robot Controller: <http://192.168.49.1:8080/dash>

In Dashboard's configuration view, select `RobotConfig` for gains/powers and `ControlBindings` for buttons.
If imports appear unresolved in Android Studio, use **Sync Project with Gradle Files** and let indexing finish.
Red filenames in the project tree can instead mean **untracked Git files**; that color does not imply a Java error.

## Tuning

All live gains/powers/geometry are in Dashboard's `RobotConfig`; button mappings are in `ControlBindings`.
Dashboard edits are in-memory: copy final values into source to preserve them after restart.

1. Check wheel/mechanism direction at low power with the robot lifted. Confirm both shooter encoders report
   **positive** velocity when shooting; motor 2 is reversed by default. A negative reading is not treated as ready.
2. Verify Pinpoint configuration: correct pod type, encoder direction, offsets, inches, and measured heading.
   Move along +X/+Y and rotate CCW; telemetry should agree before enabling field-based aiming or avoidance.
3. Turret: hold A so the turret goes to `TURRET_CENTER_POSITION`; adjust that value until it faces exactly
   backward. Nudge left with the left bumper: if it turns right, set `TURRET_REVERSED=true`.
   Set `TURRET_MIN_DEG`/`TURRET_MAX_DEG` inside any cable or mechanical limit.
4. Auto aim runs continuously in teleop; tune `TURRET_OFFSET_DEG` and the pivot offsets
   (robot-forward/robot-left distances). Time a large swing and set `TURRET_SPEED_DEG_PER_SEC` slightly
   below the real speed so the turret estimate reflects actual travel. Bumper trim is runtime only; move a good trim into the offset.
   Out-of-range targets are reported unreachable; aim calculations do not gate shooting.
5. In Dashboard -> ShootingSystem, tune `SHOOTER_POWER` and `SPINUP_SECONDS` first, then feed powers
   and gate positions. Use the same battery condition and shot location when comparing settings.
6. Set robot footprint/height and avoidance margin, and measure top speed/deceleration.
   Test the avoidance map at low commanded speed before driving near a real support.
7. Corner reset: push the robot into the red bottom-left corner, measure its center and heading, and set
   `RED_CORNER_X/Y/HEADING_DEG`. Blue uses the same pose rotated 180 degrees (top-right corner).

Autonomous uses the same power and timer as teleop. `waitUntilReady(...)` starts the flywheels
and waits for the timer. Moving from SPINNING_UP to SHOOTING preserves elapsed spin-up time.
`shoot(..., feedMs)` counts actual feeding time, then stops all shooting motors and closes the gate;
it also stops them on timeout. `AUTO_READY_TIMEOUT_MS` bounds the wait in autonomous commands.

## Using the robot

`StateController` coordinates ShootingSystem and the turret. Pick a mode, then call `update(pose)` once per loop;
it is the only place subsystems are updated, so do not update `stateController.shooting`/`turret` yourself.

```java
StateController stateController = new StateController(hardwareMap, Alliance.RED);
stateController.setMode(StateController.Mode.INTAKING);   // IDLE, INTAKING, EJECTING, SPINNING_UP, SHOOTING
stateController.update(currentPose);

stateController.turret.nudge(2);                 // degrees, positive = left
boolean ready = stateController.isReadyToShoot();
stateController.stop();                          // zeros every motor immediately; turret holds
stateController.reset();                         // stop() + turret centered backward, trim cleared
```

Both flywheel motors receive `ShootingSystem.SHOOTER_POWER` whenever SPINNING_UP or SHOOTING is active.

## Writing autonomous (Ivy)

Autonomous uses the [Ivy command library](https://pedropathing.com/docs/ivy/command-compositions).
Edit `routine()` in `bobot/autonomous/RegionalAuto.java`; Blue and Red share it.

| Command | What it does |
| --- | --- |
| `sequential(a, b, c)` | Run one after another |
| `parallel(a, b)` | Run together, finish when all finish |
| `race(a, b)` / `a.raceWith(b)` | Run together, finish when the first finishes |
| `instant(() -> ...)` | Run code once, finish immediately |
| `waitMs(400)` | Wait 400 ms |
| `waitUntil(() -> ...)` | Wait for a condition |
| `follow(follower, path)` | Drive a Pedro path (from `PedroCommands`) |
| `intake(robot)`, `spinUp(robot)`, `idle(robot)` | Switch robot mode (instant; stays until changed) |
| `waitUntilReady(robot)` | Spin up at fixed power; finish after the delay or `AUTO_READY_TIMEOUT_MS` |
| `shoot(robot, ms)` | Feed for `ms` of actual feeding, then stop and close gate; times out after `AUTO_READY_TIMEOUT_MS + ms` |

Example with paths (choose poses for your strategy, clear of the expanded supports):

```java
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.bobot.autonomous.commands.RobotCommands.*;

@Override protected Command routine() {
    Pose start = PoseStorage.startPose(alliance());
    Pose shootPose = new Pose(96, 96, Math.toRadians(225));
    Pose pickup = new Pose(120, 84, Math.toRadians(0));
    return sequential(
            parallel(follow(follower, line(start, shootPose).constant(shootPose.heading())), spinUp(robot)),
            waitUntilReady(robot),
            shoot(robot, RobotConfig.AUTO_FEED_MS),
            parallel(follow(follower, line(shootPose, pickup).constant(pickup.heading())), intake(robot)),
            waitMs(300),
            follow(follower, line(pickup, shootPose).constant(shootPose.heading())),
            waitUntilReady(robot),
            shoot(robot, RobotConfig.AUTO_FEED_MS)
    );
}
```

Each loop runs `follower.update()`, then `Scheduler.execute()`, then `stateController.update(pose)`; keep that order in new OpModes.
A timed-out step ends and the routine moves on; `shoot` never feeds unless speed and aim are ready.
Stopping the OpMode resets the scheduler and zeros every motor.

## Verification

```sh
./gradlew :TeamCode:testDebugUnitTest :TeamCode:assembleDebug
```

Tests exercise controller coordination through simulated encoder motors, both alliance/quadrant mappings,
angle limits, fixed shooting power and timer resets, Ivy autonomous commands and timeouts, pose handoff, and randomized
support-clearance trajectories. They verify software behavior; real motor/field calibration is still required.
