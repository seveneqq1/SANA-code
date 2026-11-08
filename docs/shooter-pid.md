# Shooter velocity PID

All shooter-running TeleOps use `shooterController_var2`: Regional A - Snap,
Regional A - Tracking, test dont use, Blue (through the legacy class name), and
shoot seq 0.7 +intake+trans. Servo-only tests and the empty Red sketch have no
flywheel behavior to convert.

Shooter2 is the only encoder-equipped flywheel (user correction, 2026-11-08).
One PID reads the magnitude of shooter2's encoder velocity and commands equal
power magnitudes to both motors. Shooter1 gets positive power and shooter2 gets
negative power by default. Readiness and displayed TPS use shooter2 alone; an
unmeasured shooter1 speed is never averaged in or required for feeding.
Telemetry labels shooter1 as having no encoder, shows the measured shooter2 TPS,
shared power, error and readiness. Shooter1 speed is not independently verified.

Dashboard settings are under `shooterController_var2`: `baseVelocity`, `kP`, `kI`,
`kD`, `kV`, and `velocityTolerance`. P starts at 0.0003 power per TPS error; I starts at 0.0002 power per (TPS * second), and D starts at zero. The former
0.008 gain caused oscillation in an illustrative flywheel simulation and was
reduced; the new value is a tuning start, not hardware calibration. kV starts at 0.67/2400 power per TPS, an
estimate from the former fixed power and target, not measured calibration. These
values require hardware tuning. Confirm encoder TPS and motor direction, tune
feedforward and P, then adjust I/D while watching speed recovery.
The output is bounded to 0..1 before applying motor direction; integral
anti-windup and reset on target changes prevent retained correction.

The existing always-spinning match behavior remains: cancel/stop feeding closes
the stopper; OpMode stop calls `forceStopFlywheels()` to zero both motors.
A zero velocity target also produces zero power. Transfer remains slowed only
while feeding shots; normal intake/eject presets are unchanged.

## Checks

Java compilation passed. The standalone math test covers P/I/D behavior, reset,
saturation recovery, zero target, invalid feedback and derivative setpoint kick:

```sh
rtk proxy javac -d /private/tmp/regional-pid-tests TeamCode/src/main/java/org/firstinspires/ftc/teamcode/bobot/controller/ShooterVelocityPID.java TeamCode/src/test/java/org/firstinspires/ftc/teamcode/bobot/controller/ShooterVelocityPIDTest.java
rtk proxy java -cp /private/tmp/regional-pid-tests org.firstinspires.ftc.teamcode.bobot.controller.ShooterVelocityPIDTest 0.0003
```

A disposable stub-hardware integration harness also checked the real controller's
readiness, per-wheel correction, command signs, zero target, force-stop and
legacy-class integration. No hardware tuning, deployment or shooting test was
performed.

## Single-encoder regression correction

The previous two-encoder implementation averaged shooter1's zero reading with
shooter2's velocity: shooter2 at 2400 appeared as 1200, and the readiness check
could never pass. The single-encoder correction restores shooter2-only feedback
for PID, telemetry and feeding, with the same command magnitude for both motors.
A failing-then-passing stub-hardware regression checked zero shooter1 feedback,
2400 measured TPS, feeding at target, shared motor power, blocking after a real
speed drop and stop handling. Java compilation passed. Hardware shooting still
needs verification.

The standalone simulated recovery test covers 10/20/50/100 ms loops and a speed
drop; 0.008 fails its settling check and 0.0003 passes. Its 3600-TPS illustrative
motor and 0.15-second response are test fixtures, not robot calibration.

## Persistent underspeed correction (2026-11-08)

User telemetry showed steady underspeed with output below saturation. With I=0,
P plus an underestimated feedforward can settle below target: the error supplies
the additional motor power, so that error never disappears. Integral correction
is now enabled by default at 0.0002 power per (TPS * second), preserving existing
anti-windup, reset behavior and the 120-TPS readiness tolerance. Telemetry shows
the active I gain. Shooter2 remains the only feedback source.

A simulated plant with deliberately insufficient feedforward settled at 2115 TPS
for a 2400 target with I=0. The same regression passes within 20 TPS using the new
I gain across 10/20/50/100 ms loop periods after 30 simulated seconds. This plant
is a test fixture, not a motor calibration. Hardware recovery and overshoot still
need checking. Run the test with `ShooterVelocityPIDTest 0.0003 0.0002` to include
both simulated recovery and persistent-offset cases. If Dashboard has retained
a runtime override, set `shooterController_var2.kI` to 0.0002 after loading the code.

## Reduced shot speed (2026-11-08)

User confirmed shooting works and requested less power. The shared default
`baseVelocity` is now 1000 TPS. Shooter2 feedback, PID gains, feedforward per TPS,
readiness tolerance and shooting-only transfer slowdown are unchanged. The
2400-TPS values in the regression descriptions above are historical test inputs.

User adjustment (2026-11-08): raised the shared default from 1000 to **1720 TPS**.
PID settings and other behavior remain unchanged.
