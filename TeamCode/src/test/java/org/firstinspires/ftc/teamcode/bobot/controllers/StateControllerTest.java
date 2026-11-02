package org.firstinspires.ftc.teamcode.bobot.controllers;

import org.firstinspires.ftc.teamcode.bobot.autonomous.commands.RobotCommands;

import org.firstinspires.ftc.teamcode.bobot.config.RobotConfig;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.firstinspires.ftc.teamcode.bobot.autonomous.commands.RobotCommands.idle;
import static org.firstinspires.ftc.teamcode.bobot.autonomous.commands.RobotCommands.intake;
import static org.firstinspires.ftc.teamcode.bobot.autonomous.commands.RobotCommands.shoot;
import static org.firstinspires.ftc.teamcode.bobot.autonomous.commands.RobotCommands.waitUntilReady;
import static org.junit.Assert.*;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.reflect.Proxy;
import java.util.Random;
import org.firstinspires.ftc.teamcode.bobot.autonomous.paths.AutoPaths;
import org.firstinspires.ftc.teamcode.bobot.controllers.DriveController;
import org.firstinspires.ftc.teamcode.bobot.controllers.PillarAvoidance;
import org.firstinspires.ftc.teamcode.bobot.controllers.TurretController;
import org.firstinspires.ftc.teamcode.bobot.states.Alliance;
import org.firstinspires.ftc.teamcode.bobot.states.PoseStorage;
import org.firstinspires.ftc.teamcode.bobot.utilities.TurretAim;
import org.firstinspires.ftc.teamcode.bobot.utilities.VoltageCompensator;
import org.junit.Before;
import org.junit.Test;

public class StateControllerTest {
    private static final double EPS = 1e-6;
    private FakeMotor intakeMotor, transferMotor, shooterOne, shooterTwo;
    private FakeServo turretServo, secondTurretServo, stopperServo;
    private double batteryVolts;
    private ShootingSystem shooting;
    private long now;
    private TurretController turret;
    private StateController stateController;

    private static class FakeMotor {
        double power, velocity;
        int position;
        final DcMotorEx motor = (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
                new Class<?>[] { DcMotorEx.class }, (proxy, method, args) -> {
                    switch (method.getName()) {
                        case "setPower": power = (double) args[0]; return null;
                        case "getPower": return power;
                        case "getVelocity": return velocity;
                        case "getCurrentPosition": return position;
                        default: return defaultValue(method.getReturnType());
                    }
                });
    }

    private static class FakeServo {
        double position = Double.NaN;
        int positionWrites;
        final Servo servo = (Servo) Proxy.newProxyInstance(Servo.class.getClassLoader(),
                new Class<?>[] { Servo.class }, (proxy, method, args) -> {
                    switch (method.getName()) {
                        case "setPosition": position = (double) args[0]; positionWrites++; return null;
                        case "getPosition": return position;
                        default: return defaultValue(method.getReturnType());
                    }
                });
    }

    private static Object defaultValue(Class<?> type) {
        if (type == boolean.class) return false;
        if (type == double.class) return 0.0;
        if (type == int.class) return 0;
        if (type == long.class) return 0L;
        return null;
    }

    @Before public void setup() {
        RobotConfig.AVOIDANCE_ENABLED = true;
        RobotConfig.AVOIDANCE_MAX_LOOKAHEAD_IN = 0;
        RobotConfig.TARGET_SPLIT_Y = 72; RobotConfig.TARGET_SWITCH_HYSTERESIS_IN = 2;
        RobotConfig.RED_LOWER_TARGET_X = 60; RobotConfig.RED_LOWER_TARGET_Y = 60;
        RobotConfig.RED_UPPER_TARGET_X = 60; RobotConfig.RED_UPPER_TARGET_Y = 84;
        RobotConfig.BLUE_LOWER_TARGET_X = 84; RobotConfig.BLUE_LOWER_TARGET_Y = 60;
        RobotConfig.BLUE_UPPER_TARGET_X = 84; RobotConfig.BLUE_UPPER_TARGET_Y = 84;
        RobotConfig.TURRET_SERVO_RANGE_DEG = 330; RobotConfig.TURRET_GEAR_RATIO = 1.5;
        RobotConfig.TURRET_CENTER_POSITION = 0.5;
        RobotConfig.TURRET_CENTER_HEADING_DEG = 180;
        RobotConfig.TURRET_MIN_DEG = -247.5; RobotConfig.TURRET_MAX_DEG = 247.5;
        RobotConfig.TURRET_SPEED_DEG_PER_SEC = 600;
        RobotConfig.TURRET_OFFSET_DEG = 0; RobotConfig.TURRET_PIVOT_FORWARD_IN = 0;
        RobotConfig.TURRET_PIVOT_LEFT_IN = 0;
        RobotConfig.SHOOT_FLIGHT_TIME_SECONDS = 0.35;
        RobotConfig.SHOOT_MAX_TRACKING_SPEED_IPS = 120;
        RobotConfig.AUTO_READY_TIMEOUT_MS = 4000;
        RobotConfig.VOLTAGE_COMPENSATION = true; RobotConfig.NOMINAL_VOLTAGE = 12;
        RobotConfig.VOLTAGE_MAX_SCALE = 1.5; RobotConfig.VOLTAGE_READ_INTERVAL_MS = 100;
        RobotConfig.VOLTAGE_FILTER_ALPHA = 0.3; RobotConfig.VOLTAGE_MIN_VALID = 6;
        Scheduler.reset();
        batteryVolts = 12;
        VoltageCompensator voltage = new VoltageCompensator(() -> batteryVolts);
        intakeMotor = new FakeMotor(); transferMotor = new FakeMotor();
        shooterOne = new FakeMotor(); shooterTwo = new FakeMotor(); turretServo = new FakeServo(); secondTurretServo = new FakeServo();
        turret = new TurretController(turretServo.servo, secondTurretServo.servo, Alliance.BLUE);
        stopperServo = new FakeServo();
        ShootingSystem.STOPPER_OPEN_POSITION = 0; ShootingSystem.STOPPER_CLOSED_POSITION = 0.5;
        ShootingSystem.SHOOTER_POWER = .75;
        ShootingSystem.SPINUP_SECONDS = 1;
        ShootingSystem.INTAKE_FEED_POWER = .8;
        ShootingSystem.TRANSFER_FEED_POWER = 1;
        now = 0;
        shooting = new ShootingSystem(shooterOne.motor, shooterTwo.motor, intakeMotor.motor,
                transferMotor.motor, stopperServo.servo, () -> now);
        stateController = new StateController(voltage, shooting, turret);
    }

    @Test public void targetSplitsOnYWithHysteresisAndAlliancePicksX() {
        for (double x : new double[] { 24, 120 }) {
            assertEquals(TurretAim.HiveHalf.LOWER, TurretAim.select(new Pose(x, 72), null));
            assertEquals(TurretAim.HiveHalf.UPPER, TurretAim.select(new Pose(x, 72.5), null));
        }
        assertEquals(TurretAim.HiveHalf.LOWER, TurretAim.select(new Pose(0, 73.5), TurretAim.HiveHalf.LOWER));
        assertEquals(TurretAim.HiveHalf.UPPER, TurretAim.select(new Pose(0, 74.5), TurretAim.HiveHalf.LOWER));
        assertEquals(TurretAim.HiveHalf.UPPER, TurretAim.select(new Pose(0, 70.5), TurretAim.HiveHalf.UPPER));
        assertEquals(TurretAim.HiveHalf.LOWER, TurretAim.select(new Pose(0, 69.5), TurretAim.HiveHalf.UPPER));

        Pose redLow = TurretAim.hive(Alliance.RED, TurretAim.HiveHalf.LOWER);
        Pose redHigh = TurretAim.hive(Alliance.RED, TurretAim.HiveHalf.UPPER);
        Pose blueLow = TurretAim.hive(Alliance.BLUE, TurretAim.HiveHalf.LOWER);
        Pose blueHigh = TurretAim.hive(Alliance.BLUE, TurretAim.HiveHalf.UPPER);
        assertEquals(60, redLow.x(), EPS); assertEquals(60, redLow.y(), EPS);
        assertEquals(60, redHigh.x(), EPS); assertEquals(84, redHigh.y(), EPS);
        assertEquals(84, blueLow.x(), EPS); assertEquals(60, blueLow.y(), EPS);
        assertEquals(84, blueHigh.x(), EPS); assertEquals(84, blueHigh.y(), EPS);
    }

    @Test public void aimingSubtractsRobotHeadingAndAppliesPivotAndOffset() {
        Pose robotPose = new Pose(10, 20, Math.PI / 2);
        RobotConfig.TURRET_PIVOT_FORWARD_IN = 5; RobotConfig.TURRET_PIVOT_LEFT_IN = 2;
        Pose pivot = TurretAim.pivot(robotPose);
        assertEquals(8, pivot.x(), EPS); assertEquals(25, pivot.y(), EPS);
        assertEquals(7, TurretAim.relativeDegrees(pivot, new Pose(8, 80), 7), EPS);
        assertEquals(-173, TurretAim.relativeDegrees(pivot, new Pose(8, 0), 7), EPS);
    }

    @Test public void turretLimitFindsEquivalentAngleAndDoesNotWrapThroughStops() {
        TurretAim.LimitedAngle equivalent = TurretAim.limit(-179, 180, 0, 270);
        assertTrue(equivalent.reachable); assertEquals(181, equivalent.degrees, EPS);
        TurretAim.LimitedAngle unavailable = TurretAim.limit(170, -140, -150, 150);
        assertFalse(unavailable.reachable); assertEquals(-150, unavailable.degrees, EPS);
    }

    @Test public void bothServosStayAtHalfForEveryRobotHeading() {
        RobotConfig.SHOOT_FLIGHT_TIME_SECONDS = 0;
        for (double heading : new double[] { 0, Math.PI / 2, Math.PI, -Math.PI / 2 }) {
            turret.reset();
            Pose pose = new Pose(120, 120, heading);
            turret.update(pose, .1);
            assertEquals(0.5, turretServo.position, EPS);
            assertEquals(0.5, secondTurretServo.position, EPS);
            turret.aim();
            turret.update(pose, .1);
            assertEquals(0.5, turretServo.position, EPS);
            assertEquals(0.5, secondTurretServo.position, EPS);
        }
    }

    @Test public void angleCommandsCannotMoveEitherServoFromHalf() {
        for (double angle : new double[] { 180, 90, 270, 180 }) {
            turret.setTargetPosition(angle);
            turret.update(new Pose(120, 120, 0), .1);
            assertEquals(0.5, turretServo.position, EPS);
            assertEquals(0.5, secondTurretServo.position, EPS);
        }
    }

    @Test public void turretWritesHalfAtInitializationAndReassertsItOnEveryUpdate() {
        assertEquals(0.5, turretServo.position, EPS);
        assertEquals(0.5, secondTurretServo.position, EPS);
        RobotConfig.TURRET_CENTER_POSITION = 0.7;
        turret.setManualAngle(90.0);
        for (int i = 0; i < 4; i++) {
            turretServo.position = 0.2;
            secondTurretServo.position = 0.8;
            int writes = turretServo.positionWrites;
            int secondWrites = secondTurretServo.positionWrites;
            if (i % 2 == 0) turret.update(Pose.zero(), .02);
            else turret.update(new Pose(Double.NaN, 1, 0));
            assertEquals(writes + 1, turretServo.positionWrites);
            assertEquals(secondWrites + 1, secondTurretServo.positionWrites);
            assertEquals(0.5, turretServo.position, EPS);
            assertEquals(0.5, secondTurretServo.position, EPS);
        }
    }

    @Test public void turretServoPositionUsesGearRatio() {
        assertEquals(0.5, TurretController.toServoPosition(0), EPS);
        assertEquals(1.0, TurretController.toServoPosition(247.5), EPS);
        assertEquals(0.0, TurretController.toServoPosition(-247.5), EPS);
        assertEquals(0.5 + 90 / 495.0, TurretController.toServoPosition(90), EPS);
    }

    @Test public void turretNudgeAndResetKeepBothServosAtHalf() {
        turret.nudge(2); turret.update(Pose.zero(), .1);
        assertEquals(0.5, turretServo.position, EPS);
        assertEquals(0.5, secondTurretServo.position, EPS);
        turret.nudge(-2); turret.nudge(-2); turret.update(Pose.zero(), .1);
        assertEquals(0.5, turretServo.position, EPS);
        assertEquals(0.5, secondTurretServo.position, EPS);
        turret.reset(); turret.update(Pose.zero(), .1);
        assertEquals(0.5, turretServo.position, EPS); assertEquals(0, turret.getTrimDegrees(), EPS);
    }

    @Test public void turretTrimShiftsAimAndAimedWaitsForEstimatedTravel() {
        Pose pose = new Pose(120, 120, -Math.PI / 2);
        turret.aim(); turret.update(pose, .001);
        double untrimmed = turret.getTargetAngleDegrees();
        assertFalse(turret.isAimed());
        for (int i = 0; i < 10; i++) turret.update(pose, .1);
        assertTrue(turret.isAimed());
        turret.nudge(2); turret.update(pose, .1);
        assertEquals(untrimmed + 2, turret.getTargetAngleDegrees(), EPS);
    }

    @Test public void invalidOdometryCannotMarkTurretAimed() {
        turret.aim(); turret.update(new Pose(Double.NaN, 1, 0), .1);
        assertFalse(turret.isAimed());
    }

    @Test public void movingShotCancelsInheritedBallVelocityForBothAlliances() {
        for (Alliance alliance : Alliance.values()) {
            Pose target = TurretAim.hive(alliance, TurretAim.HiveHalf.UPPER);
            Pose virtual = TurretAim.movingShotTarget(target, 24, -10, .35);

            assertEquals(target.x(), virtual.x() + 24 * .35, EPS);
            assertEquals(target.y(), virtual.y() - 10 * .35, EPS);
        }
    }

    @Test public void turretLeadsWhileMovingAndReturnsToDirectAimWhenStopped() {
        for (Alliance alliance : Alliance.values()) {
            TurretController moving = new TurretController(turretServo.servo, secondTurretServo.servo, alliance);
            Pose before = new Pose(120, 120, -Math.PI / 2);
            Pose now = new Pose(122, 120, -Math.PI / 2);
            moving.update(before, .1);
            moving.aim(); moving.update(now, .1);
            Pose hive = TurretAim.hive(alliance, TurretAim.HiveHalf.UPPER);
            double expected = TurretAim.relativeDegrees(now, hive.withX(hive.x() - 7), -180);
            assertEquals(expected, moving.getTargetAngleDegrees(), EPS);
            moving.update(now, .1);
            assertEquals(TurretAim.relativeDegrees(now, hive, -180), moving.getTargetAngleDegrees(), EPS);
        }
    }

    @Test public void zeroFlightTimePreservesDirectAimDuringMotion() {
        RobotConfig.SHOOT_FLIGHT_TIME_SECONDS = 0;
        turret.update(new Pose(120, 120, -Math.PI / 2), .1);
        Pose now = new Pose(122, 120, -Math.PI / 2);
        turret.aim(); turret.update(now, .1);
        Pose hive = TurretAim.hive(Alliance.BLUE, TurretAim.HiveHalf.UPPER);
        assertEquals(TurretAim.relativeDegrees(now, hive, -180), turret.getTargetAngleDegrees(), EPS);
    }

    @Test public void poseResetDoesNotCreateAnExtremeMovingShotCorrection() {
        turret.update(new Pose(10, 10), .02);
        Pose reset = new Pose(120, 120, -Math.PI / 2);
        turret.aim(); turret.update(reset, .02);
        Pose hive = TurretAim.hive(Alliance.BLUE, TurretAim.HiveHalf.UPPER);
        assertEquals(TurretAim.relativeDegrees(reset, hive, -180), turret.getTargetAngleDegrees(), EPS);
    }

    @Test public void voltageCompensationScalesPowerAndIgnoresBadReadings() throws InterruptedException {
        VoltageCompensator comp = new VoltageCompensator(() -> batteryVolts);
        batteryVolts = 10;
        assertEquals(0.6, comp.compensate(0.5), EPS);
        batteryVolts = 0;
        Thread.sleep(110);
        assertEquals(10, comp.getVoltage(), EPS);
        batteryVolts = 6.5;
        RobotConfig.VOLTAGE_FILTER_ALPHA = 1;
        Thread.sleep(110);
        assertEquals(1.0, comp.compensate(0.9), EPS);
        assertEquals(1.5, comp.scale(), EPS);
        RobotConfig.VOLTAGE_COMPENSATION = false;
        assertEquals(0.5, comp.compensate(0.5), EPS);
    }

    @Test public void blueAutoPathsAreRedRotatedAboutFieldCenter() {
        Pose red = new AutoPaths(Alliance.RED).start, blue = new AutoPaths(Alliance.BLUE).start;
        assertEquals(56, red.x(), EPS); assertEquals(8, red.y(), EPS); assertEquals(Math.PI / 2, red.heading(), EPS);
        assertEquals(144 - 56, blue.x(), EPS); assertEquals(144 - 8, blue.y(), EPS);
        assertEquals(0, Math.sin(blue.heading() - (red.heading() + Math.PI)), EPS);
        assertEquals(1, Math.cos(blue.heading() - (red.heading() + Math.PI)), EPS);
    }

    @Test public void cornerResetIsBottomLeftForRedAndMirroredForBlue() {
        Pose red = PoseStorage.cornerPose(Alliance.RED), blue = PoseStorage.cornerPose(Alliance.BLUE);
        assertEquals(RobotConfig.RED_CORNER_X, red.x(), EPS); assertEquals(RobotConfig.RED_CORNER_Y, red.y(), EPS);
        assertEquals(144 - red.x(), blue.x(), EPS); assertEquals(144 - red.y(), blue.y(), EPS);
        assertEquals(red.heading() + Math.PI, blue.heading(), EPS);
    }

    private Pose readyPose() {
        Pose pose = new Pose(120, 120, -Math.PI / 2);
        turret.aim();
        for (int i = 0; i < 10; i++) turret.update(pose, .1);
        stateController.spinUp(); stateController.update(pose);
        now += 1_000_000_000L; stateController.update(pose);
        return pose;
    }

    @Test public void continuousAimTracksInIdleIntakeAndEjectWithoutFeeding() {
        stateController.setTurretAlwaysOn(true);
        for (StateController.Mode mode : new StateController.Mode[] { StateController.Mode.IDLE, StateController.Mode.INTAKING, StateController.Mode.EJECTING }) {
            stateController.setMode(mode);
            stateController.update(new Pose(120, 120, -Math.PI / 2));
            double target = turret.getTargetAngleDegrees();
            assertTrue(turret.isTargetReachable());

            stateController.update(new Pose(120, 120, -Math.PI / 2 + Math.toRadians(15)));
            assertEquals(target - 15, turret.getTargetAngleDegrees(), EPS);
            assertFalse(stateController.isFeeding());
            assertEquals(ShootingSystem.STOPPER_CLOSED_POSITION, stopperServo.position, EPS);
            assertEquals(0, shooterOne.power, EPS);
            assertEquals(0, shooterTwo.power, EPS);
        }
    }

    @Test public void shootingUsesOnlyPowerAndTimerEvenWithInvalidOdometry() {
        Pose invalid = new Pose(Double.NaN, 0, 0);
        stateController.shoot();
        stateController.update(invalid);
        assertEquals(.75, shooterOne.power, EPS);
        assertEquals(.75, shooterTwo.power, EPS);
        assertFalse(stateController.isFeeding());
        assertEquals(ShootingSystem.STOPPER_CLOSED_POSITION, stopperServo.position, EPS);
        now = 1_000_000_000L;
        stateController.update(invalid);
        assertTrue(stateController.isFeeding());
        assertEquals(ShootingSystem.INTAKE_FEED_POWER, intakeMotor.power, EPS);
        assertEquals(ShootingSystem.TRANSFER_FEED_POWER, transferMotor.power, EPS);
        assertEquals(ShootingSystem.STOPPER_OPEN_POSITION, stopperServo.position, EPS);
        stateController.intake();
        stateController.update(invalid);
        assertEquals(0, shooterOne.power, EPS);
        assertEquals(0, shooterTwo.power, EPS);
        assertFalse(stateController.isFeeding());
        stateController.shoot();
        stateController.update(invalid);
        assertFalse(stateController.isFeeding());
        stateController.stop();
        assertEquals(0, intakeMotor.power, EPS);
        assertEquals(0, transferMotor.power, EPS);
        assertEquals(ShootingSystem.STOPPER_CLOSED_POSITION, stopperServo.position, EPS);
    }

    @Test public void continuousAimStopsAndResetsUntilEnabledAgain() {
        Pose pose = new Pose(120, 120, -Math.PI / 2);
        Pose rotated = new Pose(120, 120, 0);
        stateController.setTurretAlwaysOn(true);
        stateController.update(pose);
        double target = turret.getTargetAngleDegrees();

        stateController.stop();
        stateController.update(rotated);
        assertEquals(target, turret.getTargetAngleDegrees(), EPS);
        assertFalse(turret.isTargetReachable());

        stateController.setTurretAlwaysOn(true);
        stateController.reset();
        stateController.update(rotated);
        assertEquals(0.5, turretServo.position, EPS);
        assertFalse(turret.isTargetReachable());

        stateController.setTurretAlwaysOn(true);
        stateController.update(rotated);
        assertTrue(turret.isTargetReachable());
        assertEquals(0.5, turretServo.position, EPS);
        assertFalse(stateController.isFeeding());
    }

    @Test public void resetStopsEverythingAndCentersTurret() {
        Pose pose = readyPose();
        stateController.setMode(StateController.Mode.SHOOTING); stateController.update(pose);
        assertEquals(0.5, turretServo.position, EPS);
        stateController.reset(); stateController.update(pose);
        assertEquals(StateController.Mode.IDLE, stateController.getMode());
        assertEquals(0.5, turretServo.position, EPS);
        assertEquals(0, shooterOne.power, EPS); assertEquals(0, intakeMotor.power, EPS);
    }

    @Test public void fieldCommandsTransformWithRobotHeading() {
        DrivePowers powers = DriveController.toRobotFrame(0, 1, 0.3, Math.PI / 2);
        assertEquals(1, powers.forward(), EPS); assertEquals(0, powers.strafe(), EPS); assertEquals(.3, powers.turn(), EPS);
    }

    @Test public void avoidanceSteersAndSlowModeBypassesExactly() {
        PillarAvoidance avoidance = new PillarAvoidance();
        Pose robotPose = new Pose(72, RobotConfig.PILLAR_TOP_Y + avoidance.getClearanceRadius() + 2);
        PillarAvoidance.Output safe = avoidance.apply(robotPose, 0, -1, 0.2, false);
        assertTrue(safe.avoiding); assertTrue(Math.abs(safe.x) > 0); assertTrue(safe.y > -1);
        RobotConfig.AVOIDANCE_ENABLED = false;
        PillarAvoidance.Output disabled = avoidance.apply(robotPose, 0, -1, 0.2, false);
        assertEquals(0, disabled.x, EPS); assertEquals(-1, disabled.y, EPS); assertFalse(disabled.avoiding);
        RobotConfig.AVOIDANCE_ENABLED = true;
        PillarAvoidance.Output bypass = avoidance.apply(robotPose, 0, -1, 0.2, true);
        assertEquals(0, bypass.x, EPS); assertEquals(-1, bypass.y, EPS); assertEquals(.2, bypass.turn, EPS);
        assertFalse(bypass.avoiding);
        PillarAvoidance.Output stopped = avoidance.apply(robotPose, 0, 0, .2, false);
        assertEquals(0, stopped.x, EPS); assertEquals(0, stopped.y, EPS);
    }

    @Test public void avoidanceBlocksFootBarMiddleAndStillAllowsMovingAway() {
        PillarAvoidance avoidance = new PillarAvoidance();
        Pose robotPose = new Pose(72, RobotConfig.PILLAR_TOP_Y + avoidance.getClearanceRadius());
        assertTrue(avoidance.apply(robotPose, 0, -1, 0, false).y >= -EPS);
        PillarAvoidance.Output escape = avoidance.apply(robotPose, 0, 1, 0, false);
        assertEquals(1, escape.y, EPS);
    }

    @Test public void randomizedAvoidanceCommandsDoNotCrossExpandedSupportsWithinLookahead() {
        PillarAvoidance avoidance = new PillarAvoidance();
        Random random = new Random(30927);
        int checked = 0;
        for (int sample = 0; sample < 3000; sample++) {
            Pose pose = new Pose(10 + random.nextDouble() * 124, 10 + random.nextDouble() * 124);
            boolean inside = false;
            for (Pose obstacle : avoidance.getObstaclePoints(pose)) {
                if (pose.distance(obstacle) < avoidance.getClearanceRadius()) inside = true;
            }
            if (inside) continue;
            double angle = random.nextDouble() * 2 * Math.PI;
            PillarAvoidance.Output safe = avoidance.apply(pose, Math.cos(angle), Math.sin(angle), 0, false);
            assertTrue(Math.hypot(safe.x, safe.y) <= 1 + EPS);
            for (int t = 1; t <= 10; t++) {
                double travel = RobotConfig.AVOIDANCE_MAX_SPEED_IPS * RobotConfig.AVOIDANCE_LOOKAHEAD_SECONDS * t / 10;
                Pose predicted = new Pose(pose.x() + safe.x * travel, pose.y() + safe.y * travel);
                for (Pose obstacle : avoidance.getObstaclePoints(predicted)) {
                    assertTrue("Command crosses a support", predicted.distance(obstacle) >= avoidance.getClearanceRadius() - EPS);
                }
            }
            checked++;
        }
        assertTrue(checked > 1000);
    }

    @Test public void autoPoseHandoffIsAllianceMatchedAndConsumedOnce() {
        PoseStorage.save(new Pose(20, 30, 1), Alliance.RED);
        assertEquals(20, PoseStorage.consume(Alliance.RED).x(), EPS);
        assertNull(PoseStorage.consume(Alliance.RED));
        PoseStorage.save(new Pose(20, 30, 1), Alliance.RED);
        assertNull(PoseStorage.consume(Alliance.BLUE));
    }

    private boolean runUntilFinished(Command command, Pose pose, long maxMs) throws InterruptedException {
        Scheduler.schedule(command);
        long deadline = System.currentTimeMillis() + maxMs;
        while (command.isScheduled()) {
            if (System.currentTimeMillis() > deadline) return false;
            Scheduler.execute();
            stateController.update(pose);
            Thread.sleep(5);
        }
        stateController.update(pose);
        return true;
    }

    @Test public void autoTimesOutWithoutFeedingWhenSpinupDelayNeverElapses() throws InterruptedException {
        RobotConfig.AUTO_READY_TIMEOUT_MS = 50;
        Command auto = sequential(waitUntilReady(stateController), shoot(stateController, 20));
        assertTrue(runUntilFinished(auto, new Pose(120, 120), 2000));
        assertFalse(stateController.isFeeding());
        assertEquals(0, transferMotor.power, EPS);
    }

    @Test public void autoFeedsForRequestedTimeThenStopsAndClosesGate() throws InterruptedException {
        Pose pose = readyPose();
        long started = System.nanoTime();
        assertTrue(runUntilFinished(shoot(stateController, 60), pose, 2000));
        assertTrue((System.nanoTime() - started) / 1e6 >= 60);
        assertEquals(StateController.Mode.IDLE, stateController.getMode());
        assertEquals(0, transferMotor.power, EPS); assertEquals(0, intakeMotor.power, EPS);
        assertEquals(0, shooterOne.power, EPS); assertEquals(0, shooterTwo.power, EPS);
        assertEquals(ShootingSystem.STOPPER_CLOSED_POSITION, stopperServo.position, EPS);
    }

    @Test public void parallelIntakeRunsDuringWaitThenSequenceContinues() throws InterruptedException {
        Scheduler.schedule(sequential(parallel(intake(stateController), waitMs(40)), idle(stateController)));
        Scheduler.execute(); stateController.update(Pose.zero());
        assertEquals(StateController.Mode.INTAKING, stateController.getMode()); assertTrue(intakeMotor.power > 0);
        Thread.sleep(60);
        Scheduler.execute(); Scheduler.execute(); stateController.update(Pose.zero());
        assertEquals(StateController.Mode.IDLE, stateController.getMode()); assertEquals(0, intakeMotor.power, EPS);
    }
}
