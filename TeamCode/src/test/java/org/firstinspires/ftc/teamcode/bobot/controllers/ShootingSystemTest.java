package org.firstinspires.ftc.teamcode.bobot.controllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.reflect.Proxy;
import org.firstinspires.ftc.teamcode.bobot.config.RobotConfig;
import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.*;

public class ShootingSystemTest {
    private static final double EPS = 1e-9;
    private final Motor left = new Motor(), right = new Motor(), intake = new Motor(), transfer = new Motor();
    private long now;
    private double gate;
    private ShootingSystem system;

    private static class Motor {
        double power;
        final DcMotor device = (DcMotor) Proxy.newProxyInstance(DcMotor.class.getClassLoader(),
                new Class<?>[] { DcMotor.class }, (proxy, method, args) -> {
                    if (method.getName().equals("setPower")) power = (double) args[0];
                    if (method.getName().equals("getCurrentPosition")) throw new AssertionError("No encoder dependency");
                    return null;
                });
    }

    @Before public void setup() {
        ShootingSystem.SHOOTER_POWER = .75;
        ShootingSystem.SPINUP_SECONDS = 1;
        ShootingSystem.INTAKE_FEED_POWER = .8;
        ShootingSystem.TRANSFER_FEED_POWER = 1;
        ShootingSystem.STOPPER_OPEN_POSITION = .5;
        ShootingSystem.STOPPER_CLOSED_POSITION = 0;
        RobotConfig.INTAKE_POWER = 1;
        RobotConfig.TRANSFER_COLLECT_POWER = .35;
        RobotConfig.INTAKE_EJECT_POWER = -1;
        RobotConfig.TRANSFER_EJECT_POWER = -.8;
        Servo stopper = (Servo) Proxy.newProxyInstance(Servo.class.getClassLoader(),
                new Class<?>[] { Servo.class }, (proxy, method, args) -> {
                    if (method.getName().equals("setPosition")) gate = (double) args[0];
                    return null;
                });
        system = new ShootingSystem(left.device, right.device, intake.device, transfer.device,
                stopper, () -> now);
    }

    private void outputs(double flywheel, double intakePower, double transferPower, double stopper) {
        assertEquals(flywheel, left.power, EPS);
        assertEquals(flywheel, right.power, EPS);
        assertEquals(intakePower, intake.power, EPS);
        assertEquals(transferPower, transfer.power, EPS);
        assertEquals(stopper, gate, EPS);
    }

    @Test public void holdSpinsThenFeedsAtExactDelayAndKeepsConstantPower() {
        outputs(0, 0, 0, 0);
        system.update(ShootingSystem.Mode.SHOOT);
        outputs(.75, 0, 0, 0);
        assertFalse(system.isReady());
        now = 999_999_999L;
        system.update(ShootingSystem.Mode.SHOOT);
        outputs(.75, 0, 0, 0);
        now++;
        system.update(ShootingSystem.Mode.SHOOT);
        outputs(.75, .8, 1, .5);
        assertTrue(system.isFeeding());
        now += 30_000_000_000L;
        system.update(ShootingSystem.Mode.SHOOT);
        outputs(.75, .8, 1, .5);
    }

    @Test public void earlyReleaseCancelsAndNextHoldGetsFullDelay() {
        system.update(ShootingSystem.Mode.SHOOT);
        now = 500_000_000L;
        system.update(ShootingSystem.Mode.COLLECT);
        outputs(0, 1, .35, 0);
        assertFalse(system.isReady());
        now = 2_000_000_000L;
        system.update(ShootingSystem.Mode.SHOOT);
        outputs(.75, 0, 0, 0);
        now += 999_999_999L;
        system.update(ShootingSystem.Mode.SHOOT);
        assertFalse(system.isFeeding());
        now++;
        system.update(ShootingSystem.Mode.SHOOT);
        assertTrue(system.isFeeding());
        system.update(ShootingSystem.Mode.COLLECT);
        outputs(0, 1, .35, 0);
    }

    @Test public void ejectAndStopOverrideShootingAndResetTimer() {
        system.update(ShootingSystem.Mode.SHOOT);
        now = 1_000_000_000L;
        system.update(ShootingSystem.Mode.SHOOT);
        system.update(ShootingSystem.Mode.EJECT);
        outputs(0, -1, -.8, 0);
        system.update(ShootingSystem.Mode.SHOOT);
        outputs(.75, 0, 0, 0);
        now += 1_000_000_000L;
        system.update(ShootingSystem.Mode.SHOOT);
        system.stop();
        outputs(0, 0, 0, 0);
        assertFalse(system.isFeeding());
        system.update(ShootingSystem.Mode.SHOOT);
        outputs(.75, 0, 0, 0);
    }

    @Test public void autonomousSpinupDoesNotFeedAndTransitionKeepsElapsedTime() {
        system.update(ShootingSystem.Mode.SPINUP);
        now = 1_000_000_000L;
        system.update(ShootingSystem.Mode.SPINUP);
        outputs(.75, 0, 0, 0);
        assertTrue(system.isReady());
        assertFalse(system.isFeeding());
        system.update(ShootingSystem.Mode.SHOOT);
        outputs(.75, .8, 1, .5);
    }

    @Test public void dashboardChangesApplyLiveAndZeroPowerCannotFeed() {
        system.update(ShootingSystem.Mode.SHOOT);
        now = 500_000_000L;
        ShootingSystem.SHOOTER_POWER = .4;
        ShootingSystem.SPINUP_SECONDS = .5;
        ShootingSystem.INTAKE_FEED_POWER = .6;
        ShootingSystem.TRANSFER_FEED_POWER = .7;
        system.update(ShootingSystem.Mode.SHOOT);
        outputs(.4, .6, .7, .5);
        ShootingSystem.SHOOTER_POWER = 0;
        system.update(ShootingSystem.Mode.SHOOT);
        outputs(0, 0, 0, 0);
        ShootingSystem.SHOOTER_POWER = .4;
        system.update(ShootingSystem.Mode.SHOOT);
        outputs(.4, 0, 0, 0);
    }

    @Test public void clampsPowersAndTreatsInvalidPowerAsOff() {
        ShootingSystem.SPINUP_SECONDS = 0;
        ShootingSystem.SHOOTER_POWER = 2;
        ShootingSystem.INTAKE_FEED_POWER = 2;
        ShootingSystem.TRANSFER_FEED_POWER = -1;
        system.update(ShootingSystem.Mode.SHOOT);
        outputs(1, 1, 0, .5);
        ShootingSystem.SHOOTER_POWER = Double.NaN;
        system.update(ShootingSystem.Mode.SHOOT);
        outputs(0, 0, 0, 0);
    }
}
