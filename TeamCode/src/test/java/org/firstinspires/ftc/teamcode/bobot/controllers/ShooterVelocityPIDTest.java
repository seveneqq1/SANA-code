package org.firstinspires.ftc.teamcode.bobot.controllers;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class ShooterVelocityPIDTest {

    // Illustrative plant, not a calibration of the real robot. Check several loop periods
    // and a speed drop to catch excessive gain causing alternating full/zero commands.
    private static void checkRecovery(double kP) {
        for (double dt : new double[]{.01, .02, .05, .1}) {
            ShooterVelocityPID pid = new ShooterVelocityPID();
            double velocity = 0;
            for (int n = 0; n < 600; n++) {
                if (n == 300) velocity *= .5;
                double power = pid.calculate(2400, velocity, dt, kP, 0, 0, .67 / 2400);
                velocity += (3600 * power - velocity) * (1 - Math.exp(-dt / .15));
                if (n > 500 && Math.abs(velocity - 2400) >= 120) {
                    throw new AssertionError("Speed fails to settle with kP=" + kP + " dt=" + dt
                            + " speed=" + velocity);
                }
            }
        }
    }

    // Feedforward deliberately underestimates the power needed by this test plant.
    // A persistent load must be corrected by I rather than widening feed tolerance.
    private static void checkSteadyStateCorrection(double kI) {
        for (double dt : new double[]{.01, .02, .05, .1}) {
            ShooterVelocityPID pid = new ShooterVelocityPID();
            double velocity = 0;
            for (int n = 0; n < (int) (30 / dt); n++) {
                double power = pid.calculate(2400, velocity, dt, .0003, kI, 0, .67 / 2400);
                velocity += (2800 * power - velocity) * (1 - Math.exp(-dt / .15));
            }
            if (Math.abs(velocity - 2400) > 20) {
                throw new AssertionError("Persistent speed offset with kI=" + kI
                        + " dt=" + dt + " speed=" + velocity);
            }
        }
    }

    @Test
    public void calculatesBoundedPowerAndResetsPidState() {
        ShooterVelocityPID pid = new ShooterVelocityPID();
        assertEquals(.5, pid.calculate(100, 50, .1, .01, 0, 0, 0), 1e-9);
        assertEquals(0, pid.calculate(100, 150, .1, .01, 0, 0, 0), 1e-9);
        pid.reset();
        assertEquals(.1, pid.calculate(100, 90, .1, 0, .1, 0, 0), 1e-9);
        assertEquals(.2, pid.calculate(100, 90, .1, 0, .1, 0, 0), 1e-9);
        pid.reset();
        assertEquals(.1, pid.calculate(100, 90, .1, 0, .1, 0, 0), 1e-9);
        // Saturation must not accumulate integral that delays recovery.
        pid.reset();
        for (int n = 0; n < 100; n++) assertEquals(1, pid.calculate(100, 0, .1, .02, .1, 0, 0), 1e-9);
        assertEquals(0, pid.calculate(100, 100, .1, .02, .1, 0, 0), 1e-9);
        // Derivative on measurement avoids a kick on a target step.
        pid.reset();
        assertEquals(.1, pid.calculate(100, 50, .1, 0, 0, .01, .001), 1e-9);
        assertEquals(.2, pid.calculate(200, 50, .1, 0, 0, .01, .001), 1e-9);
        assertEquals(.1, pid.calculate(200, 51, .1, 0, 0, .01, .001), 1e-9);
        assertEquals(0, pid.calculate(0, 50, .1, .01, .1, .01, .001), 1e-9);
        assertEquals(0, pid.calculate(100, Double.NaN, .1, .01, 0, 0, 0), 1e-9);
        assertEquals(1, pid.calculate(100, 0, 0, .01, .1, .01, 0), 1e-9);
    }

    @Test
    public void regainsTargetAfterSpeedDrop() {
        checkRecovery(0.0003);
    }

    @Test
    public void integralCorrectsPersistentLoad() {
        checkSteadyStateCorrection(0.0002);
    }
}
