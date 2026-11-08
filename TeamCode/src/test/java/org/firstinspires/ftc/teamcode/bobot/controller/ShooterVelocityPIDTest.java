package org.firstinspires.ftc.teamcode.bobot.controller;

/** Run with javac/java; no robot hardware or test framework required. */
public class ShooterVelocityPIDTest {
    private static void near(double actual, double expected) {
        if (Math.abs(actual - expected) > 1e-9) throw new AssertionError(actual + " != " + expected);
    }
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

    public static void main(String[] args) {
        ShooterVelocityPID pid = new ShooterVelocityPID();
        near(pid.calculate(100, 50, .1, .01, 0, 0, 0), .5);
        near(pid.calculate(100, 150, .1, .01, 0, 0, 0), 0);
        pid.reset();
        near(pid.calculate(100, 90, .1, 0, .1, 0, 0), .1);
        near(pid.calculate(100, 90, .1, 0, .1, 0, 0), .2);
        pid.reset();
        near(pid.calculate(100, 90, .1, 0, .1, 0, 0), .1);
        // Saturation must not accumulate integral that delays recovery.
        pid.reset();
        for (int n = 0; n < 100; n++) near(pid.calculate(100, 0, .1, .02, .1, 0, 0), 1);
        near(pid.calculate(100, 100, .1, .02, .1, 0, 0), 0);
        // Derivative on measurement avoids a kick on a target step.
        pid.reset();
        near(pid.calculate(100, 50, .1, 0, 0, .01, .001), .1);
        near(pid.calculate(200, 50, .1, 0, 0, .01, .001), .2);
        near(pid.calculate(200, 51, .1, 0, 0, .01, .001), .1);
        near(pid.calculate(0, 50, .1, .01, .1, .01, .001), 0);
        near(pid.calculate(100, Double.NaN, .1, .01, 0, 0, 0), 0);
        near(pid.calculate(100, 0, 0, .01, .1, .01, 0), 1);
        if (args.length > 0) checkRecovery(Double.parseDouble(args[0]));
        if (args.length > 1) checkSteadyStateCorrection(Double.parseDouble(args[1]));
        System.out.println("ShooterVelocityPIDTest passed");
    }
}
