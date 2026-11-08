package org.firstinspires.ftc.teamcode.bobot.controller;

/** Standalone JVM regression checks; no Android hardware or test dependency required. */
public class HeadingControllerTest {
    private static void near(double actual, double expected) {
        if (!Double.isFinite(actual) || Math.abs(actual - expected) > 1e-9) {
            throw new AssertionError("Expected " + expected + ", got " + actual);
        }
    }

    private static void require(boolean value) {
        if (!value) {
            throw new AssertionError("Heading controller check failed");
        }
    }

    private static void rejects(Runnable action) {
        try {
            action.run();
        } catch (IllegalArgumentException expected) {
            return;
        }
        throw new AssertionError("Expected IllegalArgumentException");
    }

    public static void main(String[] args) {
        HeadingController heading = new HeadingController(1, 0);
        heading.setTargetHeading(Math.toRadians(-179));
        near(heading.calculate(Math.toRadians(179), 0.02), Math.toRadians(2));
        heading.setTargetHeading(Math.toRadians(179));
        near(heading.calculate(Math.toRadians(-179), 0.02), Math.toRadians(-2));

        heading.setMaxPower(0.25);
        heading.setTargetHeading(Math.PI / 2);
        near(heading.calculate(0, 0.02), 0.25);
        heading.setTargetHeading(-Math.PI / 2);
        near(heading.calculate(0, 0.02), -0.25);
        heading.setMaxPower(0);
        near(heading.calculate(0, 0.02), 0);

        heading.setTargetHeading(0);
        heading.snapTargetHeading(HeadingController.Direction.ANTICLOCKWISE);
        near(heading.getTargetHeading(), Math.PI / 2);
        heading.snapTargetHeading(HeadingController.Direction.CLOCKWISE);
        near(heading.getTargetHeading(), 0);
        heading.setTargetHeading(8 * Math.PI + 0.3);
        near(heading.getTargetHeading(), 0.3);
        heading.adjustTargetHeading(0.2);
        near(heading.getTargetHeading(), 0.5);
        heading.setMaxPower(1);
        heading.setTolerance(0.02);
        require(heading.isAtTarget(0.51));
        require(!heading.isAtTarget(0.53));
        near(heading.calculate(0.51, 0.02), 0);
        near(heading.calculate(0.53, 0.02), -0.03);

        HeadingController derivative = new HeadingController(0, 0.1);
        derivative.setTargetHeading(1);
        near(derivative.calculate(0, 0.1), 0);
        near(derivative.calculate(0.1, 0.1), -0.1);
        derivative.setTargetHeading(-1);
        near(derivative.calculate(0.1, 0.1), 0);
        derivative.resetHeading(Math.toRadians(179));
        near(derivative.calculate(Math.toRadians(179), 0.02), 0);
        near(derivative.calculate(Math.toRadians(-179), 0.02), -0.1 * Math.toRadians(2) / 0.02);
        derivative.resetHeading(0.4);
        near(derivative.getTargetHeading(), 0.4);
        near(derivative.calculate(0.4, 0.02), 0);

        HeadingController simulation = new HeadingController(1, 0);
        simulation.setTargetHeading(Math.PI / 2);
        double position = 0;
        for (int i = 0; i < 700; i++) {
            position += simulation.calculate(position, 0.02) * 0.02;
        }
        require(Math.abs(simulation.getError(position)) < 0.00001);

        HeadingController hold = new HeadingController(1, 0);
        near(hold.updateHold(0.4, 0.02, false, false), 0);
        require(!hold.isHoldingHeading());
        near(hold.updateHold(0.4, 0.02, true, false), 0);
        require(hold.isHoldingHeading());
        hold.snapTargetHeading(HeadingController.Direction.ANTICLOCKWISE);
        near(hold.updateHold(0.4, 0.02, false, false), 1);
        near(hold.updateHold(0.5, 0.02, false, true), 0);
        require(!hold.isHoldingHeading());
        near(hold.updateHold(0.5, 0.02, false, false), 0);
        near(hold.updateHold(0.6, 0.02, false, false), 0);
        require(!hold.isHoldingHeading());
        hold.snapTargetHeading(HeadingController.Direction.CLOCKWISE);
        require(!hold.isHoldingHeading());
        near(hold.updateHold(0.7, 0.02, true, false), 0);
        near(hold.getTargetHeading(), 0.7);
        require(hold.isHoldingHeading());
        near(hold.updateHold(0.8, 0.02, true, true), 0);
        require(!hold.isHoldingHeading());
        near(hold.updateHold(0.8, 0.02, false, false), 0);
        hold.updateHold(0.8, 0.02, true, false);
        hold.disableHold();
        near(hold.updateHold(0.9, 0.02, false, false), 0);

        rejects(() -> new HeadingController(-1, 0));
        rejects(() -> heading.setGains(1, Double.NaN));
        rejects(() -> heading.setTargetHeading(Double.POSITIVE_INFINITY));
        rejects(() -> heading.calculate(Double.NaN, 0.02));
        rejects(() -> heading.calculate(0, 0));
        rejects(() -> heading.calculate(0, -0.02));
        rejects(() -> heading.calculate(0, Double.NaN));
        rejects(() -> heading.setMaxPower(1.1));
        rejects(() -> heading.setTolerance(-0.1));
        rejects(() -> heading.snapTargetHeading(null));
        System.out.println("HeadingController checks passed: wrapping, signs, limits, snaps, tolerance, "
                + "derivative, reset, idealized convergence, latched manual override, and invalid inputs.");
    }
}
