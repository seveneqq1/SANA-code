package org.firstinspires.ftc.teamcode.bobot.controller;

/** Radians, zero along +X, positive counterclockwise. Does not read or reset hardware. */
public class HeadingController {
    public enum Direction { ANTICLOCKWISE, CLOCKWISE }

    private double kP, kD;
    private double targetHeading;
    private double previousHeading;
    private boolean hasPreviousHeading;
    private double maxPower = 1;
    private double tolerance;
    private boolean holdingHeading;

    /** Gains are power/radian and power/(radian/second); tune them on this robot. */
    public HeadingController(double kP, double kD) {
        setGains(kP, kD);
    }

    public void setGains(double kP, double kD) {
        requireFinite(kP);
        requireFinite(kD);
        if (kP < 0 || kD < 0) {
            throw new IllegalArgumentException("Heading gains must be nonnegative");
        }
        this.kP = kP;
        this.kD = kD;
        hasPreviousHeading = false;
    }

    public void setTargetHeading(double radians) {
        requireFinite(radians);
        targetHeading = wrap(radians);
    }

    public void adjustTargetHeading(double deltaRadians) {
        requireFinite(deltaRadians);
        setTargetHeading(targetHeading + wrap(deltaRadians));
    }

    /** Move the target by 90 degrees; call once per button press. */
    public void snapTargetHeading(Direction direction) {
        if (direction == null) {
            throw new IllegalArgumentException("A snap direction is required");
        }
        adjustTargetHeading(direction == Direction.ANTICLOCKWISE ? Math.PI / 2 : -Math.PI / 2);
    }

    /** Hold the supplied measured heading and discard derivative history. */
    public void resetHeading(double currentHeading) {
        setTargetHeading(currentHeading);
        hasPreviousHeading = false;
    }

    /** lockPressed is a new button press; manual override takes priority over it. */
    public double updateHold(double currentHeading, double dtSeconds,
            boolean lockPressed, boolean manualOverride) {
        if (manualOverride) {
            disableHold();
            return 0;
        }
        if (lockPressed) {
            resetHeading(currentHeading);
            holdingHeading = true;
        }
        return holdingHeading ? calculate(currentHeading, dtSeconds) : 0;
    }

    public void disableHold() {
        holdingHeading = false;
        hasPreviousHeading = false;
    }

    public boolean isHoldingHeading() {
        return holdingHeading;
    }

    public void setMaxPower(double maxPower) {
        requireFinite(maxPower);
        if (maxPower < 0 || maxPower > 1) {
            throw new IllegalArgumentException("Maximum turn power must be in [0, 1]");
        }
        this.maxPower = maxPower;
    }

    public void setTolerance(double radians) {
        requireFinite(radians);
        if (radians < 0 || radians > Math.PI) {
            throw new IllegalArgumentException("Heading tolerance must be in [0, pi]");
        }
        tolerance = radians;
    }

    /** Positive output requests CCW rotation; adapt the sign at the drivetrain interface. */
    public double calculate(double currentHeading, double dtSeconds) {
        requireFinite(currentHeading);
        requireFinite(dtSeconds);
        if (dtSeconds <= 0) {
            throw new IllegalArgumentException("Loop time must be positive seconds");
        }
        double heading = wrap(currentHeading);
        double error = getError(heading);
        double rate = hasPreviousHeading ? wrap(heading - previousHeading) / dtSeconds : 0;
        previousHeading = heading;
        hasPreviousHeading = true;

        if (Math.abs(error) <= tolerance || maxPower == 0) {
            return 0;
        }
        // Derivative on measurement avoids a power spike when the target changes.
        double power = kP * error;
        if (kD != 0) {
            power -= kD * rate;
        }
        return Math.max(-maxPower, Math.min(maxPower, power));
    }

    public double getTargetHeading() {
        return targetHeading;
    }

    public double getError(double currentHeading) {
        requireFinite(currentHeading);
        return wrap(targetHeading - wrap(currentHeading));
    }

    public boolean isAtTarget(double currentHeading) {
        return Math.abs(getError(currentHeading)) <= tolerance;
    }

    private static double wrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    private static void requireFinite(double value) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException("Heading controller values must be finite");
        }
    }
}
