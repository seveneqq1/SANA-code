package org.firstinspires.ftc.teamcode.bobot.controller;

/** Velocity PID + power-per-TPS feedforward, with output clamped to forward power. */
public class ShooterVelocityPID {
    private double integral, previousVelocity;
    private boolean hasPrevious;

    public void reset() {
        integral = 0;
        hasPrevious = false;
    }

    public double calculate(double target, double velocity, double dt,
            double kP, double kI, double kD, double kV) {
        if (!Double.isFinite(target) || !Double.isFinite(velocity) || target <= 0) {
            reset();
            return 0;
        }
        double error = target - velocity;
        boolean validDt = Double.isFinite(dt) && dt > 0;
        double derivative = validDt && hasPrevious ? -(velocity - previousVelocity) / dt : 0;
        previousVelocity = velocity;
        hasPrevious = true;
        double nextIntegral = kI == 0 ? 0 : integral + (validDt ? error * dt : 0);
        double output = kV * target + kP * error + kI * nextIntegral + kD * derivative;
        // Integrate only when unsaturated, or when the error drives us out of saturation.
        if ((output >= 0 && output <= 1) || (output > 1 && error < 0)
                || (output < 0 && error > 0)) {
            integral = nextIntegral;
        }
        output = kV * target + kP * error + kI * integral + kD * derivative;
        return Double.isFinite(output) ? Math.max(0, Math.min(1, output)) : 0;
    }
}
