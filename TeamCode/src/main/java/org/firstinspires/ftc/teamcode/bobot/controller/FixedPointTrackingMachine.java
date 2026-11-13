package org.firstinspires.ftc.teamcode.bobot.controller;

/**
 * Turret always aims at one point (targetX, targetY) in Pinpoint inches.
 * Pinpoint frame: +X forward from the start pose, +Y left, heading CCW.
 * If the point is outside the turret range, the turret waits at the nearest limit.
 */
public class FixedPointTrackingMachine extends TrackingStateMachine {
    private boolean outOfRange;

    public FixedPointTrackingMachine(shooterController_var2 shooter, intakeController intake,
            turretController turret) {
        super(shooter, intake, turret, null);
    }

    public void setTarget(double x, double y) {
        targetX = x;
        targetY = y;
    }

    public boolean isOutOfRange() { return outOfRange; }

    /** Reachable turret angle limits in radians (servo 0..1 and min/max_target_pos). */
    public double lowLimit() {
        double half = 0.5 / turretController.kV;
        return Math.toRadians(Math.max(turretController.min_target_pos,
                turretController.center_target_pos - half) + 0.5);
    }

    public double highLimit() {
        double half = 0.5 / turretController.kV;
        return Math.toRadians(Math.min(turretController.max_target_pos,
                turretController.center_target_pos + half) - 0.5);
    }

    /**
     * The driver aimed the turret by hand at the real target: move the target point onto
     * that line, keeping the same distance from the robot, then go back to AUTO.
     */
    public void lockTargetHere(double x, double y, double heading) {
        double dist = Math.hypot(targetX - x, targetY - y);
        if (dist < 1) dist = 1;
        double relative = (commandedAngle - Math.toRadians(turretController.center_target_pos))
                / turretDirection;
        double world = heading + turretForwardRadians + relative;
        targetX = x + dist * Math.cos(world);
        targetY = y + dist * Math.sin(world);
        aimTrimRadians = 0;
        setTrackingEnabled(true);
    }

    public double getTargetX() { return targetX; }
    public double getTargetY() { return targetY; }

    /** Manual mode: stop tracking and move the turret by deltaDegrees (+ = left). */
    public void manualMove(double deltaDegrees) {
        setTrackingEnabled(false);
        outOfRange = false;
        double angle = commandedAngle + turretDirection * Math.toRadians(deltaDegrees);
        commandTurret(Math.max(lowLimit(), Math.min(highLimit(), angle)));
    }

    // Keep the chosen point; do not switch between field quarters.
    @Override
    protected void selectQuarter(double x, double y) { }

    @Override
    protected void updateAim(double x, double y, double heading) {
        double dx = targetX - x, dy = targetY - y;
        if (!isTrackingEnabled() || (dx == 0 && dy == 0)) {
            outOfRange = false;
            return;
        }
        double relative = wrap(Math.atan2(dy, dx) - heading - turretForwardRadians + aimTrimRadians);
        double lo = lowLimit(), hi = highLimit();
        double desired = Math.toRadians(turretController.center_target_pos) + turretDirection * relative;

        // Try every full turn; pick the reachable one closest to where the turret is now.
        double best = Double.NaN, shortest = Double.POSITIVE_INFINITY;
        for (int turns = -2; turns <= 2; turns++) {
            double candidate = desired + turns * 2 * Math.PI;
            if (candidate >= lo && candidate <= hi
                    && Math.abs(candidate - commandedAngle) < shortest) {
                best = candidate;
                shortest = Math.abs(candidate - commandedAngle);
            }
        }
        if (!Double.isNaN(best)) {
            outOfRange = false;
            commandTurret(best);
            return;
        }
        // Not reachable: wait at the limit that is angularly closer to the target.
        outOfRange = true;
        double toLo = Math.abs(wrap(desired - lo));
        double toHi = Math.abs(wrap(desired - hi));
        commandTurret(toLo < toHi ? lo : hi);
    }
}
