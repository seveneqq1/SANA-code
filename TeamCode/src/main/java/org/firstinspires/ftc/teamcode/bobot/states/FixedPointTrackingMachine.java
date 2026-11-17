package org.firstinspires.ftc.teamcode.bobot.states;

import org.firstinspires.ftc.teamcode.bobot.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.bobot.controllers.ShooterController;
import org.firstinspires.ftc.teamcode.bobot.controllers.TurretController;

/**
 * Turret always aims at one point (targetX, targetY) in Pinpoint inches.
 * Pinpoint frame: +X forward from the start pose, +Y left, heading CCW.
 * If the point is outside the turret range, the turret waits at the nearest limit.
 */
public class FixedPointTrackingMachine extends TrackingStateMachine {
    private boolean outOfRange;

    public FixedPointTrackingMachine(ShooterController shooter, IntakeController intake,
            TurretController turret) {
        super(shooter, intake, turret);
    }

    public void setTarget(double x, double y) {
        targetX = x;
        targetY = y;
    }

    public boolean isOutOfRange() { return outOfRange; }

    /** Reachable turret angle limits in radians (servo 0..1 and min/max_target_pos). */
    public double lowLimit() {
        double atServo0 = TurretController.center_target_pos - TurretController.center_pos / TurretController.kV;
        return Math.toRadians(Math.max(TurretController.min_target_pos, atServo0) + 0.5);
    }

    public double highLimit() {
        double atServo1 = TurretController.center_target_pos + (1 - TurretController.center_pos) / TurretController.kV;
        return Math.toRadians(Math.min(TurretController.max_target_pos, atServo1) - 0.5);
    }

    /**
     * The driver aimed the turret by hand at the real target: move the target point onto
     * that line, keeping the same distance from the robot, then go back to AUTO.
     */
    public void lockTargetHere(double x, double y, double heading) {
        double relative = (commandedAngle - Math.toRadians(TurretController.center_target_pos))
                / turretDirection;
        double world = heading + turretForwardRadians + relative;
        double sin = Math.sin(world);
        double along = Math.abs(sin) > 0.2 ? (targetY - y) / sin : -1;
        if (along > 0) {
            // The goal is on the wall at targetY: slide the target along that wall only.
            targetX = x + along * Math.cos(world);
        } else {
            double dist = Math.max(1, Math.hypot(targetX - x, targetY - y));
            targetX = x + dist * Math.cos(world);
            targetY = y + dist * Math.sin(world);
        }
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
        double desired = Math.toRadians(TurretController.center_target_pos) + turretDirection * relative;

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
