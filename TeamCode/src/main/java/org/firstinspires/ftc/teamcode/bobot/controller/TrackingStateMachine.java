package org.firstinspires.ftc.teamcode.bobot.controller;

/**
 * Automatically aims the turret at the inner tile of the current field quarter.
 */

public class TrackingStateMachine extends ShootingStateMachine {
    public double aimTrimRadians;
    private boolean trackingEnabled = true;

    public TrackingStateMachine(shooterController_var2 shooter, intakeController intake,
            turretController turret, ShotTable table) {
        super(shooter, intake, turret, table);
    }

    public void setTrackingEnabled(boolean enabled) { trackingEnabled = enabled; }
    public void adjustAimTrim(double radians) {
        aimTrimRadians += radians;
    }
    @Override public void centerTurret() {
        trackingEnabled = false;
        commandTurret(Math.toRadians(turretController.center_target_pos));
    }

    @Override protected void updateAim(double x, double y, double heading) {
        double dx = targetX - x, dy = targetY - y;
        if (!trackingEnabled || (dx == 0 && dy == 0)) return;
        double relative = wrap(Math.atan2(dy, dx) - heading - turretForwardRadians + aimTrimRadians);
        double center = Math.toRadians(turretController.center_target_pos);
        double desired = center + turretDirection * relative;
        double best = Double.NaN, shortest = Double.POSITIVE_INFINITY;
        for (int turns = -2; turns <= 2; turns++) {
            double candidate = desired + turns * 2 * Math.PI;
            double degrees = Math.toDegrees(candidate);
            double servo = turretController.center_pos
                    + (degrees - turretController.center_target_pos) * turretController.kV;
            if (degrees >= turretController.min_target_pos && degrees <= turretController.max_target_pos
                    && servo >= 0 && servo <= 1 && Math.abs(candidate - commandedAngle) < shortest) {
                best = candidate;
                shortest = Math.abs(candidate - commandedAngle);
            }
        }
        if (!Double.isNaN(best)) commandTurret(best);
    }
}
