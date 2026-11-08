package org.firstinspires.ftc.teamcode.bobot.controller;

/**
 * Driver snaps the turret by 90 degrees. No automatic aim.
 */

public  class SnapStateMachine extends ShootingStateMachine {
    private boolean previousClockwise, previousCounterclockwise;

    public SnapStateMachine(shooterController shooter, intakeController intake,
            turretController turret, ShotTable table) {
        super(shooter, intake, turret, table);
    }
    public boolean rotateClockwise90() {
        return commandTurret(commandedAngle - turretDirection * Math.PI / 2);
    }
    public boolean rotateCounterclockwise90() {
        return commandTurret(commandedAngle + turretDirection * Math.PI / 2);
    }
    public void handleRotationButtons(boolean clockwise, boolean counterclockwise) {
        if (clockwise && !previousClockwise && !counterclockwise) rotateClockwise90();
        if (counterclockwise && !previousCounterclockwise && !clockwise) rotateCounterclockwise90();
        previousClockwise = clockwise;
        previousCounterclockwise = counterclockwise;
    }
    @Override public void centerTurret() {
        commandTurret(Math.toRadians(turretController.center_target_pos));
    }
    @Override protected void updateAim(double x, double y, double heading) {
        selectQuarter(x, y);
        // Snap mode leaves aiming to the driver; no automatic target alignment gate.
        // Pose still supplies distance to the selected inner tile for the shooter LUT.
    }
}
