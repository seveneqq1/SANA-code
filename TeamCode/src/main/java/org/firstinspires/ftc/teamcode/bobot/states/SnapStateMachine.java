package org.firstinspires.ftc.teamcode.bobot.states;

import org.firstinspires.ftc.teamcode.bobot.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.bobot.controllers.ShooterController;
import org.firstinspires.ftc.teamcode.bobot.controllers.TurretController;

/**
 * Driver snaps the turret by 90 degrees. No automatic aim.
 */

public class SnapStateMachine extends ShootingStateMachine {
    public SnapStateMachine(ShooterController shooter, IntakeController intake,
            TurretController turret) {
        super(shooter, intake, turret);
    }
    public boolean rotateClockwise90() {
        return commandTurret(commandedAngle - turretDirection * Math.PI / 2);
    }
    public boolean rotateCounterclockwise90() {
        return commandTurret(commandedAngle + turretDirection * Math.PI / 2);
    }
    @Override public void centerTurret() {
        commandTurret(Math.toRadians(TurretController.center_target_pos));
    }
    @Override protected void updateAim(double x, double y, double heading) {
        // Snap mode leaves aiming to the driver.
    }
}
