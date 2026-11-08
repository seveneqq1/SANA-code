package org.firstinspires.ftc.teamcode.bobot.controller;

/**
 * Shared shoot / intake / feed sequence. Subclasses decide how the turret aims.
 */

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class ShootingStateMachine {
    public enum State { IDLE, INTAKING, REVERSING, SPINNING_UP, READY, FEEDING }
    protected final shooterController_var2 shooter;
    protected final intakeController intake;
    protected final turretController turret;
    private final ShotTable table;
    private State state = State.IDLE;
    private boolean spinning, feedingRequested;
    private double manualTPS = Double.NaN;
    private double speedTrim;
    protected double commandedAngle;
    protected double targetX = 60, targetY = 60, distance;

    public double turretForwardRadians = 0;
    public double turretDirection = 1;

    protected ShootingStateMachine(shooterController_var2 shooter,
                                   intakeController intake, turretController turret, ShotTable table) {
        this.shooter = shooter;
        this.intake = intake;
        this.turret = turret;
        this.table = table;
        commandedAngle = Math.toRadians(turretController.center_target_pos);
        turret.setTargetPos(Math.toDegrees(commandedAngle));
        turret.update();
        cancelAll();
    }

    public void startIntaking() {
        cancelAll();
        state = State.INTAKING;
        intake.setState(intakeController.State.FULL_POWER);
    }

    public void reverseIntake() {
        cancelAll();
        state = State.REVERSING;
        intake.setState(intakeController.State.BALLS_OUT);
    }

    public void stopIntaking() {
        intake.stop();
        if (!spinning) state = State.IDLE;
    }

    public void spinUp() {
        if (!spinning) {
            spinning = true;
            state = State.SPINNING_UP;
            intake.stop();
            shooter.closeStopper();
        }
    }

    public void requestFeed(boolean requested) {
        feedingRequested = requested;
    }

    public void stopShooting() {
        cancelAll();
    }

    public void cancelAll() {
        spinning = false;
        feedingRequested = false;
        shooter.closeStopper();
        shooter.stop();
        intake.stop();
        state = State.IDLE;
    }

    public void setManualTPS(double tps) {
        manualTPS = tps;
    }

    public void useAutomaticTPS() {
        manualTPS = Double.NaN;
    }

    public void setSpeedTrim(double tps) {
        speedTrim = tps;
    }

    public State getState() { return state; }
    public double getDistanceInches() { return distance; }
    public double getTurretAngleRadians() { return commandedAngle; }
    public abstract void centerTurret();
    protected abstract void updateAim(double x, double y, double heading);

    protected static double wrap(double angle) { return Math.atan2(Math.sin(angle), Math.cos(angle)); }

    protected boolean commandTurret(double angle) {
        double degrees = Math.toDegrees(angle);
        double servoPosition = turretController.center_pos
                + (degrees - turretController.center_target_pos) * turretController.kV;
        if (!Double.isFinite(angle) || degrees < turretController.min_target_pos
                || degrees > turretController.max_target_pos
                || servoPosition < 0 || servoPosition > 1) return false;
        commandedAngle = angle;
        turret.setTargetPos(degrees);
        return true;
    }

    protected void selectQuarter(double x, double y) {
        targetX = x < 72 ? 60 : 84;
        targetY = y < 72 ? 60 : 84;
    }

    public final void update(double x, double y, double heading) {
        selectQuarter(x, y);
        updateAim(x, y, heading);
        distance = Math.hypot(targetX - x, targetY - y);
        double tps = Double.isNaN(manualTPS)
                ? (table == null ? shooterController_var2.baseVelocity : table.getTPS(distance))
                : manualTPS;

        shooter.setTargetVelocity(spinning ? tps + speedTrim : shooterController_var2.baseVelocity);
        shooter.update();
        turret.update();

        if (!spinning) return;

        boolean ready = shooter.getTargetVelocity() > 0 && shooter.isAtTarget();
        if (ready && feedingRequested) {
            state = State.FEEDING;
            shooter.feedStopper(); // Stopper position -> 0.5
            intake.setState(intakeController.State.FEEDING);
        } else {
            state = ready ? State.READY : State.SPINNING_UP;
            shooter.closeStopper(); // Stopper position -> 0.05
            intake.stop();
        }
    }

    public void debug(Telemetry telemetry) {
        telemetry.addData("Shooting state", state);
        telemetry.addData("Target inches", "%.1f, %.1f", targetX, targetY);
        telemetry.addData("Distance inches", distance);
        telemetry.addData("Target TPS", shooter.getTargetVelocity());
        telemetry.addData("Measured TPS", shooter.getCurrentVelocity());
        telemetry.addData("Turret command radians", commandedAngle);
    }
}