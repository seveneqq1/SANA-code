package org.firstinspires.ftc.teamcode.bobot.states;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bobot.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.bobot.controllers.ShooterController;
import org.firstinspires.ftc.teamcode.bobot.controllers.TurretController;

/** Shared intake and feed sequence. Subclasses supply turret aiming. */
@Configurable
public abstract class ShootingStateMachine {
    public enum State { IDLE, INTAKING, REVERSING, SPINNING_UP, READY, FEEDING }
    protected final ShooterController shooter;
    protected final IntakeController intake;
    protected final TurretController turret;
    private State state = State.IDLE;
    // First seconds of feeding: transfer at FULL power so balls start moving instantly, then FEEDING speed.
    public static double FEED_KICK_TIME = 0; // 0 = no full-power kick (old behavior)
    // Wait after opening the stopper so it fully opens before balls are pushed (no jam).
    public static double STOPPER_OPEN_DELAY = 0.15;
    private final ElapsedTime feedTimer = new ElapsedTime();
    private boolean spinning, feedingRequested;
    protected double commandedAngle;
    protected double targetX = 60, targetY = 60, distance;

    public double turretForwardRadians = 0;
    public double turretDirection = 1;

    protected ShootingStateMachine(ShooterController shooter,
                                   IntakeController intake, TurretController turret) {
        this.shooter = shooter;
        this.intake = intake;
        this.turret = turret;
        commandedAngle = Math.toRadians(TurretController.center_target_pos);
        turret.setTargetPos(Math.toDegrees(commandedAngle));
        turret.update();
        cancelAll();
    }

    public void startIntaking() {
        cancelAll();
        state = State.INTAKING;
        intake.setState(IntakeController.State.FULL_POWER);
    }

    public void reverseIntake() {
        cancelAll();
        state = State.REVERSING;
        intake.setState(IntakeController.State.BALLS_OUT);
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

    public void cancelAll() {
        spinning = false;
        feedingRequested = false;
        shooter.closeStopper();
        shooter.stop();
        intake.stop();
        state = State.IDLE;
    }

    public State getState() { return state; }
    public double getDistanceInches() { return distance; }
    public double getTurretAngleRadians() { return commandedAngle; }
    public abstract void centerTurret();
    protected abstract void updateAim(double x, double y, double heading);

    protected static double wrap(double angle) { return Math.atan2(Math.sin(angle), Math.cos(angle)); }

    protected boolean commandTurret(double angle) {
        double degrees = Math.toDegrees(angle);
        double servoPosition = TurretController.center_pos
                + (degrees - TurretController.center_target_pos) * TurretController.kV;
        if (!Double.isFinite(angle) || degrees < TurretController.min_target_pos
                || degrees > TurretController.max_target_pos
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
        shooter.setTargetVelocity(ShooterController.baseVelocity);
        shooter.update();
        turret.update();

        if (!spinning) return;

        boolean ready = shooter.getTargetVelocity() > 0 && shooter.isAtTarget();
        // once feeding started, keep stopper open until shooting ends (speed dips from balls don't close it)
        if (feedingRequested && (ready || state == State.FEEDING)) {
            if (state != State.FEEDING) feedTimer.reset(); // feeding just started
            state = State.FEEDING;
            shooter.feedStopper(); // stopper open
            double t = feedTimer.seconds();
            if (t < STOPPER_OPEN_DELAY) intake.stop(); // stopper still opening
            else intake.setState(t < STOPPER_OPEN_DELAY + FEED_KICK_TIME
                    ? IntakeController.State.FULL_POWER : IntakeController.State.FEEDING);
        } else {
            state = ready ? State.READY : State.SPINNING_UP;
            shooter.closeStopper(); // stopper closed
            intake.stop();
        }
    }

    public void debug(Telemetry telemetry) {
        telemetry.addData("Shooting state", state);
        telemetry.addData("Target inches", "%.1f, %.1f", targetX, targetY);
        telemetry.addData("Distance inches", distance);
        shooter.debug(telemetry);
        telemetry.addData("Turret command radians", commandedAngle);
    }
}
