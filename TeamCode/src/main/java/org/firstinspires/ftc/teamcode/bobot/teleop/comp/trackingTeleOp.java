package org.firstinspires.ftc.teamcode.bobot.teleop.comp;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.bobot.subsystem.Turret;
import org.firstinspires.ftc.teamcode.bobot.teleop.BekaBase;

@Config
@TeleOp(name = "Beka teleop")
public class trackingTeleOp extends BekaBase {
    public static double targetX = 110, targetY = 63;
    public static double slowMultiplier = 0.6;
    public static double manualTurretDegreesPerSecond = 120;
    public static double shooterTolerance = 120;
    public static double stopperOpenDelay = 0.15;
    public static double feedKickTime = 0;
    public static double shootTime = 1.5;

    private enum Mode { IDLE, INTAKE, OUTTAKE, SHOOT }

    private final ElapsedTime feedTimer = new ElapsedTime();
    private Mode mode = Mode.IDLE;
    private Pose target;
    private boolean tracking = true;
    private boolean manualLastLoop;
    private boolean feeding;
    private double previousHeading;

    @Override
    protected void onStart() {
        r.turnOn();
        initializeTracking();
    }

    @Override
    protected void onUpdate() {
        if (pressed(GamepadKeys.Button.DPAD_UP)) resetTracking();
        Pose pose = r.drive.getPose();
        double headingRate = deltaTime > 0
                ? Turret.normalizeAngle(pose.heading() - previousHeading) / deltaTime : 0;
        previousHeading = pose.heading();
        updateTurret(pose, headingRate);

        double velocity = Math.abs(r.shooter.getVel());
        double shooterTarget = r.shooter.getTarget();
        updateIntake(velocity, shooterTarget);
        updateDrive();

        telemetry.addData("Pose", pose);
        telemetry.addData("Goal", target);
        telemetry.addData("Tracking", tracking);
        telemetry.addData("Mode", mode);
        telemetry.addData("Shooter TPS", "%.0f / %.0f", velocity, shooterTarget);
        telemetry.update();
    }

    private void resetTracking() {
        r.drive.resetPos();
        initializeTracking();
    }

    private void initializeTracking() {
        target = new Pose(targetX, targetY, 0);
        previousHeading = r.drive.getPose().heading();
        tracking = true;
        manualLastLoop = false;
    }

    private void updateDrive() {
        if (first_driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) {
            r.drive.drive(0, 0, 0);
            return;
        }
        double scale = first_driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2
                ? slowMultiplier : 1.0;
        r.drive.drive(first_driver.getLeftY() * scale, first_driver.getLeftX() * scale,
                first_driver.getRightX() * scale);
    }

    private void updateTurret(Pose pose, double headingRate) {
        if (pressed(GamepadKeys.Button.DPAD_LEFT)) {
            tracking = !tracking;
            if (!tracking) r.turret.setYaw(0);
        }

        boolean left = gamepad1.left_bumper;
        boolean right = gamepad1.right_bumper;
        boolean manual = left != right;
        if (manual) {
            double step = Math.toRadians(manualTurretDegreesPerSecond * Math.min(deltaTime, 0.1));
            r.turret.addYaw(left ? step : -step);
        } else if (tracking) {
            if (manualLastLoop) lockTargetHere(pose);
            r.turret.face(target, pose, headingRate);
        }
        manualLastLoop = manual;
    }

    private void lockTargetHere(Pose pose) {
        double angle = pose.heading() + r.turret.getYaw();
        double sin = Math.sin(angle);
        double distance = Math.abs(sin) > 0.2 ? (target.y() - pose.y()) / sin : -1;
        if (distance > 0) {
            target = new Pose(pose.x() + distance * Math.cos(angle), target.y(), 0);
        } else {
            distance = Math.max(1, Math.hypot(target.x() - pose.x(), target.y() - pose.y()));
            target = new Pose(pose.x() + distance * Math.cos(angle), pose.y() + distance * sin, 0);
        }
    }

    private void updateIntake(double velocity, double shooterTarget) {
        if (pressed(GamepadKeys.Button.X)) setMode(Mode.INTAKE);
        if (pressed(GamepadKeys.Button.B)) setMode(Mode.OUTTAKE);
        if (pressed(GamepadKeys.Button.A)) setMode(Mode.IDLE);
        if (pressed(GamepadKeys.Button.Y)) setMode(Mode.SHOOT);
        if (mode != Mode.SHOOT) return;

        if (!feeding) {
            if (shooterTarget <= 0 || Math.abs(velocity - shooterTarget) >= shooterTolerance) {
                return;
            }
            r.shooter.setFeeding(true);
            feedTimer.reset();
            feeding = true;
        }

        double elapsed = feedTimer.seconds();
        if (elapsed >= shootTime) setMode(Mode.INTAKE);
        else if (elapsed < stopperOpenDelay) r.intake.spinOff();
        else if (elapsed < stopperOpenDelay + feedKickTime) r.intake.spinShoot();
        else r.intake.spinSlowShoot();
    }

    private void setMode(Mode next) {
        mode = next;
        feeding = false;
        r.shooter.setFeeding(false);
        switch (mode) {
            case INTAKE: r.intake.spinIn(); break;
            case OUTTAKE: r.intake.spinOut(); break;
            default: r.intake.spinOff();
        }
    }

    private boolean pressed(GamepadKeys.Button button) {
        return first_driver.wasJustPressed(button);
    }
}
