package org.firstinspires.ftc.teamcode.bobot.compOpModes;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.bobot.controllers.HeadingController;
import org.firstinspires.ftc.teamcode.bobot.states.ShootingStateMachine;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.TRIGGER_DEADBAND;


abstract class HeadingTeleOp<M extends ShootingStateMachine> extends CompetitionTeleOp<M> {
    protected GamepadEx operator;
    protected double xInches;
    protected double yInches;
    protected double currentHeading;

    private final boolean usesFullPose;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private HeadingController heading;
    private double dtSeconds;
    private double appliedHeadingKP;
    private double appliedHeadingKD;
    private int poseResetCount;
    private boolean halted;

    HeadingTeleOp(boolean usesFullPose) {
        this.usesFullPose = usesFullPose;
    }

    protected final void initializeHeading(double kP, double kD) {
        operator = new GamepadEx(gamepad2);
        heading = new HeadingController(kP, kD);
        appliedHeadingKP = kP;
        appliedHeadingKD = kD;
    }

    protected abstract Pose2D startPose();

    protected abstract double slowPower();

    protected abstract double rightStickDeadband();

    protected abstract void configureHeadingGains();

    protected abstract void aim();

    protected void reportAim() {}

    @Override
    protected final void onStart() {
        heading.disableHold();
        loopTimer.reset();
    }

    @Override
    protected final Command[] createCommands() {
        // Register in sensor → drive → scoring → telemetry order. Each actuator has one owner.
        return new Command[]{
                Commands.infinite(this::readPose).requiring(pinpoint),
                Commands.infinite(this::driveWithHeadingHold).requiring(drive)
                        .setEnd(reason -> stopDrive()),
                Commands.infinite(this::score).requiring(machine)
                        .setEnd(reason -> stopScoring()),
                Commands.infinite(this::report)
        };
    }

    private void readPose() {
        readDriverFrame();
        operator.readButtons();
        dtSeconds = loopTimer.seconds();
        loopTimer.reset();

        if (!gamepad1.a && driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            pinpoint.setPosition(startPose());
            heading.disableHold();
            poseResetCount++;
        }
        if (usesFullPose) {
            pinpoint.update();
        } else {
            pinpoint.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);
        }
        currentHeading = pinpoint.getHeading(AngleUnit.RADIANS);
        if (usesFullPose) {
            xInches = pinpoint.getPosX(DistanceUnit.INCH);
            yInches = pinpoint.getPosY(DistanceUnit.INCH);
        }
        halted = gamepad1.a;
    }

    private void driveWithHeadingHold() {
        if (halted) {
            heading.disableHold();
            drive.stop();
            return;
        }

        boolean manualOverride = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y)
                > rightStickDeadband();
        boolean lockPressed = operator.wasJustPressed(GamepadKeys.Button.DPAD_UP);
        if (heading.isHoldingHeading() && !manualOverride && !lockPressed) {
            if (!gamepad1.b && driver.wasJustPressed(GamepadKeys.Button.X)) {
                heading.snapTargetHeading(HeadingController.Direction.ANTICLOCKWISE);
            } else if (!gamepad1.x && driver.wasJustPressed(GamepadKeys.Button.B)) {
                heading.snapTargetHeading(HeadingController.Direction.CLOCKWISE);
            }
        }
        configureHeadingGains();
        double turnCCW = heading.updateHold(currentHeading, dtSeconds, lockPressed, manualOverride);
        double turnPower = heading.isHoldingHeading() ? -turnCCW : driver.getRightX();
        driveSticks(turnPower, gamepad1.left_bumper ? slowPower() : 1);
    }

    protected final void tuneHeading(double kP, double kD, double maxPower, double tolerance) {
        if (kP != appliedHeadingKP || kD != appliedHeadingKD) {
            heading.setGains(kP, kD);
            appliedHeadingKP = kP;
            appliedHeadingKD = kD;
        }
        heading.setMaxPower(maxPower);
        heading.setTolerance(tolerance);
    }

    private void score() {
        if (halted) {
            machine.cancelAll();
            return;
        }

        aim();
        if (driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > TRIGGER_DEADBAND) {
            machine.reverseIntake();
        } else if (gamepad1.y) {
            machine.spinUp();
            machine.requestFeed(true);
        } else if (driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_DEADBAND) {
            machine.startIntaking();
        } else {
            machine.cancelAll();
        }
        machine.update(xInches, yInches, currentHeading);
    }

    private void report() {
        reportAim();
        robotTelemetry.addData("Shooting state", machine.getState());
        robotTelemetry.addData("Pose resets", poseResetCount);
        robotTelemetry.addData("Heading hold", heading.isHoldingHeading());
        robotTelemetry.addData("Heading degrees", Math.toDegrees(currentHeading));
        robotTelemetry.addData("Heading target degrees", heading.isHoldingHeading()
                ? Math.toDegrees(heading.getTargetHeading()) : "Manual");
        shooter.debug(robotTelemetry);
        robotTelemetry.addData("Turret command degrees", Math.toDegrees(machine.getTurretAngleRadians()));
        robotTelemetry.update();
    }

    @Override
    protected final void onStop() {
        if (heading != null) heading.disableHold();
        super.onStop();
    }
}
