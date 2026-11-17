package org.firstinspires.ftc.teamcode.bobot.compOpModes;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import dev.nextftc.robot.opmode.NextTeleop;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.bobot.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.bobot.controllers.TurretController;
import org.firstinspires.ftc.teamcode.bobot.states.FixedPointTrackingMachine;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.PointTrack;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.TRIGGER_DEADBAND;


@NextTeleop(name = "Point Track", group = "Regional A")
public class RegionalAPointTrackTeleOp extends CompetitionTeleOp<FixedPointTrackingMachine> {
    private enum Mode { IDLE, INTAKE, OUTTAKE, SHOOT }

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime feedTimer = new ElapsedTime();
    private Mode mode = Mode.IDLE;
    private Mode lastMode;
    private boolean feeding;
    private boolean wasManual;
    private boolean manualNow;
    private boolean trackingOn = true;

    private double dtSeconds;
    private double x;
    private double y;
    private double heading;
    private boolean resetRequested;

    @Override
    protected FixedPointTrackingMachine createMachine(IntakeController intake, TurretController turret) {
        FixedPointTrackingMachine tracking = new FixedPointTrackingMachine(shooter, intake, turret);
        tracking.setTarget(PointTrack.TARGET_X, PointTrack.TARGET_Y);
        return tracking;
    }

    @Override
    protected void configureMode() {
        pinpoint.setOffsets(PointTrack.X_POD_OFFSET_MM, PointTrack.Y_POD_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(PointTrack.TICKS_PER_MM, DistanceUnit.MM);
        pinpoint.setEncoderDirections(direction(PointTrack.REVERSE_X_POD), direction(PointTrack.REVERSE_Y_POD));
        pinpoint.resetPosAndIMU();
        robotTelemetry.addLine("Put the robot in the BOTTOM-RIGHT corner, front away from you.");
    }

    private static GoBildaPinpointDriver.EncoderDirection direction(boolean reversed) {
        return reversed ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                : GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }

    @Override
    protected void onStart() {
        resetPose();
        resetAim();
        loopTimer.reset();
    }

    @Override
    protected Command[] createCommands() {
        return new Command[]{
                Commands.infinite(this::readPose).requiring(pinpoint),
                Commands.infinite(this::applyDriverControls).requiring(machine, drive)
                        .setEnd(reason -> stopRobot()),
                Commands.infinite(this::report)
        };
    }

    private void readPose() {
        readDriverFrame();
        dtSeconds = Math.min(loopTimer.seconds(), PointTrack.MAX_LOOP_SECONDS);
        loopTimer.reset();
        resetRequested = driver.wasJustPressed(GamepadKeys.Button.DPAD_UP);
        if (resetRequested) {
            resetPose();
        }
        pinpoint.update();
        x = pinpoint.getPosX(DistanceUnit.INCH);
        y = pinpoint.getPosY(DistanceUnit.INCH);
        heading = pinpoint.getHeading(AngleUnit.RADIANS);
    }

    private void applyDriverControls() {
        if (resetRequested) {
            resetAim();
        }
        machine.turretDirection = PointTrack.TURRET_DIRECTION;
        aim();
        selectMode();
        // The readiness check in selectMode uses the preceding shooter update.
        driveWithBrake();
        machine.update(x, y, heading);
    }

    private void aim() {
        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            trackingOn = !trackingOn;
            if (trackingOn) {
                machine.setTrackingEnabled(true);
            } else {
                machine.centerTurret();
            }
        }

        boolean left = gamepad1.left_bumper;
        boolean right = gamepad1.right_bumper;
        manualNow = left != right;
        if (manualNow) {
            double step = PointTrack.MANUAL_SPEED_DEG_PER_S * dtSeconds;
            machine.manualMove((left ? step : -step) * PointTrack.TURRET_DIRECTION);
        }
        if (wasManual && !manualNow && trackingOn) {
            machine.lockTargetHere(x, y, heading);
        }
        wasManual = manualNow;
    }

    private void selectMode() {
        if (driver.wasJustPressed(GamepadKeys.Button.X)) mode = Mode.INTAKE;
        if (driver.wasJustPressed(GamepadKeys.Button.B)) mode = Mode.OUTTAKE;
        if (driver.wasJustPressed(GamepadKeys.Button.Y)) {
            mode = Mode.SHOOT;
            feeding = false;
        }
        if (driver.wasJustPressed(GamepadKeys.Button.A)) mode = Mode.IDLE;

        shooter.boost = PointTrack.ALWAYS_SPIN || mode == Mode.SHOOT;
        boolean modeChanged = mode != lastMode;
        lastMode = mode;
        switch (mode) {
            case INTAKE:
                if (modeChanged) machine.startIntaking();
                break;
            case OUTTAKE:
                if (modeChanged) machine.reverseIntake();
                break;
            case SHOOT:
                machine.spinUp();
                machine.requestFeed(true);
                if (!feeding && shooter.isAtTarget()) {
                    feeding = true;
                    feedTimer.reset();
                }
                if (feeding && feedTimer.seconds() > PointTrack.SHOOT_TIME) {
                    mode = Mode.INTAKE;
                }
                break;
            case IDLE:
                if (modeChanged) machine.cancelAll();
                break;
        }
    }

    private void driveWithBrake() {
        if (driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > TRIGGER_DEADBAND) {
            drive.stop();
        } else {
            double scale = driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_DEADBAND
                    ? PointTrack.SLOW_MULTIPLIER : 1;
            driveSticks(driver.getRightX(), scale);
        }
    }

    private void report() {
        robotTelemetry.addData("Mode", mode);
        robotTelemetry.addData("Turret", !trackingOn ? "OFF (forward)" : manualNow ? "MANUAL" : "TRACKING");
        robotTelemetry.addData("Robot X / Y", "%.1f / %.1f", x, y);
        robotTelemetry.addData("Heading degrees", "%.1f", Math.toDegrees(heading));
        robotTelemetry.addData("Target X / Y", "%.1f / %.1f", machine.getTargetX(), machine.getTargetY());
        robotTelemetry.addData("Distance to target", "%.1f", machine.getDistanceInches());
        robotTelemetry.addData("Turret OUT OF RANGE", machine.isOutOfRange());
        robotTelemetry.addData("Pinpoint status", pinpoint.getDeviceStatus());
        shooter.debug(robotTelemetry);
        robotTelemetry.update();
    }

    private void resetPose() {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,
                PointTrack.FIELD_SIZE_INCHES - PointTrack.ROBOT_WIDTH / 2, PointTrack.ROBOT_LENGTH / 2,
                AngleUnit.DEGREES, PointTrack.RESET_HEADING_DEGREES));
    }

    private void resetAim() {
        machine.setTarget(PointTrack.TARGET_X, PointTrack.TARGET_Y);
        trackingOn = true;
        wasManual = false;
        machine.setTrackingEnabled(true);
    }
}
