package org.firstinspires.ftc.teamcode.bobot.compOpModes;

import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import dev.nextftc.robot.opmode.NextTeleop;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.bobot.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.bobot.controllers.TurretController;
import org.firstinspires.ftc.teamcode.bobot.states.TrackingStateMachine;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Tracking;

@NextTeleop(name = "dont use this shit", group = "Regional A")
public class RegionalATrackingTeleOp extends HeadingTeleOp<TrackingStateMachine> {
    public RegionalATrackingTeleOp() {
        super(true);
    }

    @Override
    protected TrackingStateMachine createMachine(IntakeController intake, TurretController turret) {
        TrackingStateMachine tracking = new TrackingStateMachine(shooter, intake, turret);
        tracking.turretDirection = Tracking.TURRET_DIRECTION;
        return tracking;
    }

    @Override
    protected void configureMode() {
        initializeHeading(Tracking.HEADING_KP, Tracking.HEADING_KD);
        robotTelemetry.addLine("Pinpoint full-pose turret tracking; fixed shooter TPS.");
        robotTelemetry.addLine("Requires calibrated Pinpoint pose in field inches, +X zero, CCW heading.");
        robotTelemetry.addLine("G1: RT intake, LT eject, hold Y shoot, A stop, LB slow");
        robotTelemetry.addLine("G1: D-pad right resets Pinpoint to START_X/Y/HEADING");
        robotTelemetry.addLine("G1: X/B heading left/right 90 degrees; right stick cancels hold");
        robotTelemetry.addLine("G2: D-pad up locks heading, LB/RB trim +/-5 deg, X tracks, Y centers");
    }

    @Override
    protected Pose2D startPose() {
        return new Pose2D(DistanceUnit.INCH, Tracking.START_X, Tracking.START_Y,
                AngleUnit.RADIANS, Tracking.START_HEADING);
    }

    @Override
    protected double slowPower() {
        return Tracking.SLOW_POWER;
    }

    @Override
    protected double rightStickDeadband() {
        return Tracking.RIGHT_STICK_DEADBAND;
    }

    @Override
    protected void configureHeadingGains() {
        tuneHeading(Tracking.HEADING_KP, Tracking.HEADING_KD, Tracking.HEADING_MAX_POWER,
                Tracking.HEADING_TOLERANCE_RADIANS);
    }

    @Override
    protected void aim() {
        if (operator.wasJustPressed(GamepadKeys.Button.Y)) {
            machine.centerTurret();
        } else if (operator.wasJustPressed(GamepadKeys.Button.X)) {
            machine.setTrackingEnabled(true);
        } else if (!gamepad2.left_bumper && operator.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            machine.adjustAimTrim(Math.toRadians(-Tracking.AIM_TRIM_DEGREES));
        } else if (!gamepad2.right_bumper && operator.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            machine.adjustAimTrim(Math.toRadians(Tracking.AIM_TRIM_DEGREES));
        }
        machine.turretDirection = Tracking.TURRET_DIRECTION;
    }

    @Override
    protected void reportAim() {
        robotTelemetry.addData("Position inches", "%.1f, %.1f", xInches, yInches);
        robotTelemetry.addData("Distance to target inches", machine.getDistanceInches());
        robotTelemetry.addData("Aim trim degrees", Math.toDegrees(machine.aimTrimRadians));
        robotTelemetry.addData("Tracking enabled", machine.isTrackingEnabled());
    }
}
