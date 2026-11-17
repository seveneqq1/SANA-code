package org.firstinspires.ftc.teamcode.bobot.compOpModes;

import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import dev.nextftc.robot.opmode.NextTeleop;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.bobot.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.bobot.controllers.TurretController;
import org.firstinspires.ftc.teamcode.bobot.states.SnapStateMachine;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Snap;

@NextTeleop(name = "Snap", group = "Regional A")
public class RegionalATeleOp extends HeadingTeleOp<SnapStateMachine> {
    public RegionalATeleOp() {
        super(false);
    }

    @Override
    protected SnapStateMachine createMachine(IntakeController intake, TurretController turret) {
        SnapStateMachine snap = new SnapStateMachine(shooter, intake, turret);
        snap.turretDirection = Snap.TURRET_DIRECTION;
        return snap;
    }

    @Override
    protected void configureMode() {
        initializeHeading(Snap.HEADING_KP, Snap.HEADING_KD);
        robotTelemetry.addLine("Pinpoint heading hold + turret snap; fixed shooter TPS.");
        robotTelemetry.addLine("G1: RT intake, LT eject, hold Y shoot, A stop, LB slow");
        robotTelemetry.addLine("G1: D-pad right resets Pinpoint to START_X/Y/HEADING");
        robotTelemetry.addLine("G1: X/B heading left/right 90 degrees; right stick cancels hold");
        robotTelemetry.addLine("G2: D-pad up locks heading, LB/RB turret CCW/CW, X centers turret");
    }

    @Override
    protected Pose2D startPose() {
        return new Pose2D(DistanceUnit.INCH, Snap.START_X, Snap.START_Y,
                AngleUnit.RADIANS, Snap.START_HEADING);
    }

    @Override
    protected double slowPower() {
        return Snap.SLOW_POWER;
    }

    @Override
    protected double rightStickDeadband() {
        return Snap.RIGHT_STICK_DEADBAND;
    }

    @Override
    protected void configureHeadingGains() {
        tuneHeading(Snap.HEADING_KP, Snap.HEADING_KD, Snap.HEADING_MAX_POWER,
                Snap.HEADING_TOLERANCE_RADIANS);
    }

    @Override
    protected void aim() {
        if (operator.wasJustPressed(GamepadKeys.Button.X)) {
            machine.centerTurret();
        } else if (!gamepad2.left_bumper && operator.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            machine.rotateClockwise90();
        } else if (!gamepad2.right_bumper && operator.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            machine.rotateCounterclockwise90();
        }
    }

}
