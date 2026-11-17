package org.firstinspires.ftc.teamcode.bobot.TestOpModes;

import com.pedropathing.ivy.Command;
import com.qualcomm.hardware.lynx.LynxModule;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import dev.nextftc.robot.opmode.NextTeleop;
import java.util.List;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.bobot.commands.CommandOpMode;
import org.firstinspires.ftc.teamcode.bobot.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.bobot.controllers.ShooterController;
import org.firstinspires.ftc.teamcode.bobot.controllers.TurretController;
import org.firstinspires.ftc.teamcode.bobot.states.SnapStateMachine;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.LegacyTest;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@NextTeleop(name = "test dont use", group = OpModeMeta.DefaultGroup)
public class LegacySnapTest extends CommandOpMode {
    private GamepadEx driver;
    private GamepadEx operator;
    private List<LynxModule> hubs;
    private MecanumDrive drive;
    private SnapStateMachine snap;
    private ShooterController shooter;
    private boolean intakeOn;

    @Override
    protected void onInit() {
        enablePanelsTelemetry();
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        Motor frontLeft = new Motor(hardwareMap, Constants.FRONT_LEFT);
        Motor frontRight = new Motor(hardwareMap, Constants.FRONT_RIGHT);
        Motor backLeft = new Motor(hardwareMap, Constants.BACK_LEFT);
        Motor backRight = new Motor(hardwareMap, Constants.BACK_RIGHT);
        frontRight.setInverted(true);
        backRight.setInverted(true);
        for (Motor motor : new Motor[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            motor.set(0);
        }
        drive = new MecanumDrive(false, frontLeft, frontRight, backLeft, backRight);

        shooter = new ShooterController(hardwareMap);
        snap = new SnapStateMachine(shooter, new IntakeController(hardwareMap),
                new TurretController(hardwareMap));
        snap.turretForwardRadians = LegacyTest.TURRET_FORWARD_RADIANS;
        snap.turretDirection = LegacyTest.TURRET_DIRECTION;

        robotTelemetry.addData("Turret mode", "90-degree snap");
        robotTelemetry.addData("Flywheels", "Encoder velocity PID");
        robotTelemetry.addData("Stopper Closed Pos", ShooterController.blockedPosition);
        robotTelemetry.addData("Stopper Feed Pos", ShooterController.feedingPosition);
        robotTelemetry.update();
    }

    @Override
    protected Command createCommand() {
        return Command.build()
                .setExecute(this::applyDriverControls)
                .requiring(drive, snap);
    }

    private void applyDriverControls() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        driver.readButtons();
        operator.readButtons();
        if (driver.wasJustPressed(GamepadKeys.Button.X)) {
            intakeOn = !intakeOn;
        }

        if (gamepad1.a) {
            intakeOn = false;
            drive.stop();
            snap.cancelAll();
        } else {
            driveRobotCentric();
            updateTurret();
            updateMechanisms();
            // This mode used an unconfigured follower and always used the starting pose.
            snap.update(LegacyTest.START_X, LegacyTest.START_Y, LegacyTest.START_HEADING);
        }

        snap.debug(robotTelemetry);
        robotTelemetry.addData("Intake toggle", intakeOn);
        robotTelemetry.addData("Localization", "Not configured");
        robotTelemetry.update();
    }

    private void driveRobotCentric() {
        double multiplier = driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
                > LegacyTest.SLOW_TRIGGER_THRESHOLD ? LegacyTest.SLOW_POWER : 1;
        drive.driveRobotCentric(driver.getLeftX() * multiplier,
                driver.getLeftY() * multiplier, driver.getRightX() * multiplier);
    }

    private void updateTurret() {
        if (operator.wasJustPressed(GamepadKeys.Button.X)) {
            snap.centerTurret();
        }
        if (operator.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            snap.rotateCounterclockwise90();
        }
        if (operator.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            snap.rotateClockwise90();
        }
    }

    private void updateMechanisms() {
        if (gamepad1.b) {
            snap.reverseIntake();
        } else if (gamepad1.y) {
            snap.spinUp();
            snap.requestFeed(true);
        } else if (intakeOn) {
            snap.startIntaking();
        } else {
            snap.cancelAll();
        }
    }

    @Override
    protected void onStop() {
        if (drive != null) {
            drive.stop();
        }
        if (snap != null) {
            snap.cancelAll();
        }
        if (shooter != null) {
            shooter.forceStopFlywheels();
        }
    }
}
