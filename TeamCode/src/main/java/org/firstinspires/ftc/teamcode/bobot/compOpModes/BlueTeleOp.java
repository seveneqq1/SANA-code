package org.firstinspires.ftc.teamcode.bobot.compOpModes;

import com.pedropathing.ivy.Command;
import com.qualcomm.hardware.lynx.LynxModule;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import dev.nextftc.robot.opmode.NextTeleop;
import java.util.List;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.bobot.commands.CommandOpMode;
import org.firstinspires.ftc.teamcode.bobot.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.bobot.controllers.ShooterController;
import org.firstinspires.ftc.teamcode.bobot.controllers.TurretController;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@NextTeleop(name = "dont usee", group = OpModeMeta.DefaultGroup)
public class BlueTeleOp extends CommandOpMode {
    private GamepadEx driver;
    private List<LynxModule> hubs;
    private IntakeController intake;
    private ShooterController shooter;
    private MecanumDrive drive;

    @Override
    protected void onInit() {
        intake = new IntakeController(hardwareMap);
        shooter = new ShooterController(hardwareMap);
        // The original mode applies the turret servo directions during INIT.
        new TurretController(hardwareMap);
        driver = new GamepadEx(gamepad1);

        Motor frontLeft = new Motor(hardwareMap, Constants.FRONT_LEFT);
        Motor frontRight = new Motor(hardwareMap, Constants.FRONT_RIGHT);
        Motor backLeft = new Motor(hardwareMap, Constants.BACK_LEFT);
        Motor backRight = new Motor(hardwareMap, Constants.BACK_RIGHT);
        frontLeft.setInverted(true);
        backLeft.setInverted(true);
        for (Motor motor : new Motor[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        }
        drive = new MecanumDrive(false, frontLeft, frontRight, backLeft, backRight);

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    protected Command createCommand() {
        return Command.build()
                .setExecute(this::driveAndSpinShooter)
                .requiring(drive, shooter);
    }

    private void driveAndSpinShooter() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        shooter.setTargetVelocity(ShooterController.baseVelocity);
        shooter.update();
        shooter.debug(robotTelemetry);
        robotTelemetry.update();
        drive.driveRobotCentric(driver.getLeftX(), driver.getLeftY(), driver.getRightX());
    }

    @Override
    protected void onStop() {
        if (drive != null) {
            drive.stop();
        }
        if (intake != null) {
            intake.stop();
        }
        if (shooter != null) {
            shooter.forceStopFlywheels();
        }
    }
}
