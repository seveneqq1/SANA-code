package org.firstinspires.ftc.teamcode.bobot.compOpModes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import java.util.List;
import org.firstinspires.ftc.teamcode.bobot.commands.CommandOpMode;
import org.firstinspires.ftc.teamcode.bobot.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.bobot.controllers.ShooterController;
import org.firstinspires.ftc.teamcode.bobot.controllers.TurretController;
import org.firstinspires.ftc.teamcode.bobot.states.ShootingStateMachine;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

abstract class CompetitionTeleOp<M extends ShootingStateMachine> extends CommandOpMode {
    protected GamepadEx driver;
    protected MecanumDrive drive;
    protected ShooterController shooter;
    protected M machine;
    protected GoBildaPinpointDriver pinpoint;
    private List<LynxModule> hubs;

    @Override
    protected final void onInit() {
        enablePanelsTelemetry();
        driver = new GamepadEx(gamepad1);
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
        machine = createMachine(new IntakeController(hardwareMap), new TurretController(hardwareMap));
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.PINPOINT_NAME);
        configureMode();
        robotTelemetry.update();
    }

    protected abstract M createMachine(IntakeController intake, TurretController turret);

    protected abstract void configureMode();

    protected final void readDriverFrame() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        driver.readButtons();
    }

    protected final void driveSticks(double turn, double scale) {
        drive.driveRobotCentric(driver.getLeftX() * scale, driver.getLeftY() * scale, turn * scale);
    }

    @Override
    protected void onStop() {
        stopRobot();
    }

    protected final void stopRobot() {
        try {
            stopDrive();
        } finally {
            stopScoring();
        }
    }

    protected final void stopDrive() {
        if (drive != null) drive.stop();
    }

    protected final void stopScoring() {
        try {
            if (machine != null) machine.cancelAll();
        } finally {
            if (shooter != null) shooter.forceStopFlywheels();
        }
    }
}
