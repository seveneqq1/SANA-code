package org.firstinspires.ftc.teamcode.bobot.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import java.util.List;
import org.firstinspires.ftc.teamcode.bobot.controller.*;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Config
@TeleOp(name = "test dont use")
public class teleop extends OpMode {

    public static double START_X = 0, START_Y = 0, START_HEADING = 0;
    public static double SLOW_POWER = 0.35;
    public static double TURRET_FORWARD_RADIANS = 0;
    public static double TURRET_DIRECTION = 1;

    private GamepadEx first_driver, second_driver;
    private List<LynxModule> allHubs;
    private Motor frontLeft, frontRight, backLeft, backRight;
    private MecanumDrive mecanumDrive;
    private Follower follower;

    private SnapStateMachine snap;
    private shooterController_var2 shooter;
    private boolean intakeOn;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        first_driver = new GamepadEx(gamepad1);
        second_driver = new GamepadEx(gamepad2);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        follower = Constants.create(hardwareMap);
        if (follower != null) {
            follower.setPose(new Pose(START_X, START_Y, START_HEADING));
        }

        frontLeft = new Motor(hardwareMap, "frontleft");
        frontRight = new Motor(hardwareMap, "frontright");
        backLeft = new Motor(hardwareMap, "backleft");
        backRight = new Motor(hardwareMap, "backright");

        frontRight.setInverted(true);
        backRight.setInverted(true);

        for (Motor motor : new Motor[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            motor.set(0);
        }
        mecanumDrive = new MecanumDrive(false, frontLeft, frontRight, backLeft, backRight);

        shooter = new shooterController_var2(hardwareMap);
        intakeController intake = new intakeController(hardwareMap);
        turretController turret = new turretController(hardwareMap);

        snap = new SnapStateMachine(shooter, intake, turret, null);
        snap.turretForwardRadians = TURRET_FORWARD_RADIANS;
        snap.turretDirection = TURRET_DIRECTION;

        telemetry.addData("Turret mode", "90-degree snap");
        telemetry.addData("Flywheels", "Encoder velocity PID");
        telemetry.addData("Stopper Closed Pos", shooterController_var2.blockedPosition);
        telemetry.addData("Stopper Feed Pos", shooterController_var2.feedingPosition);
        telemetry.update();
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        first_driver.readButtons();
        second_driver.readButtons();

        if (follower != null) {
            follower.update();
        }

        if (first_driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT) && follower != null) {
            follower.setPose(new Pose(START_X, START_Y, START_HEADING));
        }

        if (first_driver.wasJustPressed(GamepadKeys.Button.X)) intakeOn = !intakeOn;

        if (gamepad1.a) {
            intakeOn = false;
            mecanumDrive.stop();
            snap.cancelAll();
        } else {
            drive();

            if (second_driver.wasJustPressed(GamepadKeys.Button.X)) snap.centerTurret();
            if (second_driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) snap.rotateCounterclockwise90();
            if (second_driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) snap.rotateClockwise90();

            if (gamepad1.b) {
                snap.reverseIntake();
            } else if (gamepad1.y) {
                // Pressing Y opens stopper (0.5) to shoot/feed ring
                snap.spinUp();
                snap.requestFeed(true);
            } else if (intakeOn) {
                snap.startIntaking();
            } else {
                snap.cancelAll();
            }
        }

        Pose pose = follower == null ? new Pose(START_X, START_Y, START_HEADING) : follower.pose();
        if (!gamepad1.a) {
            snap.update(pose.x(), pose.y(), pose.heading());
        }

        snap.debug(telemetry);
        telemetry.addData("Intake toggle", intakeOn);
        telemetry.addData("Localization", follower == null ? "Not configured" : pose);
        telemetry.update();
    }

    private void drive() {
        double turnSpeed = first_driver.getRightX();
        double strafePower = first_driver.getLeftX();
        double forwardPower = first_driver.getLeftY();
        double multiplier = first_driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2
                ? SLOW_POWER : 1;
        mecanumDrive.driveRobotCentric(strafePower * multiplier,
                forwardPower * multiplier, turnSpeed * multiplier);
    }

    @Override
    public void stop() {
        mecanumDrive.stop();
        snap.cancelAll();
        shooter.forceStopFlywheels();
    }
}