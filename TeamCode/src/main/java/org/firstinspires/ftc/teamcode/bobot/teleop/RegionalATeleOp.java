package org.firstinspires.ftc.teamcode.bobot.teleop;

/**
 * Match TeleOp for Regional A (snap turret, always-on shooter).
 *
 * gamepad1 (driver, PS4/PS5):
 *   left stick = translate, right stick = turn (also cancels heading hold)
 *   R2 = intake, L2 = eject, hold Triangle (Y) = shoot, Cross (A) = stop
 *   L1 = slow, Square/Circle (X/B) = snap heading 90 degrees while hold is on
 *
 * gamepad2 (optional operator):
 *   D-pad up = lock current heading, L1/R1 = turret 90-degree snap, Square (X) = center turret
 */

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.bobot.controller.HeadingController;
import org.firstinspires.ftc.teamcode.bobot.controller.SnapStateMachine;
import org.firstinspires.ftc.teamcode.bobot.controller.intakeController;
import org.firstinspires.ftc.teamcode.bobot.controller.shooterController_var2;
import org.firstinspires.ftc.teamcode.bobot.controller.turretController;

import java.util.List;

@Config
@TeleOp(name = "Regional A - Snap", group = "Regional A")
public class RegionalATeleOp extends OpMode {
    public static double SLOW_POWER = 0.35;
    public static double TURRET_DIRECTION = 1;
    // Starting values for tuning, not measured calibration.
    public static double HEADING_KP = 1;
    public static double HEADING_KD = 0;
    public static double HEADING_MAX_POWER = 0.35;
    public static double HEADING_TOLERANCE_RADIANS = Math.toRadians(2);
    public static double RIGHT_STICK_DEADBAND = 0.05;

    private GamepadEx driver, operator;
    private List<LynxModule> hubs;
    private MecanumDrive drive;
    private shooterController_var2 shooter;
    private SnapStateMachine machine;
    private GoBildaPinpointDriver pinpoint;
    private HeadingController heading;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double appliedHeadingKP, appliedHeadingKD;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        Motor frontLeft = new Motor(hardwareMap, "frontleft");
        Motor frontRight = new Motor(hardwareMap, "frontright");
        Motor backLeft = new Motor(hardwareMap, "backleft");
        Motor backRight = new Motor(hardwareMap, "backright");
        frontRight.setInverted(true);
        backRight.setInverted(true);
        for (Motor motor : new Motor[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            motor.set(0);
        }
        drive = new MecanumDrive(false, frontLeft, frontRight, backLeft, backRight);

        shooter = new shooterController_var2(hardwareMap);
        intakeController intake = new intakeController(hardwareMap);
        turretController turret = new turretController(hardwareMap);
        machine = new SnapStateMachine(shooter, intake, turret, null);
        machine.turretDirection = TURRET_DIRECTION;

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        heading = new HeadingController(HEADING_KP, HEADING_KD);
        appliedHeadingKP = HEADING_KP;
        appliedHeadingKD = HEADING_KD;

        telemetry.addLine("Pinpoint heading hold + turret snap; fixed shooter TPS.");
        telemetry.addLine("G1: RT intake, LT eject, hold Y shoot, A stop, LB slow");
        telemetry.addLine("G1: X/B heading left/right 90 degrees; right stick cancels hold");
        telemetry.addLine("G2: D-pad up locks heading, LB/RB turret CCW/CW, X centers turret");
        telemetry.update();
    }

    @Override
    public void start() {
        heading.disableHold();
        loopTimer.reset();
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        driver.readButtons();
        operator.readButtons();
        double dtSeconds = loopTimer.seconds();
        loopTimer.reset();
        pinpoint.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);
        double currentHeading = pinpoint.getHeading(AngleUnit.RADIANS);

        if (gamepad1.a) {
            heading.disableHold();
            drive.stop();
            machine.cancelAll();
        } else {
            boolean manualOverride = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y)
                    > RIGHT_STICK_DEADBAND;
            boolean lockPressed = operator.wasJustPressed(GamepadKeys.Button.DPAD_UP);
            if (heading.isHoldingHeading() && !manualOverride && !lockPressed) {
                if (!gamepad1.b && driver.wasJustPressed(GamepadKeys.Button.X)) {
                    heading.snapTargetHeading(HeadingController.Direction.ANTICLOCKWISE);
                } else if (!gamepad1.x && driver.wasJustPressed(GamepadKeys.Button.B)) {
                    heading.snapTargetHeading(HeadingController.Direction.CLOCKWISE);
                }
            }
            if (HEADING_KP != appliedHeadingKP || HEADING_KD != appliedHeadingKD) {
                heading.setGains(HEADING_KP, HEADING_KD);
                appliedHeadingKP = HEADING_KP;
                appliedHeadingKD = HEADING_KD;
            }
            heading.setMaxPower(HEADING_MAX_POWER);
            heading.setTolerance(HEADING_TOLERANCE_RADIANS);
            double turnCCW = heading.updateHold(currentHeading, dtSeconds, lockPressed, manualOverride);
            // SolversLib positive turn power is clockwise; the heading helper returns CCW.
            double turnPower = heading.isHoldingHeading() ? -turnCCW : driver.getRightX();
            double scale = gamepad1.left_bumper ? SLOW_POWER : 1;
            drive.driveRobotCentric(driver.getLeftX() * scale,
                    driver.getLeftY() * scale, turnPower * scale);

            if (operator.wasJustPressed(GamepadKeys.Button.X)) {
                machine.centerTurret();
            } else if (!gamepad2.left_bumper && operator.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                machine.rotateClockwise90();
            } else if (!gamepad2.right_bumper && operator.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                machine.rotateCounterclockwise90();
            }

            if (driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) {
                machine.reverseIntake();
            } else if (gamepad1.y) {
                machine.spinUp();
                machine.requestFeed(true);
            } else if (driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2) {
                machine.startIntaking();
            } else {
                machine.cancelAll();
            }

            machine.setManualTPS(shooterController_var2.baseVelocity);
            // X/Y are unused for snap aiming and fixed TPS; only heading is measured.
            machine.update(0, 0, currentHeading);
        }

        telemetry.addData("Shooting state", machine.getState());
        telemetry.addData("Heading hold", heading.isHoldingHeading());
        telemetry.addData("Heading degrees", Math.toDegrees(currentHeading));
        telemetry.addData("Heading target degrees", heading.isHoldingHeading()
                ? Math.toDegrees(heading.getTargetHeading()) : "Manual");
        telemetry.addData("Target TPS", shooter.getTargetVelocity());
        telemetry.addData("Measured TPS", shooter.getCurrentVelocity());
        telemetry.addData("Turret command degrees", Math.toDegrees(machine.getTurretAngleRadians()));
        telemetry.update();
    }

    @Override
    public void stop() {
        if (heading != null) {
            heading.disableHold();
        }
        if (drive != null) {
            drive.stop();
        }
        if (machine != null) {
            machine.cancelAll();
        }
    }
}
