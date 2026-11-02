package org.firstinspires.ftc.teamcode.bobot.TestOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.bobot.controllers.StateController;
import org.firstinspires.ftc.teamcode.bobot.config.RobotConfig;
import org.firstinspires.ftc.teamcode.bobot.controllers.DriveController;
import org.firstinspires.ftc.teamcode.bobot.controllers.FlowerTakerController;
import org.firstinspires.ftc.teamcode.bobot.controllers.TurretController;
import org.firstinspires.ftc.teamcode.bobot.states.Alliance;
import org.firstinspires.ftc.teamcode.bobot.utilities.ControlMath;
import org.firstinspires.ftc.teamcode.bobot.states.PoseStorage;

@Config
@TeleOp(name = "Beka Tuning", group = "Tuning")
public class TuningTeleOp extends OpMode {
    public static Alliance ALLIANCE = Alliance.RED;
    public static boolean TURRET_MANUAL = false;
    public static double TURRET_MANUAL_DEG = 0.0;
    public static double TURRET_STEP_DEG = 15.0;

    private StateController stateController;
    private DriveController drive;
    private boolean intakeOn;
    private FlowerTakerController flowerTaker;
    private boolean wasDown, wasLeft, wasLeftBumper, wasRightBumper, wasRight, wasX, wasY;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        stateController = new StateController(hardwareMap, ALLIANCE);
        drive = new DriveController(hardwareMap, stateController.voltage);
        flowerTaker = new FlowerTakerController(hardwareMap);
        drive.setPose(PoseStorage.startPose(ALLIANCE));
        telemetry.addLine("Open http://192.168.43.1:8080/dash -> Configuration: ShootingSystem, RobotConfig, TuningTeleOp");
        telemetry.update();
    }

    @Override
    public void loop() {
        drive.updateOdometry();

        if (gamepad1.dpad_down && !wasDown) TURRET_MANUAL = !TURRET_MANUAL;
        if (gamepad1.left_bumper && !wasLeftBumper) TURRET_MANUAL_DEG += TURRET_STEP_DEG;
        if (gamepad1.right_bumper && !wasRightBumper) TURRET_MANUAL_DEG -= TURRET_STEP_DEG;
        if (gamepad1.dpad_right && !wasRight) drive.setPose(PoseStorage.cornerPose(ALLIANCE));
        if (gamepad1.dpad_left && !wasLeft) flowerTaker.toggle();
        if (gamepad1.x && !wasX) intakeOn = !intakeOn;
        if (gamepad1.y && !wasY) intakeOn = true;
        wasX = gamepad1.x;
        wasY = gamepad1.y;
        wasLeft = gamepad1.dpad_left;
        wasDown = gamepad1.dpad_down;
        wasLeftBumper = gamepad1.left_bumper;
        wasRightBumper = gamepad1.right_bumper;
        wasRight = gamepad1.dpad_right;

        TURRET_MANUAL_DEG = ControlMath.clamp(TURRET_MANUAL_DEG, RobotConfig.TURRET_MIN_DEG, RobotConfig.TURRET_MAX_DEG);
        stateController.turret.setManualAngle(TURRET_MANUAL && !gamepad1.a ? TURRET_MANUAL_DEG : null);

        if (gamepad1.a) {
            intakeOn = false;
            drive.stop();
            stateController.reset();
            flowerTaker.close();
        } else {
            boolean slow = gamepad1.right_trigger > 0.5;
            drive.drive(stick(-gamepad1.left_stick_y), stick(-gamepad1.left_stick_x), stick(-gamepad1.right_stick_x), slow);
            if (gamepad1.b) stateController.eject();
            else if (gamepad1.y) stateController.shoot();
            else if (intakeOn) stateController.intake();
            else stateController.idle();
        }
        stateController.setTurretAlwaysOn(!gamepad1.a);
        stateController.update(drive.getPose());
        flowerTaker.update();

        telemetry.addData("Shooter", gamepad1.y && !gamepad1.a && !gamepad1.b ? "spin up / shoot" : "off");
        telemetry.addData("Intake toggle (X)", intakeOn ? "ON" : "OFF");
        telemetry.addData("Flower taker (DPAD LEFT)", flowerTaker.isOpen() ? "OPEN" : "CLOSED");
        telemetry.addData("Alliance (set before INIT)", ALLIANCE);
        telemetry.addData("Pose reset (DPAD RIGHT)", drive.getPoseReset());
        telemetry.addData("Turret control (DPAD DOWN)", TURRET_MANUAL ? "MANUAL" : "AUTO AIM");
        telemetry.addData("Turret manual degrees", TURRET_MANUAL_DEG);
            telemetry.addData("Turret servo position", TurretController.FIXED_POSITION);
        drive.addOdometryTelemetry(telemetry);
        stateController.addTelemetry(telemetry, drive.getPose());
        telemetry.update();
    }

    private static double stick(double value) { return ControlMath.deadband(value, RobotConfig.DRIVE_DEADBAND); }

    @Override
    public void stop() {
        if (drive != null) drive.stop();
        if (stateController != null) stateController.stop();
    }
}
