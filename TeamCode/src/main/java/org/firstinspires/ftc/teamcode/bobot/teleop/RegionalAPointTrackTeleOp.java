package org.firstinspires.ftc.teamcode.bobot.teleop;

/**
 * Turret tracks one point on the field. The driver drives the robot.
 *
 * Field: (0,0) = bottom-left corner seen from the driver, +X right, +Y away, 144 x 144 inches.
 * RESET: robot pushed into the BOTTOM-RIGHT corner, front facing away from the driver.
 * The pose is set there on START and on D-pad up.
 *
 * gamepad1 (PS4/PS5):
 *   left stick = drive, right stick = turn, L3 (press left stick) = slow
 *   R2 = intake, L2 = eject, hold Triangle = shoot, Cross = stop all
 *   L1 / R1 (hold) = turn turret left / right by hand
 *   Circle         = "target is HERE": remember where the turret points and track it
 *   D-pad up       = reset pose (robot in the bottom-right corner)
 */

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.bobot.controller.FixedPointTrackingMachine;
import org.firstinspires.ftc.teamcode.bobot.controller.intakeController;
import org.firstinspires.ftc.teamcode.bobot.controller.shooterController_var2;
import org.firstinspires.ftc.teamcode.bobot.controller.turretController;

import java.util.List;

@Config
@TeleOp(name = "Regional A - Point Track", group = "Regional A")
public class RegionalAPointTrackTeleOp extends OpMode {
    // Target point on the field (inches). Write here the "Target X / Y" you see after Circle.
    public static double TARGET_X = 72, TARGET_Y = 72;

    // Reset corner (bottom-right)
    public static double ROBOT_LENGTH = 18, ROBOT_WIDTH = 18; // inches

    // Turret
    public static double TURRET_DIRECTION = -1;         // if the turret aims the wrong way, set -1
    public static double MANUAL_SPEED_DEG_PER_S = 120;

    // Pinpoint
    // goBILDA 4-bar pods = 19.894 ticks/mm. If distance is off: new = TICKS_PER_MM * (shown inches / real inches)
    public static double TICKS_PER_MM = 19.894;
    public static boolean REVERSE_X_POD = true;
    public static boolean REVERSE_Y_POD = false;
    public static double X_POD_OFFSET_MM = -245.5;
    public static double Y_POD_OFFSET_MM = -133.5;

    public static double SLOW_POWER = 0.35;

    private GamepadEx driver;
    private List<LynxModule> hubs;
    private MecanumDrive drive;
    private shooterController_var2 shooter;
    private FixedPointTrackingMachine machine;
    private GoBildaPinpointDriver pinpoint;
    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        driver = new GamepadEx(gamepad1);
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        Motor frontLeft = new Motor(hardwareMap, "frontleft");
        Motor frontRight = new Motor(hardwareMap, "frontright");
        Motor backLeft = new Motor(hardwareMap, "backleft");
        Motor backRight = new Motor(hardwareMap, "backright");
        frontRight.setInverted(true);
        backRight.setInverted(true);
        for (Motor m : new Motor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            m.set(0);
        }
        drive = new MecanumDrive(false, frontLeft, frontRight, backLeft, backRight);

        shooter = new shooterController_var2(hardwareMap);
        machine = new FixedPointTrackingMachine(shooter,
                new intakeController(hardwareMap), new turretController(hardwareMap));
        machine.setTarget(TARGET_X, TARGET_Y);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(TICKS_PER_MM, DistanceUnit.MM);
        pinpoint.setEncoderDirections(
                REVERSE_X_POD ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                        : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                REVERSE_Y_POD ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                        : GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU(); // robot must stand still during INIT

        telemetry.addLine("Put the robot in the BOTTOM-RIGHT corner, front away from you.");
        telemetry.addLine("L1/R1 = turret by hand, Circle = target is here, D-pad up = reset");
        telemetry.update();
    }

    @Override
    public void start() {
        resetPose();
        machine.setTrackingEnabled(true);
        loopTimer.reset();
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        driver.readButtons();
        double dt = Math.min(loopTimer.seconds(), 0.1);
        loopTimer.reset();

        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_UP)) resetPose();
        pinpoint.update();
        double x = pinpoint.getPosX(DistanceUnit.INCH);
        double y = pinpoint.getPosY(DistanceUnit.INCH);
        double heading = pinpoint.getHeading(AngleUnit.RADIANS);
        machine.turretDirection = TURRET_DIRECTION;

        // Turret: L1/R1 by hand, Circle = remember this direction as the target.
        boolean left = gamepad1.left_bumper, right = gamepad1.right_bumper;
        if (left != right) {
            double step = MANUAL_SPEED_DEG_PER_S * dt;
            machine.manualMove((left ? step : -step) * TURRET_DIRECTION); // keeps L1/R1 the same physical way
        }
        if (driver.wasJustPressed(GamepadKeys.Button.B)) {
            machine.lockTargetHere(x, y, heading);
        }

        if (gamepad1.a) {
            drive.stop();
            machine.cancelAll();
        } else {
            double scale = gamepad1.left_stick_button ? SLOW_POWER : 1;
            drive.driveRobotCentric(driver.getLeftX() * scale,
                    driver.getLeftY() * scale, driver.getRightX() * scale);
            updateMechanisms();
        }
        machine.setManualTPS(shooterController_var2.baseVelocity);
        machine.update(x, y, heading);

        telemetry.addData("Turret mode", machine.isTrackingEnabled() ? "AUTO" : "MANUAL");
        telemetry.addData("Robot X / Y", "%.1f / %.1f", x, y);
        telemetry.addData("ODO forward pod (encX)", pinpoint.getEncoderX());
        telemetry.addData("ODO side pod (encY)", pinpoint.getEncoderY());
        telemetry.addData("Pinpoint status", pinpoint.getDeviceStatus());
        telemetry.addData("Heading degrees", "%.1f", Math.toDegrees(heading));
        telemetry.addData("Target X / Y", "%.1f / %.1f", machine.getTargetX(), machine.getTargetY());
        telemetry.addData("Distance to target", "%.1f", machine.getDistanceInches());
        telemetry.addData("Turret OUT OF RANGE", machine.isOutOfRange());
        telemetry.addData("Shooting state", machine.getState());
        shooter.debug(telemetry);
        telemetry.update();
    }

    /** Robot pushed into the bottom-right corner, front facing away from the driver. */
    private void resetPose() {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,
                144 - ROBOT_WIDTH / 2, ROBOT_LENGTH / 2, AngleUnit.DEGREES, 90));
    }

    private void updateMechanisms() {
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
    }

    @Override
    public void stop() {
        if (drive != null) drive.stop();
        if (machine != null) machine.cancelAll();
        if (shooter != null) shooter.forceStopFlywheels();
    }
}
