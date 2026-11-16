package org.firstinspires.ftc.teamcode.bobot.teleop;

/**
 * Turret tracks one point on the field. Controls are the same as the Worlds TeleOp.
 *
 * Field: (0,0) = bottom-left corner seen from the driver, +X right, +Y away, 144 x 144 inches.
 * RESET: robot pushed into the BOTTOM-RIGHT corner, front facing away from the driver.
 *
 * gamepad1 (PS4/PS5), like Worlds:
 *   left stick = drive, right stick = turn
 *   R2 (hold) = slow, L2 (hold) = brake
 *   Square   = intake            Circle = balls out
 *   Triangle = shoot all balls   Cross  = stop everything
 *   L1 / R1 (hold) = turn turret by hand; on release the new target is saved automatically
 *   D-pad left = tracking on / off (off = turret looks forward)
 *   D-pad up   = FULL reset: pose (bottom-right corner) + target back to default + tracking on
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
    // Target (inches). At (137.4, 63) turret = 90 deg left; at reset (135, 9) about 25 deg left.
    // Smaller TARGET_X = aims more LEFT at reset, bigger = more right. Keep TARGET_Y = 63.
    public static double TARGET_X = 110, TARGET_Y = 63;

    // Reset corner (bottom-right)
    public static double ROBOT_LENGTH = 18, ROBOT_WIDTH = 18; // inches

    // Turret
    public static double TURRET_DIRECTION = 1;          // servo position up = turret turns left (measured Nov 14)
    public static double MANUAL_SPEED_DEG_PER_S = 120;

    // Pinpoint
    // goBILDA 4-bar pods = 19.894 ticks/mm. If distance is off: new = TICKS_PER_MM * (shown inches / real inches)
    public static double TICKS_PER_MM = 19.894;
    public static boolean REVERSE_X_POD = true;
    public static boolean REVERSE_Y_POD = false;
    public static double X_POD_OFFSET_MM = -154.5; // measured with Odo Offset Calibrate (Nov 13)
    public static double Y_POD_OFFSET_MM = -165.5; // measured with Odo Offset Calibrate (Nov 13)

    public static double SLOW_MULTIPLIER = 0.6;  // R2, same as Worlds
    public static double SHOOT_TIME = 1.5; // seconds stopper stays open; bigger if not all balls fly out
    // true = shooter always at full power (ready instantly). false = full power only while shooting.
    public static boolean ALWAYS_SPIN = true;

    private enum Mode { IDLE, INTAKE, OUTTAKE, SHOOT }
    private Mode mode = Mode.IDLE;
    private Mode lastMode = null;

    private GamepadEx driver;
    private List<LynxModule> hubs;
    private MecanumDrive drive;
    private shooterController_var2 shooter;
    private FixedPointTrackingMachine machine;
    private GoBildaPinpointDriver pinpoint;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime feedTimer = new ElapsedTime();
    private boolean feeding;
    private boolean wasManual;
    private boolean trackingOn = true;

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

        // ---- Turret ----
        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            trackingOn = !trackingOn;
            if (trackingOn) machine.setTrackingEnabled(true);
            else machine.centerTurret();
        }
        boolean left = gamepad1.left_bumper, right = gamepad1.right_bumper;
        boolean manualNow = left != right;
        if (manualNow) {
            double step = MANUAL_SPEED_DEG_PER_S * dt;
            machine.manualMove((left ? step : -step) * TURRET_DIRECTION); // L1 = left, R1 = right
        }
        if (wasManual && !manualNow && trackingOn) {
            machine.lockTargetHere(x, y, heading); // save the new target automatically
        }
        wasManual = manualNow;

        // ---- Buttons (Worlds layout) ----
        if (driver.wasJustPressed(GamepadKeys.Button.X)) mode = Mode.INTAKE;   // Square
        if (driver.wasJustPressed(GamepadKeys.Button.B)) mode = Mode.OUTTAKE;  // Circle
        if (driver.wasJustPressed(GamepadKeys.Button.Y)) { mode = Mode.SHOOT; feeding = false; } // Triangle
        if (driver.wasJustPressed(GamepadKeys.Button.A)) mode = Mode.IDLE;     // Cross

        shooter.boost = ALWAYS_SPIN || mode == Mode.SHOOT;
        boolean modeChanged = mode != lastMode; // set motors only once when mode changes (no 0/1 flicker every loop)
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
                if (!feeding && shooter.isAtTarget()) { feeding = true; feedTimer.reset(); }
                if (feeding && feedTimer.seconds() > SHOOT_TIME) mode = Mode.INTAKE; // keep collecting after shooting
                break;
            default:
                if (modeChanged) machine.cancelAll();
        }

        // ---- Drive ----
        boolean brake = driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2;
        if (brake) {
            drive.stop();
        } else {
            double scale = driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2 ? SLOW_MULTIPLIER : 1;
            drive.driveRobotCentric(driver.getLeftX() * scale,
                    driver.getLeftY() * scale, driver.getRightX() * scale);
        }

        machine.setManualTPS(shooterController_var2.baseVelocity);
        machine.update(x, y, heading);

        telemetry.addData("Mode", mode);
        telemetry.addData("Turret", !trackingOn ? "OFF (forward)" : manualNow ? "MANUAL" : "TRACKING");
        telemetry.addData("Robot X / Y", "%.1f / %.1f", x, y);
        telemetry.addData("Heading degrees", "%.1f", Math.toDegrees(heading));
        telemetry.addData("Target X / Y", "%.1f / %.1f", machine.getTargetX(), machine.getTargetY());
        telemetry.addData("Distance to target", "%.1f", machine.getDistanceInches());
        telemetry.addData("Turret OUT OF RANGE", machine.isOutOfRange());
        telemetry.addData("Pinpoint status", pinpoint.getDeviceStatus());
        shooter.debug(telemetry);
        telemetry.update();
    }

    /** Robot pushed into the bottom-right corner, front facing away from the driver. */
    private void resetPose() {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,
                144 - ROBOT_WIDTH / 2, ROBOT_LENGTH / 2, AngleUnit.DEGREES, 90));
        // full reset: forget manual turret corrections, tracking back on
        machine.setTarget(TARGET_X, TARGET_Y);
        trackingOn = true;
        wasManual = false;
        machine.setTrackingEnabled(true);
    }

    @Override
    public void stop() {
        if (drive != null) drive.stop();
        if (machine != null) machine.cancelAll();
        if (shooter != null) shooter.forceStopFlywheels();
    }
}