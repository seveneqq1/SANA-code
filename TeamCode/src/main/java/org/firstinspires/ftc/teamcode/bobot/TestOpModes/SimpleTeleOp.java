package org.firstinspires.ftc.teamcode.bobot.TestOpModes;

import org.firstinspires.ftc.teamcode.bobot.config.ControlBindings;

import static org.firstinspires.ftc.teamcode.bobot.config.ControlBindings.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.bobot.config.RobotConfig;
import org.firstinspires.ftc.teamcode.bobot.controllers.FlowerTakerController;
import org.firstinspires.ftc.teamcode.bobot.utilities.ControlMath;
import org.firstinspires.ftc.teamcode.bobot.utilities.VoltageCompensator;
import org.firstinspires.ftc.teamcode.bobot.controllers.ShootingSystem;

@Config
@TeleOp(name = "Simple (no odo)", group = "Regional")
public class SimpleTeleOp extends OpMode {
    public static double DRIVE_SPEED = 1.0;
    public static double TURN_SPEED = 1.0;
    public static double SLOW_SPEED = 0.3;
    private enum Mode { IDLE, INTAKING, EJECTING, SHOOTING }

    private Mecanum drivetrain;
    private VoltageCompensator voltage;
    private ShootingSystem shooting;
    private FlowerTakerController flowerTaker;
    private Mode mode = Mode.IDLE;
    private boolean intakeOn, stopped;
    private boolean wasFlowerTaker, wasIntake, wasShoot;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        voltage = new VoltageCompensator(hardwareMap);
        drivetrain = new Mecanum(hardwareMap, Constants.driveConfig);
        String[] motorNames = {
                Constants.driveConfig.frontLeftName.get(), Constants.driveConfig.backLeftName.get(),
                Constants.driveConfig.frontRightName.get(), Constants.driveConfig.backRightName.get() };
        for (String name : motorNames) {
            DcMotor motor = hardwareMap.get(DcMotor.class, name);
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        drivetrain.stop();
        shooting = new ShootingSystem(hardwareMap);
        flowerTaker = new FlowerTakerController(hardwareMap);

        telemetry.addLine("Dashboard: http://192.168.43.1:8080/dash -> Configuration: ShootingSystem, SimpleTeleOp, RobotConfig");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (pressed(FLOWER_TAKER_DRIVER, FLOWER_TAKER_BUTTON, wasFlowerTaker)) flowerTaker.toggle();
        if (pressed(INTAKE_DRIVER, INTAKE_BUTTON, wasIntake)) intakeOn = !intakeOn;
        if (pressed(SHOOT_DRIVER, SHOOT_BUTTON, wasShoot)) intakeOn = true;
        wasFlowerTaker = held(FLOWER_TAKER_DRIVER, FLOWER_TAKER_BUTTON);
        wasIntake = held(INTAKE_DRIVER, INTAKE_BUTTON);
        wasShoot = held(SHOOT_DRIVER, SHOOT_BUTTON);

        boolean slow = held(SLOW_DRIVER, SLOW_BUTTON);
        stopped = held(STOP_DRIVER, STOP_BUTTON);
        if (stopped) {
            intakeOn = false;
            mode = Mode.IDLE;
            drivetrain.stop();
            flowerTaker.close();
        } else {
            drive(ControlBindings.driver(DRIVE_DRIVER, gamepad1, gamepad2), slow);
            mode = chooseMode();
        }
        updateMechanisms();
        flowerTaker.update();

        telemetry.addData("Mode", mode);
        telemetry.addData("Slow mode", slow);
        telemetry.addData("Flower taker", flowerTaker.isOpen() ? "OPEN" : "CLOSED");
        telemetry.addData("Intake toggle", intakeOn ? "ON" : "OFF");
        telemetry.addData("Battery volts", voltage.getVoltage());
        shooting.addTelemetry(telemetry);
        telemetry.update();
    }

    private void drive(Gamepad driver, boolean slow) {
        double scale = (slow ? ControlMath.clamp(SLOW_SPEED, 0, 1) : 1) * voltage.scale();
        double move = ControlMath.clamp(DRIVE_SPEED, 0, 1) * scale;
        double forward = stick(-driver.left_stick_y) * move;
        double left = stick(-driver.left_stick_x) * move;
        double turn = stick(-driver.right_stick_x) * ControlMath.clamp(TURN_SPEED, 0, 1) * scale;
        drivetrain.drive(new DrivePowers(forward, left, turn), true);
    }

    private Mode chooseMode() {
        if (held(EJECT_DRIVER, EJECT_BUTTON)) return Mode.EJECTING;
        if (held(SHOOT_DRIVER, SHOOT_BUTTON)) return Mode.SHOOTING;
        if (intakeOn) return Mode.INTAKING;
        return Mode.IDLE;
    }

    private void updateMechanisms() {
        switch (mode) {
            case SHOOTING: shooting.update(ShootingSystem.Mode.SHOOT); break;
            case INTAKING: shooting.update(ShootingSystem.Mode.COLLECT); break;
            case EJECTING: shooting.update(ShootingSystem.Mode.EJECT); break;
            default: shooting.stop();
        }
    }

    private boolean pressed(int driver, Button button, boolean wasHeld) { return held(driver, button) && !wasHeld; }

    private boolean held(int driver, Button button) { return ControlBindings.held(driver, button, gamepad1, gamepad2); }

    private static double stick(double value) { return ControlMath.deadband(value, RobotConfig.DRIVE_DEADBAND); }

    @Override
    public void stop() {
        if (drivetrain != null) drivetrain.stop();
        if (shooting != null) shooting.stop();
    }
}
