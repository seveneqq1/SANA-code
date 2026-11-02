package org.firstinspires.ftc.teamcode.bobot.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.function.LongSupplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bobot.config.RobotConfig;

@Config
public final class ShootingSystem {

    public static double SHOOTER_POWER = 0.6;
    public static double SPINUP_SECONDS = 1.0;
    public static double INTAKE_FEED_POWER = 0.8;
    public static double TRANSFER_FEED_POWER = 0.5;
    public static double STOPPER_OPEN_POSITION = 0.5;
    public static double STOPPER_CLOSED_POSITION = 0.0;
    public enum Mode { OFF, COLLECT, EJECT, SPINUP, SHOOT }

    private final DcMotor left, right, intake, transfer;
    private final Servo stopper;
    private final LongSupplier clock;
    private boolean spinning, ready, feeding;
    private long startedAt;
    private double power, elapsed;
    private Mode mode = Mode.OFF;

    public ShootingSystem(HardwareMap map) {
        this(map.get(DcMotor.class, RobotConfig.SHOOTER_LEFT_NAME),
                map.get(DcMotor.class, RobotConfig.SHOOTER_RIGHT_NAME),
                map.get(DcMotor.class, RobotConfig.INTAKE_NAME),
                map.get(DcMotor.class, RobotConfig.TRANSFER_NAME),
                map.get(Servo.class, RobotConfig.STOPPER_NAME), System::nanoTime);
    }

    public ShootingSystem(DcMotor left, DcMotor right, DcMotor intake, DcMotor transfer,
                          Servo stopper, LongSupplier clock) {
        this.left = left;
        this.right = right;
        this.intake = intake;
        this.transfer = transfer;
        this.stopper = stopper;
        this.clock = clock;
        configure(left, RobotConfig.SHOOTER_LEFT_DIRECTION, DcMotor.ZeroPowerBehavior.FLOAT);
        configure(right, RobotConfig.SHOOTER_RIGHT_DIRECTION, DcMotor.ZeroPowerBehavior.FLOAT);
        configure(intake, RobotConfig.INTAKE_DIRECTION, DcMotor.ZeroPowerBehavior.BRAKE);
        configure(transfer, RobotConfig.TRANSFER_DIRECTION, DcMotor.ZeroPowerBehavior.BRAKE);
        stop();
    }

    private static void configure(DcMotor motor, DcMotorSimple.Direction direction,
                                  DcMotor.ZeroPowerBehavior brake) {
        motor.setPower(0);
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(brake);
    }

    public void update(Mode requested) {
        mode = requested;
        power = mode == Mode.SPINUP || mode == Mode.SHOOT ? unit(SHOOTER_POWER) : 0;
        boolean run = power > 0;
        long now = clock.getAsLong();
        if (run && !spinning) startedAt = now;
        spinning = run;
        elapsed = run ? (now - startedAt) / 1e9 : 0;
        double delay = Double.isFinite(SPINUP_SECONDS) ? Math.max(0, SPINUP_SECONDS) : 1.0;
        ready = run && power > 0 && elapsed >= delay;
        feeding = mode == Mode.SHOOT && ready;

        left.setPower(power);
        right.setPower(power);
        stopper.setPosition(unit(feeding ? STOPPER_OPEN_POSITION : STOPPER_CLOSED_POSITION));
        double intakePower = 0, transferPower = 0;
        if (feeding) {
            intakePower = unit(INTAKE_FEED_POWER);
            transferPower = unit(TRANSFER_FEED_POWER);
        } else if (mode == Mode.COLLECT) {
            intakePower = signed(RobotConfig.INTAKE_POWER);
            transferPower = signed(RobotConfig.TRANSFER_COLLECT_POWER);
        } else if (mode == Mode.EJECT) {
            intakePower = signed(RobotConfig.INTAKE_EJECT_POWER);
            transferPower = signed(RobotConfig.TRANSFER_EJECT_POWER);
        }
        intake.setPower(intakePower);
        transfer.setPower(transferPower);
    }

    public void stop() { update(Mode.OFF); }

    public boolean isReady() { return ready; }
    public boolean isFeeding() { return feeding; }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooting mode", mode);
        telemetry.addData("Shooter commanded power", power);
        telemetry.addData("Spin-up elapsed seconds", elapsed);
        telemetry.addData("Spin-up delay seconds", SPINUP_SECONDS);
        telemetry.addData("Spin-up timer ready", ready);
        telemetry.addData("Feeding / stopper open", feeding);
    }

    private static double unit(double value) {
        return Double.isFinite(value) ? Math.max(0, Math.min(1, value)) : 0;
    }

    private static double signed(double value) {
        return Double.isFinite(value) ? Math.max(-1, Math.min(1, value)) : 0;
    }
}
