package org.firstinspires.ftc.teamcode.bobot.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.controller.PDController;

/**
 * Two flywheel motors (shooter1, shooter2) plus the stopper servo that feeds balls.
 *
 * The motors must spin opposite each other. By default shooter2 gets the negated
 * power command (REVERSE_SHOOTER2). If both wheels spin the same way, set that
 * flag to false in FTC Dashboard.
 *
 * Default mode matches this season's working robot: constant power, always on
 * during the match (flywheelPower). Set USE_VELOCITY_CONTROL = true to hold a
 * target speed with PD plus optional kV feedforward instead.
 *
 * This class replaces both the old PD shooterController and shooterController_var2.
 */
@Config
public class shooterController {
    public final DcMotorEx shooter1, shooter2;
    public final Servo stopper;
    private final VoltageSensor voltageSensor;
    private final PDController controller;

    /** Near-zone / default target speed in encoder ticks per second. Tune in Dashboard. */
    public static double baseVelocity = 2400;
    public static double velocityTolerance = 120;

    public static double blockedPosition = 0.05;
    public static double feedingPosition = 0.5;

    /** Used when USE_VELOCITY_CONTROL is false. */
    public static double flywheelPower = 0.67;

    /** False = constant power (current robot). True = PD + optional kV on measured speed. */
    public static boolean USE_VELOCITY_CONTROL = false;
    public static double kP = 0.008;
    public static double kD = 0;
    public static double kV = 0; // feedforward: power per TPS; leave 0 until you measure it
    public static boolean REVERSE_SHOOTER2 = true;

    private double targetVelocity = baseVelocity;
    private double currentVelocity;
    private double power = flywheelPower;
    private boolean flywheelsActive = true;

    public shooterController(HardwareMap hardwareMap) {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        stopper = hardwareMap.get(Servo.class, "stopper");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        controller = new PDController(kP, kD);

        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        closeStopper();
        update();
    }

    public void setTargetVelocity(double velocity) {
        this.targetVelocity = velocity > 0 ? velocity : baseVelocity;
    }

    /** Close the stopper. Flywheels keep spinning unless forceStopFlywheels() is called. */
    public void stop() {
        closeStopper();
    }

    public void forceStopFlywheels() {
        flywheelsActive = false;
        shooter1.setPower(0);
        shooter2.setPower(0);
        power = 0;
    }

    public void setStopperPos(double position) {
        stopper.setPosition(position);
    }

    public void feedStopper() {
        setStopperPos(feedingPosition);
    }

    public void closeStopper() {
        setStopperPos(blockedPosition);
    }

    public void update() {
        currentVelocity = (Math.abs(shooter1.getVelocity()) + Math.abs(shooter2.getVelocity())) / 2.0;

        if (!flywheelsActive) {
            power = 0;
            shooter1.setPower(0);
            shooter2.setPower(0);
            return;
        }

        if (USE_VELOCITY_CONTROL) {
            double target = targetVelocity > 0 ? targetVelocity : baseVelocity;
            controller.setPID(kP, 0, kD);
            double pd = controller.calculate(currentVelocity, target);
            double battery = voltageSensor.getVoltage();
            if (battery < 1) {
                battery = 12;
            }
            // kV is power-per-TPS. If you tune kV as voltage-per-TPS, divide by battery.
            power = kV * target + pd;
            if (power > 1) power = 1;
            if (power < 0) power = 0;
        } else {
            power = flywheelPower;
        }

        shooter1.setPower(power);
        shooter2.setPower(REVERSE_SHOOTER2 ? -power : power);
    }

    public boolean isAtTarget() {
        if (!USE_VELOCITY_CONTROL) {
            return flywheelsActive; // constant-power mode: ready as soon as wheels are on
        }
        double target = targetVelocity > 0 ? targetVelocity : baseVelocity;
        return Math.abs(currentVelocity - target) < velocityTolerance;
    }

    public boolean isAlmostAtTarget() {
        if (!USE_VELOCITY_CONTROL) {
            return flywheelsActive;
        }
        double target = targetVelocity > 0 ? targetVelocity : baseVelocity;
        return Math.abs(currentVelocity - target) < velocityTolerance * 2.5;
    }

    public double getCurrentVelocity() {
        return currentVelocity;
    }

    public double getTargetVelocity() {
        return targetVelocity > 0 ? targetVelocity : baseVelocity;
    }

    public double getPower() {
        return power;
    }
}
