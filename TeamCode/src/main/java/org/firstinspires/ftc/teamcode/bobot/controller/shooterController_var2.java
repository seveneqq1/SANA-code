package org.firstinspires.ftc.teamcode.bobot.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Shooter2 encoder drives shared flywheel PID. All speeds are ticks/second. */
@Config
public class shooterController_var2 {
    public final DcMotorEx shooter1, shooter2;
    public final Servo stopper;
    public static double baseVelocity = 1720;
    public static double velocityTolerance = 120;
    public static double blockedPosition = 0.05;
    public static double feedingPosition = 0.5;

    // Power per TPS error. Conservative starting gain; verify on hardware.
    public static double kP = 0.0003;
    // Power per (TPS * second); removes persistent speed error from feedforward mismatch.
    public static double kI = 0.0002;
    public static double kD = 0;
    // Starting estimate from previous 0.67 power / 2400 TPS, not measured calibration.
    public static double kV = 0.67 / 2400;
    public static boolean REVERSE_SHOOTER2 = true;

    private final ShooterVelocityPID pid = new ShooterVelocityPID();
    private double targetVelocity = baseVelocity;
    private double currentVelocity, power;
    private long previousUpdate;
    private boolean flywheelsActive = true;

    public shooterController_var2(HardwareMap hardwareMap) {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        stopper = hardwareMap.get(Servo.class, "stopper");
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        closeStopper();
        update();
    }

    public void setTargetVelocity(double velocity) {
        double target = Double.isFinite(velocity) && velocity > 0 ? velocity : 0;
        if (target != targetVelocity) {
            pid.reset();
        }
        targetVelocity = target;
    }

    /** Cancel feeding; preserve the existing always-spinning match behavior. */
    public void stop() { closeStopper(); }

    public void forceStopFlywheels() {
        flywheelsActive = false;
        pid.reset();
        power = 0;
        shooter1.setPower(0);
        shooter2.setPower(0);
        closeStopper();
    }

    public void setStopperPos(double position) { stopper.setPosition(position); }
    public void feedStopper() { setStopperPos(feedingPosition); }
    public void closeStopper() { setStopperPos(blockedPosition); }

    public void update() {
        long now = System.nanoTime();
        double dt = previousUpdate == 0 ? 0 : (now - previousUpdate) * 1e-9;
        previousUpdate = now;
        // Shooter1 has no encoder (user confirmed); never average its zero reading.
        currentVelocity = Math.abs(shooter2.getVelocity());
        if (!flywheelsActive) return;
        power = pid.calculate(targetVelocity, currentVelocity, dt, kP, kI, kD, kV);
        shooter1.setPower(power);
        shooter2.setPower(REVERSE_SHOOTER2 ? -power : power);
    }

    private boolean withinTolerance(double tolerance) {
        return flywheelsActive && targetVelocity > 0
                && Math.abs(currentVelocity - targetVelocity) < tolerance;
    }

    public boolean isAtTarget() { return withinTolerance(velocityTolerance); }
    public boolean isAlmostAtTarget() { return withinTolerance(velocityTolerance * 2.5); }

    public void debug(Telemetry telemetry) {
        telemetry.addData("Target TPS", targetVelocity);
        telemetry.addData("Measured TPS (shooter2)", currentVelocity);
        telemetry.addData("Shared shooter power", power);
        telemetry.addData("Shooter1 encoder", "Not installed; follows shooter2 PID");
        telemetry.addData("Shooter kP", kP);
        telemetry.addData("Shooter kI", kI);
        telemetry.addData("Shooter ready", isAtTarget());
        telemetry.addData("Shooter speed error TPS", targetVelocity - currentVelocity);
        if (!isAtTarget() && flywheelsActive && targetVelocity > 0) {
            telemetry.addData("Feed blocked", "Shooter2 must be within %.0f TPS of target", velocityTolerance);
            if (power >= .99) {
                telemetry.addLine("Full shooter power commanded: check shooter2 TPS, encoder, load and target.");
            }
        }
    }

    public double getCurrentVelocity() { return currentVelocity; }
    public double getTargetVelocity() { return targetVelocity; }
    public double getPower() { return power; }
}
