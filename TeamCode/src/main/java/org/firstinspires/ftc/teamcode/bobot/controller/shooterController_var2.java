package org.firstinspires.ftc.teamcode.bobot.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Shooter2 encoder drives shared flywheel PID. All speeds are ticks/second. */
@Config
public class  shooterController_var2 {
    public final DcMotorEx shooter1, shooter2;
    public final Servo stopper;
    private final VoltageSensor voltageSensor;
    private double voltage = 12.0;
    public static double baseVelocity = 2400; // max ~2800 TPS (6000 RPM motor)
    public static double velocityTolerance = 120;
    // FULL_POWER = true: shooter always at max power (no PID). Feeding allowed above READY_TPS.
    public static boolean FULL_POWER = false; // false = PID holds baseVelocity (has headroom to recover after each ball)
    // Voltage compensation: power * (NOMINAL_VOLTAGE / battery), same shot on full or tired battery.
    public static boolean VOLTAGE_COMP = true;
    public static double NOMINAL_VOLTAGE = 12.0;
    // Smooths noisy encoder speed. 1 = no filter, 0.2 = very smooth (slower reaction).
    public static double VELOCITY_FILTER = 0.3;
    public static double READY_TPS = 2300;
    // Power when not shooting (saves battery). 0 = off.
    public static double IDLE_POWER = 0.3;
    /** Set true while the driver wants to shoot: full power. False: IDLE_POWER. */
    public boolean boost = true;
    public static double blockedPosition = 0.35;
    public static double feedingPosition = 0.5;

    // Power per TPS error. Conservative starting gain; verify on hardware.
    public static double kP = 0.0008;
    // Power per (TPS * second); removes persistent speed error from feedforward mismatch.
    public static double kI = 0.0001;
    public static double kD = 0;
    // Starting estimate from previous 0.67 power / 2400 TPS, not measured calibration.
    public static double kV = 0.67 / 2400;

    // Motor directions. Both false = both motors get the same sign (spin the same way).
    // If both wheels spin the wrong way for shooting, set BOTH to true.
    // If only one wheel is wrong, flip only that one.
    public static boolean REVERSE_SHOOTER1 = false;
    public static boolean REVERSE_SHOOTER2 = true; // both true = both wheels flipped to push the ball outward

    private final ShooterVelocityPID pid = new ShooterVelocityPID();
    private double targetVelocity = baseVelocity;
    private double currentVelocity, rawVelocity, filteredVolts = 12.0, power;
    private long previousUpdate;
    private boolean flywheelsActive = true;

    public shooterController_var2(HardwareMap hardwareMap) {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        stopper = hardwareMap.get(Servo.class, "stopper");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
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
        // Speed is measured as abs(), so the encoder sign does not affect the PID.
        rawVelocity = Math.abs(shooter2.getVelocity());
        currentVelocity += VELOCITY_FILTER * (rawVelocity - currentVelocity);
        if (!flywheelsActive) return;
        filteredVolts += 0.05 * (voltageSensor.getVoltage() - filteredVolts); // voltage dips when balls hit, smooth it
        voltage = filteredVolts;
        if (FULL_POWER) {
            power = boost ? 1.0 : IDLE_POWER;
        } else if (boost) {
            power = pid.calculate(targetVelocity, currentVelocity, dt, kP, kI, kD, kV);
            if (VOLTAGE_COMP && voltage > 6) power *= NOMINAL_VOLTAGE / voltage;
        } else {
            power = IDLE_POWER;
        }
        power = Math.max(0, Math.min(1, power));
        shooter1.setPower(REVERSE_SHOOTER1 ? -power : power);
        shooter2.setPower(REVERSE_SHOOTER2 ? -power : power);
    }

    private boolean withinTolerance(double tolerance) {
        if (FULL_POWER) return flywheelsActive && currentVelocity > READY_TPS;
        return flywheelsActive && targetVelocity > 0
                && Math.abs(currentVelocity - targetVelocity) < tolerance;
    }

    public boolean isAtTarget() { return withinTolerance(velocityTolerance); }
    public boolean isAlmostAtTarget() { return withinTolerance(velocityTolerance * 2.5); }

    public void debug(Telemetry telemetry) {
        telemetry.addData("Target TPS", targetVelocity);
        telemetry.addData("Measured TPS (shooter2)", currentVelocity);
        telemetry.addData("Raw TPS (no filter)", rawVelocity);
        telemetry.addData("Shared shooter power", power);
        telemetry.addData("Shooter1 encoder", "Not installed; follows shooter2 PID");
        telemetry.addData("Reverse shooter1/2", REVERSE_SHOOTER1 + " / " + REVERSE_SHOOTER2);
        telemetry.addData("Battery voltage", "%.2f V (comp %s)", voltage, VOLTAGE_COMP);
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