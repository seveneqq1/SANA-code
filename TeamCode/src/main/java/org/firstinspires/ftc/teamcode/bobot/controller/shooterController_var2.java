package org.firstinspires.ftc.teamcode.bobot.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class shooterController_var2 {
    public final DcMotorEx shooter1, shooter2;
    public final Servo stopper;

    public static double baseVelocity = 2400;
    public static double velocityTolerance = 120;

    // Stopper Positions from test file
    public static double blockedPosition = 0.05; // Closed (default)
    public static double feedingPosition = 0.5;  // Open (shooting)

    // Constant Flywheel Power
    public static double flywheelPower = 0.67;

    private double targetVelocity = baseVelocity;
    private double power = flywheelPower;
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
        // Keep target positive for state machine readiness checks
        this.targetVelocity = velocity > 0 ? velocity : baseVelocity;
    }

    public void stop() {
        // Stopper closes when state machine cancels, flywheels remain spinning
        closeStopper();
    }

    public void forceStopFlywheels() {
        flywheelsActive = false;
        shooter1.setPower(0);
        shooter2.setPower(0);
    }

    public void setStopperPos(double position) {
        stopper.setPosition(position);
    }

    public void feedStopper() {
        setStopperPos(feedingPosition); // 0.5
    }

    public void closeStopper() {
        setStopperPos(blockedPosition); // 0.05
    }

    public void update() {
        if (flywheelsActive) {
            power = flywheelPower;
            shooter1.setPower(power);   // 0.8
            shooter2.setPower(-power);  // -0.8
        }
    }

    public boolean isAtTarget() {
        return true; // Always ready immediately
    }

    public boolean isAlmostAtTarget() {
        return true;
    }

    public double getCurrentVelocity() {
        return targetVelocity;
    }

    public double getTargetVelocity() {
        return targetVelocity > 0 ? targetVelocity : baseVelocity;
    }

    public double getPower() {
        return power;
    }
}