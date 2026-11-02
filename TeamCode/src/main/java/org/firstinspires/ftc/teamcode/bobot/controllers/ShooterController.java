package org.firstinspires.ftc.teamcode.bobot.controllers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

public class ShooterController {
     
    public static double SHOOTER1_RPM = 2400;
    public static double SHOOTER2_RPM = 2800;

    public static double HOOD1_POS = 0.70;
    public static double HOOD2_POS = 0.75;

    private DcMotorEx shooter1, shooter2;
    Servo hood2;
    Servo stopper1, stopper2;
    private long lastTime = 0;
    private double lastError1 = 0;
    private double lastError2 = 0;
    private double lastVelocity1 = 0;
    private double lastVelocity2 = 0;
    private double targetVelocityRPM1 = 0;
    private double targetVelocityRPM2 = 0;
    private VoltageSensor voltageSensor;
    private double integral1 = 0;
    private double integral2 = 0;
    public static double kV = 0.00527;
    public static double kP = 0.025;
    public static double kI = 0;
    public static double kD = 0;
    public double error1, error2;

    public enum BlockStates {
        BLOCK(0.67),
        UNBLOCK(0.5);
        final double blockPosition;
        BlockStates(double blockPosition) { this.blockPosition = blockPosition; }
    }

    public ShooterController(HardwareMap hardwareMap) {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        shooter1.setDirection(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE);

        lastTime = System.nanoTime();

        hood2 = hardwareMap.get(Servo.class, "hood2");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        stopper1 = hardwareMap.get(Servo.class, "stopper1");
        stopper2 = hardwareMap.get(Servo.class, "stopper2");
    }

    public void setBlockState1(BlockStates state) {
        if (state == BlockStates.BLOCK) {
            stopper1.setPosition(0.5);
        } else {
            stopper1.setPosition(0.25);
        }
    }

    public void setBlockState1test(double pos) { stopper1.setPosition(pos); }
    public void setBlockState2test(double pos) { stopper2.setPosition(pos); }

    public void setBlockState2(BlockStates state) {
        if (state == BlockStates.BLOCK) {
            stopper2.setPosition(0.67);
        } else {
            stopper2.setPosition(0.5);
        }
    }


    public void setHoodPosition2(double pos) { hood2.setPosition(pos); }
    public double getHood2Position() { return hood2.getPosition(); }
    public double getShooter1Power() { return shooter1.getPower(); }
    public double getShooter2Power() { return shooter2.getPower(); }
    public void setShooter1Power(double power) { shooter1.setPower(power); }
    public void setShooter2Power(double power) { shooter2.setPower(power); }
    public double getShooter1Velocity() { return currentVelocity1; }
    public double getShooter2Velocity() { return currentVelocity2; }
    public void setTargetVelocityRPM1(double rpm) { this.targetVelocityRPM1 = rpm; }
    public void setTargetVelocityRPM2(double rpm) { this.targetVelocityRPM2 = rpm; }
    public double getTargetVelocityRPM1() { return targetVelocityRPM1; }
    public double getTargetVelocityRPM2() { return targetVelocityRPM2; }

    public void stop() {
        targetVelocityRPM1 = 0;
        targetVelocityRPM2 = 0;
        shooter1.setPower(0);
        shooter2.setPower(0);
        lastError1 = 0;
        lastError2 = 0;
        integral1 = 0;
        integral2 = 0;
    }

    public boolean isShooter1AtTarget() { return Math.abs(targetVelocityRPM1 - currentVelocity1) < 50; }
    public boolean isShooter1AlmostAtTarget() { return Math.abs(targetVelocityRPM1 - currentVelocity1) < 300; }
    public boolean isShooter2AtTarget() { return Math.abs(targetVelocityRPM2 - currentVelocity2) < 50; }
    public boolean isShooter2AlmostAtTarget() { return Math.abs(targetVelocityRPM2 - currentVelocity2) < 300; }

    public double currentVelocity1;
    public double currentVelocity2;

    public void update() {
        if (targetVelocityRPM1 == 0 && targetVelocityRPM2 == 0) {
            shooter1.setPower(0);
            shooter2.setPower(0);
            lastError1 = 0;
            lastError2 = 0;
            return;
        }

        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9;
        lastTime = currentTime;

        currentVelocity1 = Math.abs(shooter1.getVelocity());
        currentVelocity2 = Math.abs(shooter2.getVelocity());

        double acceleration1 = (currentVelocity1 - lastVelocity1) / dt;
        double acceleration2 = (currentVelocity2 - lastVelocity2) / dt;

        lastVelocity1 = currentVelocity1;
        lastVelocity2 = currentVelocity2;

        double error1 = targetVelocityRPM1 - currentVelocity1;
        double error2 = targetVelocityRPM2 - currentVelocity2;

        integral1 += error1 * dt;
        integral2 += error2 * dt;

        double derivative1 = (error1 - lastError1) / dt;
        double derivative2 = (error2 - lastError2) / dt;

        lastError1 = error1;
        lastError2 = error2;

        double feedForward1 = targetVelocityRPM1 * kV;
        double feedForward2 = targetVelocityRPM2 * kV;

        double output1 = kP * error1 + kI * integral1 + kD * derivative1;
        double output2 = kP * error2 + kI * integral2 + kD * derivative2;

        double totalOutput1 = Range.clip(feedForward1 + output1, 0.0, 1.0);
        double totalOutput2 = Range.clip(feedForward2 + output2, 0.0, 1.0);

        shooter1.setPower(totalOutput1);
        shooter2.setPower(totalOutput2);
    }
}