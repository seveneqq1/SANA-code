package org.firstinspires.ftc.teamcode.bobot.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

 
















@Config
public class ShooterControllerOLD {

     
     
    public static double MOTOR_TICKS_PER_REV = 28.0;
    public static double MOTOR_FREE_SPEED_RPM = 6000.0;

     
    public static double POLLEN_MASS_KG = 0.0249;    
    public static double POLLEN_RADIUS_M = 0.0356;   
    public static double NECTAR_MASS_KG = 0.0413;    
    public static double NECTAR_RADIUS_M = 0.0457;   

     
     
     
     
    public static double AIR_DENSITY_KG_M3 = 1.225;
    public static double DRAG_COEFFICIENT = 0.5;

    public static double SHOOTER1_RPM = 2150;  
    public static double SHOOTER2_RPM = 3000;  
    public static double HOOD2_POS = 0.5;
     
    public static double STOPPER1_BLOCK_POS = 0.875;
    public static double STOPPER1_UNBLOCK_POS = 0.25;
    public static double STOPPER2_BLOCK_POS = 0.67;
    public static double STOPPER2_UNBLOCK_POS = 0.50;

     
    public static double kV1 = 0.00015;  
    public static double kP1 = 0.0015;
    public static double kI1 = 0.000;   
    public static double kD1 = 0.00001;  

     
     
     
     
    public static double kV2 = 0.00015;
    public static double kP2 = 0.0018;
    public static double kI2 = 0.000;
    public static double kD2 = 0.00004;

    public static double MAX_INTEGRAL = 500;
     
     
     
     
     
    public static double INTEGRAL_ENGAGE_ERROR_RPM = 180;
    public static double NOMINAL_VOLTAGE = 13.0;

     
     
     
     
    public static double VELOCITY_FILTER_ALPHA = 0.5;

     
     
     
     
     
     
     
     
    public static double SHOT_KICK_POWER1 = 0.15;  
    public static double SHOT_KICK_POWER2 = 0.11;  
    public static long SHOT_KICK_DURATION_MS = 62;  

    private final DcMotorEx shooter1, shooter2;
    private final Servo hood2;
    private final Servo stopper1, stopper2;
    private final VoltageSensor voltageSensor;

    private long lastTime;
    private double integral1 = 0;
    private double integral2 = 0;
    private double filteredVelocity1 = 0;
    private double filteredVelocity2 = 0;
    private double lastVelocity1 = 0;
    private double lastVelocity2 = 0;
    private double targetVelocityRPM1 = 0;
    private double targetVelocityRPM2 = 0;
    private long shotKickEndTimeNs1 = 0;
    private long shotKickEndTimeNs2 = 0;

    public double error1, error2;
    public double currentVelocity1, currentVelocity2;

    public enum BlockStates { BLOCK, UNBLOCK }

    public ShooterControllerOLD(HardwareMap hardwareMap) {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        hood2 = hardwareMap.get(Servo.class, "hood2");
        stopper1 = hardwareMap.get(Servo.class, "stopper1");
        stopper2 = hardwareMap.get(Servo.class, "stopper2");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        lastTime = System.nanoTime();
    }

    public void setBlockState1(BlockStates state) {
        stopper1.setPosition(state == BlockStates.BLOCK ? STOPPER1_BLOCK_POS : STOPPER1_UNBLOCK_POS);
    }

    public void setBlockState2(BlockStates state) {
        stopper2.setPosition(state == BlockStates.BLOCK ? STOPPER2_BLOCK_POS : STOPPER2_UNBLOCK_POS);
    }

    public void setBlockState1test(double pos) { stopper1.setPosition(pos); }
    public void setBlockState2test(double pos) { stopper2.setPosition(pos); }

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

    public double getVoltage() { return voltageSensor.getVoltage(); }

    public boolean isShooter1AtTarget() { return Math.abs(targetVelocityRPM1 - currentVelocity1) < 80; }
    public boolean isShooter1AlmostAtTarget() { return Math.abs(targetVelocityRPM1 - currentVelocity1) < 250; }
    public boolean isShooter2AtTarget() { return Math.abs(targetVelocityRPM2 - currentVelocity2) < 80; }
    public boolean isShooter2AlmostAtTarget() { return Math.abs(targetVelocityRPM2 - currentVelocity2) < 250; }

     




    public void fireShot1() {
        shotKickEndTimeNs1 = System.nanoTime() + (long) (SHOT_KICK_DURATION_MS * 1e6);
    }

     
    public void fireShot2() {
        shotKickEndTimeNs2 = System.nanoTime() + (long) (SHOT_KICK_DURATION_MS * 1e6);
    }

    public void stop() {
        targetVelocityRPM1 = 0;
        targetVelocityRPM2 = 0;
        shooter1.setPower(0);
        shooter2.setPower(0);
        integral1 = 0;
        integral2 = 0;
        shotKickEndTimeNs1 = 0;
        shotKickEndTimeNs2 = 0;
    }

    public void update() {
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9;
        lastTime = currentTime;
        if (dt <= 0) return;

        double rawVelocity1 = (Math.abs(shooter1.getVelocity()) * 60.0) / MOTOR_TICKS_PER_REV;
        double rawVelocity2 = (Math.abs(shooter2.getVelocity()) * 60.0) / MOTOR_TICKS_PER_REV;

         
         
        filteredVelocity1 += VELOCITY_FILTER_ALPHA * (rawVelocity1 - filteredVelocity1);
        filteredVelocity2 += VELOCITY_FILTER_ALPHA * (rawVelocity2 - filteredVelocity2);
        currentVelocity1 = filteredVelocity1;
        currentVelocity2 = filteredVelocity2;

         
         
         
         
         
         
         
        double voltage = voltageSensor.getVoltage();
        double voltageComp = voltage > 1.0 ? NOMINAL_VOLTAGE / voltage : 1.0;
        long now = currentTime;

         
        if (targetVelocityRPM1 > 0) {
            error1 = targetVelocityRPM1 - currentVelocity1;

            if (Math.abs(error1) < INTEGRAL_ENGAGE_ERROR_RPM) {
                integral1 = Range.clip(integral1 + error1 * dt, -MAX_INTEGRAL, MAX_INTEGRAL);
            }

             
             
            double derivative1 = -(currentVelocity1 - lastVelocity1) / dt;
            lastVelocity1 = currentVelocity1;

            double feedForward1 = targetVelocityRPM1 * kV1;
            double output1 = kP1 * error1 + kI1 * integral1 + kD1 * derivative1;
            double kick1 = (now < shotKickEndTimeNs1) ? SHOT_KICK_POWER1 : 0.0;

            shooter1.setPower(Range.clip((feedForward1 + output1 + kick1) * voltageComp, 0.0, 1.0));
        } else {
            shooter1.setPower(0);
            integral1 = 0;
            lastVelocity1 = currentVelocity1;
            shotKickEndTimeNs1 = 0;
        }

         
        if (targetVelocityRPM2 > 0) {
            error2 = targetVelocityRPM2 - currentVelocity2;

            if (Math.abs(error2) < INTEGRAL_ENGAGE_ERROR_RPM) {
                integral2 = Range.clip(integral2 + error2 * dt, -MAX_INTEGRAL, MAX_INTEGRAL);
            }

            double derivative2 = -(currentVelocity2 - lastVelocity2) / dt;
            lastVelocity2 = currentVelocity2;

            double feedForward2 = targetVelocityRPM2 * kV2;
            double output2 = kP2 * error2 + kI2 * integral2 + kD2 * derivative2;
            double kick2 = (now < shotKickEndTimeNs2) ? SHOT_KICK_POWER2 : 0.0;

            shooter2.setPower(Range.clip((feedForward2 + output2 + kick2) * voltageComp, 0.0, 1.0));
        } else {
            shooter2.setPower(0);
            integral2 = 0;
            lastVelocity2 = currentVelocity2;
            shotKickEndTimeNs2 = 0;
        }
    }

     
     
     
     

     
    public static double flywheelSurfaceSpeedMps(double rpm, double flywheelRadiusM) {
        double omega = rpm * 2.0 * Math.PI / 60.0;
        return omega * flywheelRadiusM;
    }

     
    public static double dragDeceleration(double speedMps, double massKg, double radiusM) {
        double area = Math.PI * radiusM * radiusM;
        double dragForce = 0.5 * AIR_DENSITY_KG_M3 * speedMps * speedMps * DRAG_COEFFICIENT * area;
        return dragForce / massKg;
    }

    public static double pollenDragDeceleration(double speedMps) {
        return dragDeceleration(speedMps, POLLEN_MASS_KG, POLLEN_RADIUS_M);
    }

    public static double nectarDragDeceleration(double speedMps) {
        return dragDeceleration(speedMps, NECTAR_MASS_KG, NECTAR_RADIUS_M);
    }
}