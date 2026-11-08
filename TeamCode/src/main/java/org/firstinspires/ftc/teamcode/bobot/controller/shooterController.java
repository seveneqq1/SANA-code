package org.firstinspires.ftc.teamcode.bobot.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PDController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

@Config
public class shooterController {
    public Motor shooter1, shooter2;
    public Servo stopper;
    public static double kP = 0.008;
    public static double kD = 0;

    public static double baseVelocity = 2400;
    public static double velocityTolerance = 120;

    public static double blockedPosition = 0.5;
    public static double feedingPosition = 0.05;

    private double targetVelocity;
    private double currentVelocity;

    private final PDController controller;
    public shooterController(HardwareMap hardwareMap){
        shooter1 = new Motor(hardwareMap, "shooter1");
        shooter2 = new Motor(hardwareMap, "shooter2");
        stopper = hardwareMap.get(Servo.class, "stopper");
        controller = new PDController(kP, kD);
        closeStopper();
    }

    public void setTargetVelocity(double velocity){
        targetVelocity = Math.max(0, velocity);
    }

    public void stop(){
        setTargetVelocity(0);
    }

    public void setStopperPos(double position){
        stopper.setPosition(position);
    }

    public void feedStopper(){
        stopper.setPosition(feedingPosition);
    }

    public void closeStopper(){
        stopper.setPosition(blockedPosition);
    }

    public void update(){
        controller.setPID(kP, 0, kD);
        currentVelocity = shooter2.getCorrectedVelocity();

        double power = 0;

        if(targetVelocity > 0){
            power = controller.calculate(currentVelocity, targetVelocity);
            if(power > 1) power = 1;
            if(power < 0) power = 0;
        }

        shooter1.set(power);
        shooter2.set(power);
    }

    public boolean isAtTarget(){
        return Math.abs(currentVelocity - targetVelocity) < velocityTolerance;
    }

    public boolean isAlmostAtTarget(){
        return Math.abs(currentVelocity - targetVelocity) < velocityTolerance * 2.5;
    }
}
