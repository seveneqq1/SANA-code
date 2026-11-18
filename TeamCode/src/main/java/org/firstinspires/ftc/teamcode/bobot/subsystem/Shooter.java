package org.firstinspires.ftc.teamcode.bobot.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@Config
public class Shooter extends SubsystemBase {
    public Shooter(){}
    private MotorEx l, r;
    private Servo stopper;
    private final PIDController pid = new PIDController(kP, kI, kD);
    private double previousTarget;
    public static double blockedPosition = 0.35, feedingPosition = 0.5;
    public static double kP = 0.0005, kI = 0.0, kD = 0.0;
    public static double kS = 0.025, kV = 0.00038;
    public static double baseVel = 2480;
    public static boolean volComp = false;
    public static boolean on = true;
    public void init(HardwareMap hw){
        l = new MotorEx(hw, "shooter1");
        r = new MotorEx(hw, "shooter2");
        r.setInverted(true);
        stopper = hw.get(Servo.class, "stopper");
        setFeeding(false);
    }
    public void set(double p){
        l.set(p);
        r.set(p);
    }
    public double getTarget(){
        return baseVel;
    }
    public double getVel(){
        return r.getVelocity();
    }
    public double getAccel(){
        return r.getAcceleration();
    }
    public void turnOff(){
        pid.reset();
        set(0);
        setFeeding(false);
        on = false;
    }

    public void setFeeding(boolean feeding) {
        stopper.setPosition(feeding ? feedingPosition : blockedPosition);
    }
    public void turnOn(){
        on = true;
    }
    public boolean isOn(){
        return on;
    }

    @Override
    public void periodic() {
        if (on) {
            double target = getTarget();
            double velocity = Math.abs(getVel());
            if (!Double.isFinite(target) || target <= 0 || !Double.isFinite(velocity)) {
                pid.reset();
                set(0);
                return;
            }
            if (target != previousTarget) {
                pid.reset();
                previousTarget = target;
            }
            pid.setPID(kP, kI, kD);
            double power = pid.calculate(velocity, target) + kS + kV * target;
            set(Double.isFinite(power) ? Math.max(0, Math.min(1, power)) : 0);
        }
    }
    public InstantCommand on(){return new InstantCommand(this::turnOn);}
    public InstantCommand off(){return new InstantCommand(this::turnOff);}
}
