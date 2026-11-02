package org.firstinspires.ftc.teamcode.bobot.controllers;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;


@Configurable
public class ShooterControllerjana1 {

    Servo hood;
    Servo blocker;
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private VoltageSensor voltageSensor;

    public static double kV = 0.00537;
    public static double offset_tuner=0;
    public static double kP = 0.025;
    public static double kI = 0.00;
    public static double kD = 0.00;
    private double targetVelocityRPM = 0;
    private double lastError = 0;
    private double integral = 0;
    private double lastVelocity = 0;
    private long lastTime = 0;
    private BlockStates blockStates = BlockStates.BLOCK;
    public enum BlockStates {
        BLOCK(0.67),

        SUPPLY(0.47);
        final double block_position;
        BlockStates(double blockposition){
            this.block_position =blockposition;
        }
    }
    public ShooterControllerjana1(HardwareMap hardwareMap) {
        shooter2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooter2"));
        shooter1 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooter1"));
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        shooter1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        lastTime = System.nanoTime();
         hood =hardwareMap.get(Servo.class,"hood");
         blocker= hardwareMap.get(Servo.class,"block");

        blocker.setDirection(Servo.Direction.REVERSE);
        hood.setDirection(Servo.Direction.REVERSE);  
    }
    public ShooterControllerjana1(HardwareMap hardwareMap, double kvnumber) {
        shooter2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooter2"));
        shooter1 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooter1"));
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        shooter1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        lastTime = System.nanoTime();
        hood =hardwareMap.get(Servo.class,"hood");
        blocker= hardwareMap.get(Servo.class,"block");
        blocker.setDirection(Servo.Direction.REVERSE);
        hood.setDirection(Servo.Direction.FORWARD);
        kV=kvnumber;
    }
    public ShooterControllerjana1(HardwareMap hardwareMap, double kp, double huyny) {
        shooter2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooter2"));
        shooter1 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooter1"));
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        shooter1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        lastTime = System.nanoTime();
        hood =hardwareMap.get(Servo.class,"hood");
        blocker= hardwareMap.get(Servo.class,"block");
        blocker.setDirection(Servo.Direction.REVERSE);
        hood.setDirection(Servo.Direction.FORWARD);
        kP=kp;
    }
    public void setBlockerPosition(double position ){
        blocker.setPosition(position);
    }
    public void setRampPosition(double degrees){
        double degrees_of_servo= MathFunctions.clamp(-0.00777329*degrees*degrees*degrees+0.860198*degrees*degrees-25.21064*degrees+269.69355,78,190);
        double servo_pos = -0.0719014084507+((degrees_of_servo-5)/355);
        if(servo_pos>0.14 && servo_pos<=0.5) {
            hood.setPosition(servo_pos);
        }
    }
    public void setHoodPosition(double position){
        hood.setPosition(position);
    }
    public void setShooterVelocity(double targetRPM) {
        this.targetVelocityRPM = -targetRPM;
    }
    public void setSupplyState(BlockStates blockStates){
        this.blockStates = blockStates;
        blocker.setPosition(blockStates.block_position);
    }
    public double getHoodPosition(){
        return hood.getPosition();
    }
    public double getTargetPositon(){
            return Math.abs(targetVelocityRPM);
    }
    public double getCurrentVelocity(){
        return Math.abs(currentVelocity);
    }
    public boolean isShooterAtTarget(){
        return getCurrentVelocity() >=getTargetPositon()-10;
    }
    public boolean isShooterAlmsotAtTarget(){
        return getCurrentVelocity() >=getTargetPositon()-300;
    }
    public double getShooter1_Amps(){
        return  shooter1.getCurrent(CurrentUnit.MILLIAMPS);
    }
    public double getShooter2_Amps(){
        return  shooter2.getCurrent(CurrentUnit.MILLIAMPS);
    }
    public double power;
    public double currentVelocity;
    public void update() {
        if (targetVelocityRPM == 0) {
            shooter2.setPower(0);
            shooter1.setPower(0);
            integral = 0;
            lastError = 0;
            return;
        }

        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) * 1e-9;
        lastTime = currentTime;

        currentVelocity = (-shooter2.getVelocity());

        double acceleration = (currentVelocity - lastVelocity) / dt;
        lastVelocity = currentVelocity;

        double feedforwardVoltage = kV * targetVelocityRPM;

        double error = (targetVelocityRPM+offset_tuner) - currentVelocity;
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double pidVoltage = kP * error + kI * integral + kD * derivative;

        double totalVoltage = feedforwardVoltage + pidVoltage;

        double batteryVoltage = voltageSensor.getVoltage();
        power = totalVoltage / batteryVoltage;
        power = Math.max(-1.0, Math.min(1.0, power));

        shooter1.setPower(-power);
        shooter2.setPower(-power);
    }
}
