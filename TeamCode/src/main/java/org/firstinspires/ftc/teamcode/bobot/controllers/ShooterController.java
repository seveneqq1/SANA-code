package org.firstinspires.ftc.teamcode.bobot.controllers;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PDController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@Configurable
public class ShooterController {

    public Motor shooter1, shooter2;
    Servo hood;
    Servo blocker;
    public static double kP=0.008;
    public static double kD=0;
    private final PDController PDController;
    public enum ShooterState {
        NEAR_ZONE(1550),
        FAR_ZONE(3000),
        ZERO(0)
       ;
        final double targetVelocity;
        ShooterState(double targetVelocity) {
            this.targetVelocity =targetVelocity;
        }
    }
    private ShooterState shooterState=ShooterState.ZERO;
    private BlockStates blockStates = BlockStates.BLOCK;
    public enum BlockStates {
        BLOCK(0.57),

        SUPPLY(0.37);
        final double block_position;
        BlockStates(double blockposition){
            this.block_position =blockposition;
        }
    }
    public ShooterController(HardwareMap hardwareMap) {
      PDController =new PDController(kP,kD);
      PDController.setTolerance(10,5);
      shooter1 =new Motor(hardwareMap,"shooter1");
      shooter2 =new Motor(hardwareMap,"shooter2");

      hood =hardwareMap.get(Servo.class,"hood");
      blocker= hardwareMap.get(Servo.class,"block");
    }
    public void setShooterPower(double power){
        shooter1.set(power);
        shooter2.set(power);
    }
    public void setBlockerPosition(double positoin){
        blocker.setPosition(positoin);
    }
    public boolean angle=false;
    public void setRampPosition(double degrees){
        double degrees_of_servo= MathFunctions.clamp(-0.00777329*degrees*degrees*degrees+0.860198*degrees*degrees-25.21064*degrees+269.69355,78,190);
        double servo_pos = -0.0719014084507+(degrees_of_servo/355);
        if(servo_pos>0.14 && servo_pos<=0.5) {
            hood.setPosition(servo_pos);
        }
    }
    public void setHoodPosition(double position){
        angle=true;
        hood.setPosition(position);
    }
    public void setShootState(ShooterState shooterState){
        state=true;
        this.shooterState=shooterState;
    }
    public double getCorrectedVelocityOfTurret(){
        return shooter1.getCorrectedVelocity();
    }
    private boolean state=false;
    public void setShooterVelocity(double velocity){
        state=false;
        shooter_target =velocity;
    }
    public void setSupplyState(BlockStates blockStates){
        this.blockStates = blockStates;
        blocker.setPosition(blockStates.block_position);
    }
    public double getHoodPosition(){
        return hood.getPosition();
    }
    public double shooterAMPS;
    public double shooter_corrected_velcoity;
    public double current_Velocity;
    public double motor1Power;
    double shooter_target =0;
    public double getTargetPositon(){
        if(state){
        return shooterState.targetVelocity;}
        else {
            return shooter_target;
        }
    }
    public boolean isShooterAtTarget(){
        if(!state){
            return current_Velocity >= shooter_target-110;
        }
        else{
            return current_Velocity >=shooterState.targetVelocity-110;
        }
    }
    public double getCurrentAMPS(){
        return ((DcMotorEx) shooter2.motor).getCurrent(CurrentUnit.MILLIAMPS)*2;
    }
    public boolean isShooterAlmostAtTarget(){
        if(!state){
            return current_Velocity >=shooter_target-350;
        }
        else{
            return current_Velocity >=shooterState.targetVelocity-350;
        }
    }
    public double getShooter1Ampers(){
        return((DcMotorEx) shooter2.motor).getCurrent(CurrentUnit.MILLIAMPS);
    }
    public double getShooter2Ampers(){
        return((DcMotorEx) shooter1.motor).getCurrent(CurrentUnit.MILLIAMPS);
    }
    public void update() {
        PDController.setPID(kP,0,kD);
        current_Velocity = shooter2.getCorrectedVelocity();
        if(state){
        motor1Power = PDController.calculate(current_Velocity,shooterState.targetVelocity);}
        if(!state){
            motor1Power = PDController.calculate(current_Velocity,shooter_target);
        }
        if(motor1Power<-0.1){
            motor1Power=-0.1;
        }
        shooter1.set(-motor1Power);
        shooter2.set(-motor1Power);
        shooterAMPS =((DcMotorEx) shooter2.motor).getCurrent(CurrentUnit.MILLIAMPS)*2;
    }

    public void debug(Telemetry telemetry) {
        telemetry.addData("POWER", motor1Power);
        telemetry.addData("Current AMPS of Shooter", shooterAMPS);
        telemetry.addData("LEFT shooter CORRECTED VELOCITY", shooter1.getCorrectedVelocity());
        telemetry.addData("RIGHT shooter CORRECTED VELOCITY", shooter2.getCorrectedVelocity());
    }
}
