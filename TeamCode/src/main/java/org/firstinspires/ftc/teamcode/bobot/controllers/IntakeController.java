package org.firstinspires.ftc.teamcode.bobot.controllers;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PDController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Configurable
public class IntakeController {

    Motor intake,transfer;

    public ElapsedTime timer;
    public double power, MILLIAMPS_intake, MILLIAMPS_transfer;

    private State state = State.ZERO;
    public enum State {
        ZERO(0,0),
        IN_TAKING_FULL_POWER(1,1),
        FULL_POWER(1,1),
        IN_TAKING_SC_TH_BALL(1,0.1),
        GOT_INTAKED_ALL_BALL(0.4,0),
        SUPPLYING_FOR_FAR_ZONE(0.6,0.5),
        SLIGHTLY_PUSH_BALLS_OUT(0.15,-0.9),
        BALLS_OUT(-0.8,-0.8)
        ;
        final double intakePower, transferPower;
        State(double intakePower, double transferPower) {
            this.intakePower = intakePower;
            this.transferPower =transferPower;
        }
    }
    public IntakeController(HardwareMap hardwareMap) {

      intake=new Motor(hardwareMap,"intake");
      intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
      intake.setInverted(true);
      transfer=new Motor(hardwareMap,"transfer");
      transfer.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
      transfer.setInverted(true);
      timer=new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    }
    public void setIntakePower(double power){
        intake.set(power);
    }
    public void setTransferPower(double power){transfer.set(power);}
    public double getMILLIAMPS_transfer(){
        return ((DcMotorEx) transfer.motor).getCurrent(CurrentUnit.MILLIAMPS);
    }
    public double getMILLIAMPS_intake(){
        return ((DcMotorEx) intake.motor).getCurrent(CurrentUnit.MILLIAMPS);
    }
    public double getIntakeVelocity(){
        return -intake.getCorrectedVelocity();
    }
    public double getTransferVelocity(){
        return transfer.getCorrectedVelocity();
    }
    public void setState(State state){
        this.state = state;
        intake.set(-state.intakePower);
        transfer.set(state.transferPower);
    }
    public double corrected_velocity_transfer, corrected_velocity_intake;

    public void update() {
 
 
 
 
 
 
     }

    public void debug(Telemetry telemetry) {
        telemetry.addData("Target Velocity Intake", state.intakePower);
        telemetry.addData("Power intake",power);
        telemetry.addData("Power transfer", state.transferPower);
        telemetry.addData("AMPS Transfer Motor", MILLIAMPS_intake);
     }
}
