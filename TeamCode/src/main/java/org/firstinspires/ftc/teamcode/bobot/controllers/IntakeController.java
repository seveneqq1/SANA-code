package org.firstinspires.ftc.teamcode.bobot.controllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class IntakeController {
    DcMotorEx intake;
    public State state = State.ZERO;
    public enum State {
        ZERO(0),
        INTAKING_FULL_POWER(1),
        PUSH_BALLS_OUT(-1),
        SUPPLYING(0.8),
        SLIGHTLY_PUSH_BALLS_OUT(-0.5);
        final double intakePower;
        State(double intakePower){
            this.intakePower = intakePower;
        }
    }
    public IntakeController(HardwareMap hardwareMap){
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setIntakePower(double power) { intake.setPower(power); }
    public double getIntakePower() { return intake.getPower(); }
    public double getMilliAmpsIntake() { return intake.getCurrent(CurrentUnit.MILLIAMPS); }
    public void setState(State state){
        this.state = state;
        intake.setPower(state.intakePower);
    }
    public void update(){}
}
