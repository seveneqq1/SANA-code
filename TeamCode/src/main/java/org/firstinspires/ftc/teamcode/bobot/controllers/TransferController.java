package org.firstinspires.ftc.teamcode.bobot.controllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class TransferController {
    DcMotorEx transfer;
    public State state = State.ZERO;
    public enum State {
        ZERO(0),
        TRANSFER_FULL_POWER(1),
        TRANSFER_HALF_POWER(0.5),
        PUSH_BALLS_OUT(-1),
        SUPPLYING(0.8),
        SLIGHTLY_PUSH_BALLS_OUT(-0.5);
        public double transferPower;
        State(double transferPower){
            this.transferPower = transferPower;
        }
    }
    public TransferController(HardwareMap hardwareMap){
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setTransferPower(double power) { transfer.setPower(power); }
    public double getTransferPower() { return transfer.getPower(); }
    public double getMilliAmpsTransfer() { return transfer.getCurrent(CurrentUnit.MILLIAMPS); }
    public void setState(State state){
        this.state = state;
        transfer.setPower(state.transferPower);
    }
    public void update(){
    }
}
