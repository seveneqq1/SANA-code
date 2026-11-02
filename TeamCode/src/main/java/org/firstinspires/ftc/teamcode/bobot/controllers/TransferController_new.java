package org.firstinspires.ftc.teamcode.bobot.controllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class TransferController_new {

    public static double STALL_CURRENT_MA = 9200;
    public static double STALL_TRIGGER_MS = 250;

    DcMotorEx transfer;
    public State state = State.ZERO;

    private final ElapsedTime stallTimer = new ElapsedTime();
    private boolean pastStallThreshold = false;

    public enum State {
        ZERO(0),
        TRANSFER_FULL_POWER(1),
        TRANSFER_HALF_POWER(0.5),
        PUSH_BALLS_OUT(-1),
        SUPPLYING(0.65),
        SLOW(0.15);
        public double transferPower;
        State(double transferPower) {
            this.transferPower = transferPower;
        }
    }

    public TransferController_new(HardwareMap hardwareMap) {
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setTransferPower(double power) { transfer.setPower(power); }
    public double getTransferPower() { return transfer.getPower(); }
    public double getMilliAmpsTransfer() { return transfer.getCurrent(CurrentUnit.MILLIAMPS); }

    public void setState(State state) {
        this.state = state;
        transfer.setPower(state.transferPower);
    }

     



    public boolean isStalled() {
        return pastStallThreshold && stallTimer.milliseconds() > STALL_TRIGGER_MS;
    }

    public void update() {
        boolean overCurrent = (state == State.TRANSFER_FULL_POWER || state == State.SUPPLYING)
                && getMilliAmpsTransfer() > STALL_CURRENT_MA;
        if (overCurrent && !pastStallThreshold) {
            stallTimer.reset();
        }
        pastStallThreshold = overCurrent;
    }
}