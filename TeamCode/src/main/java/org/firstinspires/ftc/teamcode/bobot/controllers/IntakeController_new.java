package org.firstinspires.ftc.teamcode.bobot.controllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class IntakeController_new {

     
     
    public static double STALL_CURRENT_MA = 9200;
    public static double STALL_TRIGGER_MS = 250;

    DcMotorEx intake;
    public State state = State.ZERO;
    Servo fl;
    private final ElapsedTime stallTimer = new ElapsedTime();
    private boolean pastStallThreshold = false;

    public enum State {
        ZERO(0,0),
        INTAKING_FULL_POWER(1,0),
        PUSH_BALLS_OUT(-1,0),
        SUPPLYING(1,0),
        SLIGHTLY_PUSH_BALLS_OUT(-0.5,0),
        INTAKE_FLOWER(1,1);
        final double intakePower, flPower;
        State(double intakePower, double flPower) {
            this.flPower = flPower;
            this.intakePower = intakePower;
        }
    }

    public IntakeController_new(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        fl = hardwareMap.get(Servo.class, "fl");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setIntakePower(double power) { intake.setPower(power); }
    public void setFlPos(double pos) { fl.setPosition(pos); }
    public double getIntakePower() { return intake.getPower(); }
    public double getMilliAmpsIntake() { return intake.getCurrent(CurrentUnit.MILLIAMPS); }

    public void setState(State state) {
        this.state = state;
        intake.setPower(state.intakePower);
    }

     





    public boolean isStalled() {
        return pastStallThreshold && stallTimer.milliseconds() > STALL_TRIGGER_MS;
    }

    public void update() {
        boolean overCurrent = state == State.INTAKING_FULL_POWER && getMilliAmpsIntake() > STALL_CURRENT_MA;
        if (overCurrent && !pastStallThreshold) {
            stallTimer.reset();
        }
        pastStallThreshold = overCurrent;
    }
}