package org.firstinspires.ftc.teamcode.bobot.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

@Config
public class intakeController {
    Motor intake, transfer;
    public ElapsedTime timer;
    private State state = State.ZERO;
    public enum State{
        ZERO(0, 0),
        STOP(0, 0),
        FULL_POWER(1, 1),
        FEEDING(0.5, 0.8),
        BALLS_OUT(-0.8, -0.8);

        final double intakePower;
        final double transferPower;
        State(double intakePower, double transferPower){
            this.intakePower = intakePower;
            this.transferPower = transferPower;
        }
    }

    public intakeController(HardwareMap hardwareMap){
        intake = new Motor(hardwareMap, "intake");
        transfer = new Motor(hardwareMap, "trans");
        timer = new ElapsedTime();

        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        intake.setInverted(true);
        transfer.setInverted(true);
        stop();
    }
    public void setState(State state) {
        this.state = state;

        intake.set(-state.intakePower);
        transfer.set(state.transferPower);
    }

    public void stop() {
        setState(State.STOP);
    }

    public State getState() {
        return state;
    }
}
