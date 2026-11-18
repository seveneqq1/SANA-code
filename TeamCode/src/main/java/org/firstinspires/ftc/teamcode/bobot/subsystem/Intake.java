package org.firstinspires.ftc.teamcode.bobot.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

@Config
public class Intake extends SubsystemBase {
    private Motor intake;
    private Motor transfer;

    public void init(HardwareMap hw) {
        intake = new Motor(hw, "intake");
        transfer = new Motor(hw, "trans");
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        intake.setInverted(true);
        transfer.setInverted(true);
        spinOff();
    }

    private void setPower(double intakePower, double transferPower) {
        intake.set(-intakePower);
        transfer.set(transferPower);
    }

    public void spinIn() {
        setPower(1.0, 1.0);
    }

    public void spinOut() {
        setPower(-0.8, -0.8);
    }

    public void spinOff() {
        setPower(0.0, 0.0);
    }

    public void spinShoot() {
        setPower(1.0, 1.0);
    }

    public void spinSlowShoot() {
        setPower(0.6, 0.55);
    }

    public InstantCommand in() {
        return new InstantCommand(this::spinIn, this);
    }

    public InstantCommand out() {
        return new InstantCommand(this::spinOut, this);
    }

    public InstantCommand off() {
        return new InstantCommand(this::spinOff, this);
    }

    public InstantCommand shoot() {
        return new InstantCommand(this::spinShoot, this);
    }

    public InstantCommand slowShoot() {
        return new InstantCommand(this::spinSlowShoot, this);
    }
}
