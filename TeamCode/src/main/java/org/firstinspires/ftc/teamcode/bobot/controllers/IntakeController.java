package org.firstinspires.ftc.teamcode.bobot.controllers;

/**
 * Intake and transfer motors that feed balls into the shooter.
 * Config names: "intake" and "trans".
 */

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

@Configurable
public class IntakeController {
    // Multiplies only FEEDING transfer power; collection and eject keep their presets.
    public static double TRANSFER_SHOOTING_SPEED_SCALE = 0.967;
    private final Motor intake, transfer;
    public enum State{
        STOP(0, 0),
        FULL_POWER(1, 1),
        FEEDING(0.5, 0.8), // old feeding speed
        BALLS_OUT(-0.8, -0.8);

        final double intakePower;
        final double transferPower;
        State(double intakePower, double transferPower){
            this.intakePower = intakePower;
            this.transferPower = transferPower;
        }
    }

    public IntakeController(HardwareMap hardwareMap){
        intake = new Motor(hardwareMap, "intake");
        transfer = new Motor(hardwareMap, "trans");

        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        intake.setInverted(true);
        transfer.setInverted(true);
        stop();
    }
    public void setState(State state) {
        intake.set(-state.intakePower);
        double scale = state == State.FEEDING ? TRANSFER_SHOOTING_SPEED_SCALE : 1.0;
        transfer.set(state.transferPower * scale);
    }

    public void stop() {
        setState(State.STOP);
    }

}
