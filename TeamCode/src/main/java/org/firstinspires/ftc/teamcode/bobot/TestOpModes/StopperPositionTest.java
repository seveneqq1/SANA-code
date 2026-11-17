package org.firstinspires.ftc.teamcode.bobot.TestOpModes;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.bobot.commands.CommandOpMode;

abstract class StopperPositionTest extends CommandOpMode {
    private Servo stopper;

    @Override
    protected final void onInit() {
        stopper = hardwareMap.get(Servo.class, "stopper");
    }

    @Override
    protected final Command createCommand() {
        double position = targetPosition();
        return Command.build()
                .setExecute(() -> stopper.setPosition(position))
                .requiring(stopper);
    }

    protected abstract double targetPosition();
}
