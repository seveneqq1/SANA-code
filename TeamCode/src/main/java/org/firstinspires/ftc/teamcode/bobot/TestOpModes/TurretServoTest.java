package org.firstinspires.ftc.teamcode.bobot.TestOpModes;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Servo;
import dev.nextftc.robot.opmode.NextTeleop;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.bobot.commands.CommandOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Diagnostics;

@NextTeleop(name = "Turret Servo Test", group = OpModeMeta.DefaultGroup)
public class TurretServoTest extends CommandOpMode {
    private Servo servo1;
    private Servo servo2;
    private double position = Diagnostics.TURRET_TEST_CENTER;
    private long nextStepNanos;

    @Override
    protected void onInit() {
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
    }

    @Override
    protected void onStart() {
        nextStepNanos = System.nanoTime();
    }

    @Override
    protected Command createCommand() {
        return Command.build()
                .setExecute(this::adjustPosition)
                .requiring(servo1, servo2);
    }

    private void adjustPosition() {
        if (System.nanoTime() - nextStepNanos < 0) {
            return;
        }
        if (gamepad1.dpad_left) {
            position += Diagnostics.TURRET_TEST_STEP;
        }
        if (gamepad1.dpad_right) {
            position -= Diagnostics.TURRET_TEST_STEP;
        }
        position = Math.max(0, Math.min(1, position));
        if (!gamepad1.circle) {
            servo1.setPosition(position);
        }
        if (!gamepad1.square) {
            servo2.setPosition(position);
        }
        robotTelemetry.addLine("D-pad left/right = move both");
        robotTelemetry.addLine("Hold Square = only servo1, hold Circle = only servo2");
        robotTelemetry.addData("Position", "%.3f", position);
        robotTelemetry.addData("servo1 port", servo1.getPortNumber());
        robotTelemetry.addData("servo2 port", servo2.getPortNumber());
        robotTelemetry.update();
        nextStepNanos = System.nanoTime() + TimeUnit.MILLISECONDS.toNanos(
                Diagnostics.TURRET_TEST_INTERVAL_MS);
    }
}
