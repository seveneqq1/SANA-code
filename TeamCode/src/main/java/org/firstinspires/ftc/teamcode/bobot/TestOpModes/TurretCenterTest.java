package org.firstinspires.ftc.teamcode.bobot.TestOpModes;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Servo;
import dev.nextftc.robot.opmode.NextTeleop;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.bobot.commands.CommandOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Diagnostics;

@NextTeleop(name = "turret set 0.5 both", group = OpModeMeta.DefaultGroup)
public class TurretCenterTest extends CommandOpMode {
    private Servo servo1;
    private Servo servo2;

    @Override
    protected void onInit() {
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo1 = hardwareMap.get(Servo.class, "servo1");
    }

    @Override
    protected Command createCommand() {
        return Command.build()
                .setExecute(() -> {
                    servo2.setPosition(Diagnostics.TURRET_TEST_CENTER);
                    servo1.setPosition(Diagnostics.TURRET_TEST_CENTER);
                })
                .requiring(servo1, servo2);
    }
}
