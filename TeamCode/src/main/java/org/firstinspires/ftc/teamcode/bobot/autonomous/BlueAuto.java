package org.firstinspires.ftc.teamcode.bobot.autonomous;

import dev.nextftc.robot.opmode.NextAutonomous;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.bobot.commands.CommandOpMode;

@NextAutonomous(name = "nO", group = OpModeMeta.DefaultGroup)
public class BlueAuto extends CommandOpMode {
    @Override
    protected void onInit() {
        robotTelemetry.addLine("Blue autonomous is not written yet.");
        robotTelemetry.addLine("Configure the Pedro follower and paths first.");
        robotTelemetry.update();
    }
}
