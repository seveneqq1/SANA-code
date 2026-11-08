package org.firstinspires.ftc.teamcode.bobot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Placeholder Blue autonomous. Pedro pathing is not wired yet
 * (pedro.Constants.create() still returns null).
 */
@Autonomous(name = "Blue")
public class blue extends OpMode {
    @Override
    public void init() {
        telemetry.addLine("Blue autonomous is not written yet.");
        telemetry.addLine("Finish pedro.Constants.create() and paths first.");
        telemetry.update();
    }

    @Override
    public void loop() {
    }
}
