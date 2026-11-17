package org.firstinspires.ftc.teamcode.bobot.TestOpModes;

import dev.nextftc.robot.opmode.NextTeleop;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Diagnostics;

@NextTeleop(name = "servo feed", group = OpModeMeta.DefaultGroup)
public class StopperFeedTest extends StopperPositionTest {
    @Override
    protected final double targetPosition() {
        return Diagnostics.STOPPER_FEED_POSITION;
    }
}
