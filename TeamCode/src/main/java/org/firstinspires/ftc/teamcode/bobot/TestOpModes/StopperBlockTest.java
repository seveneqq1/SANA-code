package org.firstinspires.ftc.teamcode.bobot.TestOpModes;

import dev.nextftc.robot.opmode.NextTeleop;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Diagnostics;

@NextTeleop(name = "servo block", group = OpModeMeta.DefaultGroup)
public class StopperBlockTest extends StopperPositionTest {
    @Override
    protected double targetPosition() {
        return Diagnostics.STOPPER_BLOCK_POSITION;
    }
}
