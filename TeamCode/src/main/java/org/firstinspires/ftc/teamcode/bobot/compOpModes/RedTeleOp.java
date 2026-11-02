package org.firstinspires.ftc.teamcode.bobot.compOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.bobot.states.Alliance;

@TeleOp(name = "Red", group = "Regional")
public class RedTeleOp extends RegionalTeleOp {
    @Override protected Alliance alliance() { return Alliance.RED; }
}
