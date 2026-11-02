package org.firstinspires.ftc.teamcode.bobot.compOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.bobot.states.Alliance;

@TeleOp(name = "Blue", group = "Regional")
public class BlueTeleOp extends RegionalTeleOp {
    @Override protected Alliance alliance() { return Alliance.BLUE; }
}
