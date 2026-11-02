package org.firstinspires.ftc.teamcode.bobot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.bobot.states.Alliance;

@Autonomous(name = " Blue  Auto", group = "Regional")
public class BlueAuto extends RegionalAuto {
    @Override protected Alliance alliance() { return Alliance.BLUE; }
}
